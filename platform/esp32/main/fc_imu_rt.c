// Единая realtime-цепочка FloatCore (ТЗ v0.6D §8).
//
// До v0.6D их было две: mock-задача задавала ритм контура, а отдельная задача
// читала физический датчик. Две независимые задачи с одинаковым периодом на
// одном ядре систематически просыпались на одном тике и конкурировали — это
// измерено и описано в docs/realtime_timing.md. Теперь цепочка одна:
//
//     опрос датчика → валидация → AHRS → callback Refloat → Motor Gate
//
// Ритм контура задаёт сам датчик: одна принятая физическая выборка — ровно
// одна итерация контура. Дубликаты в контур не идут.
//
// Период опроса — 1 мс, при частоте выдачи датчика 562.5 Гц (1778 мкс).
// Оверсемплинг здесь не роскошь, а необходимость: FIFO не используется, а
// вывод DRDY не разведён, поэтому единственная защита от потери семпла —
// опрашивать чаще, чем датчик выдаёт. Запас 778 мкс на опоздание пробуждения
// выбран по измеренному джиттеру планировщика (p99.9 периода контура на
// v0.6A доходил до 2840 мкс при номинале 2000, то есть +840). Каждая потеря
// всё равно была бы видна: интервал между принятыми семплами удвоился бы, и
// тракт считает это как suspected_skip.
//
// Что здесь НЕ делается: ни перестановок осей, ни инверсий знаков, ни
// компенсации монтажного наклона. Оси датчика уходят в Refloat как есть —
// обоснование тождественности преобразования в docs/imu_orientation_mapping.md.

#include "fc_platform.h"

#include "../../../compat/imu/fc_imu_pipeline.h"
#include "../../../compat/safety/fc_imu_health.h"
#include "../../../compat/safety/fc_supervisor.h"
#include "../drivers/icm20948.h"
#include "fc_imu_cal_store.h"

#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

static const char *TAG = "imu_rt";

// Период итерации контура: 2 тика FreeRTOS = 2000 мкс = 500 Гц.
//
// Выбор обоснован измерением, а не желаемой цифрой.
//
// Регистры данных ICM-20948 обновляются на внутренней частоте 1.125 кГц
// НЕЗАВИСИМО от SMPLRT_DIV: делитель управляет флагом готовности и FIFO, а не
// содержимым регистров. Это измерено на живой плате — при чтении 1619 раз/с
// слова движения менялись 1126 раз/с (docs/real_imu_refloat_integration.md).
// Документация v0.6A приписывала регистрам 562.5 Гц; это было неверно.
//
// Следствия:
//
//  * дедупликация по содержимому регистров не может отметить границу семпла:
//    при опросе медленнее 1125 Гц каждое чтение и так свежее. Она остаётся —
//    но уже как детектор замершего датчика, а не как источник ритма;
//  * ритм задаёт платформа, а поток 1125 Гц децимируется. Ни одно чтение не
//    обрабатывается дважды: за одну итерацию делается ровно одно чтение, и
//    его результат используется ровно один раз;
//  * алиасинга децимация не вносит: цифровой ФНЧ датчика ограничивает полосу
//    196.6 Гц по гироскопу при Найквисте 250 Гц. По акселерометру полоса
//    246 Гц — на грани, и это отмечено как открытый вопрос тюнинга, а не
//    замолчано.
//
// Почему 500, а не 1000 Гц: при 1 тике итерация не укладывается в период.
// Измерено: чтение 700 мкс плюс исполнение контура 260 мкс, и цикл сваливался
// на свободный ход с фактическими 768 Гц. 500 Гц — тот же номинал, что у
// главного потока Refloat (MAIN_THREAD_FREQ, main.c:61), и вдвое ниже частоты
// обновления регистров, так что каждое чтение гарантированно новое.
#define FC_IMU_RT_POLL_TICKS 2
#define FC_IMU_RT_CONTROL_HZ (configTICK_RATE_HZ / FC_IMU_RT_POLL_TICKS)

// Сколько подряд принятых семплов считать признаком «датчик стабилен».
//
// До этого момента imu_startup_done() возвращает false, Refloat остаётся в
// STATE_STARTUP, а супервизор не может уйти в READY. 250 семплов при 500 Гц —
// это 0.5 с: достаточно, чтобы фильтр Mahony сошёлся из начальной ориентации,
// и заметно меньше, чем пауза 1.5 с, которую держал mock.
#define FC_IMU_RT_STARTUP_SAMPLES 250

// Политика восстановления связи с датчиком (ТЗ v0.6D §19).
//
// Одиночная неудачная транзакция — не повод переинициализировать датчик:
// диагностика её и так учтёт, а реинициализация стоит 140 мс полной слепоты.
// Переинициализация запускается только после длинной серии неудач подряд и не
// чаще одного раза в две секунды. Отказ супервизора при этом НЕ снимается:
// поднять его обратно может только явное действие оператора.
#define FC_IMU_RT_REINIT_AFTER_FAILURES 200
#define FC_IMU_RT_REINIT_MIN_INTERVAL_US 2000000ULL

static struct {
    TaskHandle_t task;
    volatile bool run;
    volatile bool startup_done;
    bool driver_ok;

    void (*callback)(float *acc, float *gyro, float *mag, float dt);

    icm20948_config_t cfg;

    uint64_t accepted_streak;
    uint32_t consecutive_failures;
    uint64_t reinits;
    uint64_t last_reinit_us;
    uint32_t max_read_us;
    uint64_t iterations;

    volatile int stall_ms;
    float ahrs_kp;
    float ahrs_decay;
    FcImuCalStatus cal_status;
} R;

int fc_imu_rt_cal_status(void) {
    return (int) R.cal_status;
}

void fc_imu_rt_set_cal_status(int s) {
    R.cal_status = (FcImuCalStatus) s;
}

// ------------------------------------------------------------------ доступ

bool fc_imu_rt_available(void) {
    return R.driver_ok;
}

bool fc_imu_rt_running(void) {
    return R.task != NULL;
}

bool fc_imu_rt_startup_done(void) {
    return R.startup_done;
}

int fc_imu_rt_rate_hz(void) {
    // Частота, с которой Refloat ФАКТИЧЕСКИ получает семплы, а не частота
    // датчика: именно её Refloat использует как sample rate. Ноль здесь
    // недопустим — Refloat трактует его как прошивку 6.02 и подставляет
    // выдуманные 620 Гц (main.c:1186-1190).
    return FC_IMU_RT_CONTROL_HZ;
}

void fc_imu_rt_set_callback(void (*cb)(float *acc, float *gyro, float *mag, float dt)) {
    R.callback = cb;
}

uint64_t fc_imu_rt_iterations(void) {
    return R.iterations;
}

uint32_t fc_imu_rt_max_read_us(void) {
    return R.max_read_us;
}

uint64_t fc_imu_rt_reinits(void) {
    return R.reinits;
}

uint32_t fc_imu_rt_stack_watermark(void) {
    return R.task ? (uint32_t) uxTaskGetStackHighWaterMark(R.task) : 0;
}

void fc_imu_rt_inject_stall(int ms) {
    R.stall_ms = ms;
}

// -------------------------------------------------------------------- задача

static void imu_rt_task(void *arg) {
    (void) arg;
    esp_task_wdt_add(NULL);

    TickType_t next = xTaskGetTickCount();

    while (R.run) {
        vTaskDelayUntil(&next, FC_IMU_RT_POLL_TICKS);

        if (R.stall_ms) {
            // Проверка watchdog: контур намеренно перестаёт отмечаться.
            int ms = R.stall_ms;
            R.stall_ms = 0;
            vTaskDelay(pdMS_TO_TICKS(ms));
            next = xTaskGetTickCount();
            continue;
        }

        int64_t t0 = esp_timer_get_time();
        icm20948_sample_t s;
        esp_err_t err = icm20948_read(&s);
        int64_t t1 = esp_timer_get_time();
        uint32_t dur = (uint32_t) (t1 - t0);
        if (dur > R.max_read_us) {
            R.max_read_us = dur;
        }
        ++R.iterations;
        fc_timing_tick(FC_TIMING_IMU_READ);

        // Параметры AHRS берутся из того же хранилища, что читает Refloat, и
        // применяются при изменении: он перезаписывает их при инициализации
        // (main.c:210-214), уже после того, как эта задача запущена.
        float kp = fc_cfg_imu_mahony_kp();
        float decay = fc_cfg_imu_accel_confidence_decay();
        if (kp != R.ahrs_kp || decay != R.ahrs_decay) {
            R.ahrs_kp = kp;
            R.ahrs_decay = decay;
            fc_ahrs_configure((FcAhrs *) fc_imu_pipeline_ahrs(), kp, decay);
        }

        FcImuPipeVerdict v = fc_imu_pipeline_submit(
            err == ESP_OK, s.raw, s.accel_g, s.gyro_dps, s.temperature_c, (uint64_t) t1
        );

        if (err == ESP_OK) {
            R.consecutive_failures = 0;
        } else if (++R.consecutive_failures >= FC_IMU_RT_REINIT_AFTER_FAILURES) {
            uint64_t now = (uint64_t) t1;
            if (now - R.last_reinit_us >= FC_IMU_RT_REINIT_MIN_INTERVAL_US) {
                R.last_reinit_us = now;
                ++R.reinits;
                R.consecutive_failures = 0;
                // Реинициализация не «кормит» супервизора выдуманными
                // отметками: пока датчик не отдаст валидный семпл, отказ
                // остаётся, и снять его может только оператор.
                ESP_LOGW(TAG, "серия отказов чтения — переинициализация датчика (#%llu)",
                         (unsigned long long) R.reinits);
                icm20948_config_t cfg = R.cfg;
                icm20948_init(&cfg);
                esp_task_wdt_reset();
                next = xTaskGetTickCount();
            }
        }

        if (v != FC_IMU_PIPE_ACCEPTED) {
            // Ни дубликат, ни отвергнутый, ни несостоявшийся семпл в Refloat
            // не идут, и итерацией контура не считаются.
            esp_task_wdt_reset();
            continue;
        }

        // --- принятый физический семпл: ровно одна итерация контура --------
        FcImuSample sample = fc_imu_pipeline_sample();

        fc_timing_tick(FC_TIMING_CONTROL);
        fc_supervisor_report_loop_tick(sample.timestamp_us);
        fc_supervisor_report_imu_sample(sample.timestamp_us);

        // Контур не запускается, пока не измерен остаток смещения гироскопа.
        // Само измерение ничего не компенсирует (постоянная калибровка уже
        // всё вычла), но оно требует неподвижной доски — и тем самым остаётся
        // единственной защитой от старта контура на движущейся доске.
        if (!R.startup_done && ++R.accepted_streak >= FC_IMU_RT_STARTUP_SAMPLES &&
            fc_imu_pipeline_residual_ready()) {
            R.startup_done = true;
            FcImuPipelineStats ps = fc_imu_pipeline_stats();
            ESP_LOGI(TAG,
                     "IMU стабилен, остаток смещения %s: %+.2f %+.2f %+.2f °/с, контур запущен",
                     fc_imu_residual_state_name(ps.residual_state),
                     (double) ps.residual_bias_dps[0], (double) ps.residual_bias_dps[1],
                     (double) ps.residual_bias_dps[2]);
        }

        void (*cb)(float *, float *, float *, float) = R.callback;
        if (cb && R.startup_done) {
            fc_timing_exec_begin(FC_TIMING_CONTROL);
            // Копии: Refloat получает буферы, которые не изменятся под ним.
            float acc[3], gyro[3];
            memcpy(acc, sample.accel_g, sizeof(acc));
            memcpy(gyro, sample.gyro_rad_s, sizeof(gyro));
            cb(acc, gyro, NULL, sample.dt_s);
            fc_timing_exec_end(FC_TIMING_CONTROL);
        }
        esp_task_wdt_reset();
    }

    esp_task_wdt_delete(NULL);
    R.task = NULL;
    vTaskDelete(NULL);
}

// --------------------------------------------------------------- жизненный цикл

bool fc_imu_rt_start(void) {
    void (*cb)(float *, float *, float *, float) = R.callback;
    memset(&R, 0, sizeof(R));
    R.callback = cb;

    R.cfg = icm20948_default_config();
    esp_err_t err = icm20948_init(&R.cfg);
    // Драйвер мог разрешить адрес опросом — забираем фактическую конфигурацию.
    R.cfg = *icm20948_active_config();

    FcImuHealthConfig hc = fc_imu_health_config_for(
        icm20948_accel_fs_g(R.cfg.accel_fs), icm20948_gyro_fs_dps(R.cfg.gyro_fs)
    );
    uint32_t nominal_us = 1000000u / FC_IMU_RT_CONTROL_HZ;
    FcImuPipelineConfig pc = fc_imu_pipeline_default_config(nominal_us);
    fc_imu_pipeline_init(&pc, &hc);

    // Постоянная калибровка читается ОДИН раз при старте и применяется ко
    // всему потоку. Автоматической калибровки при каждой загрузке нет
    // намеренно: в штатной прошивке VESC её тоже нет, а измерять ориентацию
    // при включении означало бы принимать за ноль то положение, в котором
    // доску случайно оставили.
    FcImuCalibration cal;
    R.cal_status = fc_imu_cal_store_load(&cal);
    fc_imu_pipeline_set_calibration(&cal);
    ESP_LOGI(TAG, "калибровка: %s, поворот %+.2f %+.2f %+.2f град, смещения %+.3f %+.3f %+.3f °/с",
             fc_imu_cal_status_name(R.cal_status), (double) cal.rot_roll_deg,
             (double) cal.rot_pitch_deg, (double) cal.rot_yaw_deg,
             (double) cal.gyro_offset_dps[0], (double) cal.gyro_offset_dps[1],
             (double) cal.gyro_offset_dps[2]);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ICM-20948 не инициализирован: %s (шаг: %s)", esp_err_to_name(err),
                 icm20948_last_stage());
        // Подмены источника НЕ происходит: контур просто не запускается.
        // imu_startup_done() остаётся false, Refloat стоит в STATE_STARTUP,
        // супервизор не может уйти в READY. Молча подставить mock было бы
        // худшим из возможных решений.
        R.driver_ok = false;
    } else {
        uint8_t who = 0;
        icm20948_who_am_i(&who);
        ESP_LOGI(TAG, "ICM-20948 на 0x%02x, WHO_AM_I = 0x%02x, контур %.0f Гц, период %" PRIu32 " мкс",
                 R.cfg.i2c_addr, who, (double) FC_IMU_RT_CONTROL_HZ, nominal_us);
        R.driver_ok = true;
    }

    R.run = true;
    fc_timing_set_nominal(FC_TIMING_CONTROL, nominal_us);
    fc_timing_set_nominal(FC_TIMING_IMU_READ, nominal_us);

    xTaskCreatePinnedToCore(
        imu_rt_task, "fc_imu_rt", 5120, NULL, FC_PRIO_IMU, &R.task, FC_CORE_REALTIME
    );
    return R.driver_ok;
}

void fc_imu_rt_stop(void) {
    R.run = false;
}
