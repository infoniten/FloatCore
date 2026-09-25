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

#include "fc_gap_port.h"
#include "fc_log_port.h"
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
#include "esp_freertos_hooks.h"
#include "fc_rt_clock.h"
#include "freertos/semphr.h"
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
#if FC_LAB_DIAGNOSTICS
    volatile int accel_low_n;
#endif
    uint32_t max_read_us;
    uint64_t i2c_calls;
    uint32_t i2c_hist[FC_IMU_I2C_HIST_BINS];
    uint64_t i2c_wall_sum_us;
    uint64_t i2c_others_sum_us;
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

#if FC_DIAG_LOOP_PROBES
// Суммарное время, которое получили другие задачи ядра 1: главный поток
// Refloat, простой ядра и супервизор. Вспомогательный поток Refloat живёт на
// ядре 0 и сюда не входит. Счётчик FreeRTOS — в микросекундах esp_timer.
static uint32_t others_runtime(void) {
    uint32_t sum = 0;
    TaskHandle_t h[3] = {(TaskHandle_t) fc_thread_handle(0), xTaskGetIdleTaskHandleForCore(1),
                         (TaskHandle_t) fc_supervisor_task_handle()};
    for (int i = 0; i < 3; ++i) {
        if (h[i]) {
            // Без подсчёта свободного стека и с заданным состоянием: оба
            // вычисления дорогие, а нужен только счётчик времени.
            TaskStatus_t st;
            vTaskGetInfo(h[i], &st, pdFALSE, eRunning);
            sum += (uint32_t) st.ulRunTimeCounter;
        }
    }
    return sum;
}
#endif

FcImuI2cSplit fc_imu_rt_i2c_split(void) {
    FcImuI2cSplit r = {0};
    r.calls = R.i2c_calls;
    memcpy(r.hist, R.i2c_hist, sizeof(r.hist));
    if (r.calls) {
        r.wall_mean_us = (double) R.i2c_wall_sum_us / (double) r.calls;
        r.others_mean_us = (double) R.i2c_others_sum_us / (double) r.calls;
        r.own_mean_us = r.wall_mean_us - r.others_mean_us;
    }
    return r;
}

void fc_imu_rt_i2c_split_reset(void) {
    R.i2c_calls = 0;
    memset(R.i2c_hist, 0, sizeof(R.i2c_hist));
    R.i2c_wall_sum_us = 0;
    R.i2c_others_sum_us = 0;
}

// Перекрёстный снимок пробуждения (ТЗ v0.9I §20).
//
// Пробуждение задачи датчика дрожит: через раз оно опаздывает примерно на
// 680 мкс после тика. Выше неё на ядре 1 нет задач, работающих с такой
// частотой, поэтому подозревается закрытое прерывание — критическая секция,
// попавшая на тик. Здесь в момент каждого пробуждения записывается, в каком
// вызове интерфейса стоит главный поток Refloat, отдельно для поздних и для
// своевременных пробуждений. Если поздние стабильно совпадают с одним вызовом,
// а своевременные — нет, виновник назван.
#define WAKE_SNAP_SLOTS 12
// Время последнего тика FreeRTOS на КАЖДОМ ядре. В многоядерном IDF счётчик
// тиков увеличивает только ядро 0 (port_systick.c), а ядро 1 на своём тике
// лишь подбирает разбуженные задачи. Таймеры тиков у ядер независимые, и их
// фазы расходятся. Если задача ядра 1 просыпается то по тику ядра 0, то по
// собственному, её пробуждение будет прыгать ровно на эту разность фаз.
static volatile uint64_t s_tick_us[2];

static void IRAM_ATTR tick_hook_core0(void) {
    s_tick_us[0] = (uint64_t) esp_timer_get_time();
}

static void IRAM_ATTR tick_hook_core1(void) {
    s_tick_us[1] = (uint64_t) esp_timer_get_time();
}

static struct {
    // сколько прошло от тика каждого ядра до пробуждения, суммы по видам
    uint64_t d0_sum[2], d1_sum[2], n[2];
    uint64_t phase_sum;   // фаза тика ядра 1 относительно ядра 0, мкс
    uint64_t phase_n;
    uint32_t phase_min, phase_max;
    const char *name[WAKE_SNAP_SLOTS];
    uint32_t late[WAKE_SNAP_SLOTS];
    uint32_t ontime[WAKE_SNAP_SLOTS];
    uint32_t overflow;
    uint64_t prev_us;
} WS;

static void wake_snapshot(void) {
    uint64_t now = fc_uptime_us();
    uint64_t prev = WS.prev_us;
    WS.prev_us = now;
    if (prev == 0) {
        return;
    }
    bool late = (now - prev) > 2400u;
    {
        uint64_t t0 = s_tick_us[0], t1 = s_tick_us[1];
        int k = late ? 1 : 0;
        if (t0 && t1 && now >= t0 && now >= t1) {
            WS.d0_sum[k] += now - t0;
            WS.d1_sum[k] += now - t1;
            ++WS.n[k];
            // Фаза: насколько тик ядра 1 позже тика ядра 0, по модулю тика.
            uint32_t ph = (uint32_t) (((int64_t) t1 - (int64_t) t0) % 1000 + 1000) % 1000;
            WS.phase_sum += ph;
            if (WS.phase_n == 0 || ph < WS.phase_min) {
                WS.phase_min = ph;
            }
            if (ph > WS.phase_max) {
                WS.phase_max = ph;
            }
            ++WS.phase_n;
        }
    }
    uint64_t age = 0;
    const char *st = fc_thread_stage(0, &age);
    for (int i = 0; i < WAKE_SNAP_SLOTS; ++i) {
        if (WS.name[i] == st || WS.name[i] == NULL) {
            WS.name[i] = st;
            if (late) {
                ++WS.late[i];
            } else {
                ++WS.ontime[i];
            }
            return;
        }
    }
    ++WS.overflow;
}

void fc_imu_rt_wake_snapshot_print(void) {
    printf("главный поток Refloat в момент пробуждения задачи датчика:\n");
    printf("  %-30s %10s %10s\n", "вызов", "поздно", "вовремя");
    for (int i = 0; i < WAKE_SNAP_SLOTS && WS.name[i]; ++i) {
        printf("  %-30s %10" PRIu32 " %10" PRIu32 "\n", WS.name[i], WS.late[i], WS.ontime[i]);
    }
    if (WS.overflow) {
        printf("  (не поместилось: %" PRIu32 ")\n", WS.overflow);
    }
    if (WS.phase_n) {
        printf("фаза тика ядра 1 относительно ядра 0: mean %.0f мкс, min %" PRIu32 ", max %" PRIu32 "\n",
               (double) WS.phase_sum / (double) WS.phase_n, WS.phase_min, WS.phase_max);
    }
    for (int k = 0; k < 2; ++k) {
        if (!WS.n[k]) {
            continue;
        }
        printf("  пробуждения %-8s n=%-6llu  от тика ядра 0: %6.0f мкс   от тика ядра 1: %6.0f мкс\n",
               k ? "ПОЗДНИЕ" : "вовремя", (unsigned long long) WS.n[k],
               (double) WS.d0_sum[k] / (double) WS.n[k], (double) WS.d1_sum[k] / (double) WS.n[k]);
    }
}

void fc_imu_rt_wake_snapshot_reset(void) {
    memset(&WS, 0, sizeof(WS));
}

static void imu_rt_task(void *arg) {
    (void) arg;
    esp_task_wdt_add(NULL);

    TickType_t next = xTaskGetTickCount();

    // Ритм — от аппаратного таймера с прерыванием на ЭТОМ ядре (fc_rt_clock.h).
    // Таймер запускается отсюда, а не из старта: прерывание размещается на
    // ядре вызывающей задачи, а эта задача закреплена за ядром 1. Если таймер
    // не поднялся, остаётся прежний ритм от тика — с известным дрожанием, но
    // без остановки контура.
    fc_rt_clock_subscribe(FC_RT_CLOCK_SLOT_IMU, xTaskGetCurrentTaskHandle());
#if FC_DIAG_NO_RT_CLOCK
    // Диагностический вариант для A/B (ТЗ v0.9I §15): ритм от тика.
    bool hw_clock = false;
#else
    bool hw_clock = fc_rt_clock_start(1000000u / FC_IMU_RT_CONTROL_HZ);
#endif

    while (R.run) {
        if (hw_clock) {
            // Таймаут в пять периодов: дольше ждать нельзя, иначе остановка
            // таймера выглядела бы как тихо замёрзший контур. Супервизор
            // увидит это по возрасту семпла, а счётчик — здесь.
            (void) fc_rt_clock_wait(10);
        } else {
            vTaskDelayUntil(&next, FC_IMU_RT_POLL_TICKS);
        }
        fc_timing_tick(FC_TIMING_IMU_WAKE);
#if FC_DIAG_LOOP_PROBES
        wake_snapshot();
#endif

        if (R.stall_ms) {
            // Задержка задачи чтения на известное время. Служит двум целям:
            // проверке watchdog (контур намеренно перестаёт отмечаться) и,
            // с v0.9B, сверке показаний приборов — трассировка зазоров и
            // возраст семпла у супервизора обязаны показать одно и то же
            // число (ТЗ v0.9B §6).
            int ms = R.stall_ms;
            R.stall_ms = 0;
            vTaskDelay(pdMS_TO_TICKS(ms));
            next = xTaskGetTickCount();
            continue;
        }

        int64_t t0 = esp_timer_get_time();
        icm20948_sample_t s;
        // Чужое время на ядре 1 внутри вызова (ТЗ v0.9I §2): счётчики FreeRTOS
        // обновляются при снятии задачи с процессора, поэтому каждый отрезок
        // чужой работы внутри вызова, закончившийся возвратом к этой задаче,
        // попадает в разность целиком.
#if FC_DIAG_LOOP_PROBES
        uint32_t o0 = others_runtime();
#endif
        uint64_t w0 = fc_uptime_us();
        fc_timing_exec_begin(FC_TIMING_IMU_I2C);
        esp_err_t err = icm20948_read(&s);
        fc_timing_exec_end(FC_TIMING_IMU_I2C);
        uint64_t w1 = fc_uptime_us();
#if FC_DIAG_LOOP_PROBES
        uint32_t o1 = others_runtime();
#else
        uint32_t o1 = 0, o0 = 0;
#endif
        ++R.i2c_calls;
        R.i2c_wall_sum_us += w1 - w0;
        {
            uint32_t b = (uint32_t) ((w1 - w0) / 50u);
            ++R.i2c_hist[b < FC_IMU_I2C_HIST_BINS ? b : FC_IMU_I2C_HIST_BINS - 1u];
        }
        R.i2c_others_sum_us += (uint64_t) (o1 - o0);
        int64_t t1 = esp_timer_get_time();
        uint32_t dur = (uint32_t) (t1 - t0);
        if (dur > R.max_read_us) {
            R.max_read_us = dur;
        }
        ++R.iterations;
        fc_timing_tick(FC_TIMING_IMU_READ);
        // Исполнение этой задачи раньше не мерилось вовсе: был только период.
        // А через неё идёт вся цепочка датчик -> Refloat, и её время — самая
        // крупная неучтённая статья бюджета ядра (ТЗ v0.9H §8).
        fc_timing_exec_begin(FC_TIMING_IMU_READ);

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

#if FC_LAB_DIAGNOSTICS
        // Впрыск провала модуля ускорения (ТЗ v0.9C §9). Подменяется ТОЛЬКО
        // ускорение: гироскоп остаётся настоящим, потому что проверяется
        // именно то, что такой семпл больше не считается потерей датчика.
        if (R.accel_low_n > 0) {
            --R.accel_low_n;
            for (int i = 0; i < 3; ++i) {
                s.accel_g[i] *= 0.05f;
            }
        }
#endif

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
                // Печать отсюда — из контура 500 Гц. Синхронный вывод в UART
                // стоил бы миллисекунд и съел бы несколько дедлайнов подряд
                // ровно в тот момент, когда шина уже сбоит. Кладём в кольцо.
                FC_LOGW(TAG, "серия отказов чтения — переинициализация датчика (#%llu)",
                        (unsigned long long) R.reinits);
                icm20948_config_t cfg = R.cfg;
                // Код возврата больше НЕ отбрасывается. Неудачная
                // переинициализация — неустранимый отказ, и политика обязана
                // узнать о нём как о таковом, а не вывести из протухания
                // (ТЗ v0.9D §3).
                icm20948_note_reinit(icm20948_init(&cfg));
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

        // Трассировка длинных зазоров (ТЗ v0.9A §3, v0.9B §4).
        //
        // Собственной копии «последнего времени» здесь больше НЕТ: интервал
        // берётся из канонической шкалы конвейера. Пока таких копий было
        // несколько, показания трассировки и супервизора нельзя было
        // сопоставлять, и это породило мнимое противоречие приборов
        // (docs/imu_time_model.md).
        //
        // Дешёвая проверка порога — в горячем пути; сбор снимка происходит
        // только когда порог превышен, то есть в среднем никогда.
        FcImuTimeline tl = fc_imu_pipeline_timeline();
        if (tl.prev_valid_us != 0) {
            fc_gap_port_capture(tl.last_valid_us, tl.prev_valid_us, tl.last_gap_us);
        }

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
            // Целые милли-градусы, а не %f: преобразование float в строку в
            // newlib на ESP32 программное и стоит сотни микросекунд. Здесь
            // это путь контура, и такой ценой информация не стоит — тем
            // более что в milli°/с она не теряется.
            FC_LOGI(TAG,
                    "IMU стабилен, остаток смещения %s: %+ld %+ld %+ld m°/с, контур запущен",
                    fc_imu_residual_state_name(ps.residual_state),
                    (long) (ps.residual_bias_dps[0] * 1000.0f),
                    (long) (ps.residual_bias_dps[1] * 1000.0f),
                    (long) (ps.residual_bias_dps[2] * 1000.0f));
        }

        // Измерение чтения закрывается ЗДЕСЬ, до колбэка. Канал control
        // меряется внутри этой же задачи, вокруг вызова Refloat: если бы
        // измерения вложились друг в друга, одно и то же время попало бы в
        // бюджет ядра дважды (ТЗ v0.9H §8, §9).
        fc_timing_exec_end(FC_TIMING_IMU_READ);

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

// Создание драйвера датчика на ЯДРЕ РЕАЛЬНОГО ВРЕМЕНИ (ТЗ v0.9I §9, §16).
//
// Зачем. Драйвер I²C размещает обработчик прерывания на том ядре, которое
// создало шину (esp_intr_alloc_intrstatus в i2c_master.c). Старт датчика
// вызывается из app_main, то есть с ядра 0, — и прерывание оказывалось там
// (подтверждено esp_intr_dump: I2C_EXT0 на CPU 0), а задача, ждущая
// завершения транзакции, живёт на ядре 1. Каждое завершение будило её через
// межъядерное уведомление.
//
// Измерено: развёртка по длине чтения с ядра 0 даёт пол 599 мкс на 14 байт
// (провод ~408 + накладные ~193), а живое чтение с ядра 1 — около 912 мкс.
// Разница ~313 мкс совпадает по порядку с ценой межъядерного пробуждения,
// измеренной на v0.7D (236 мкс в среднем).
//
// Как. Временная задача, закреплённая за ядром 1, выполняет ту же
// инициализацию и завершается; старт ждёт её. Порядок шагов не меняется.
// Путь повторной инициализации и так исполняется внутри задачи датчика, то
// есть на ядре 1, — после этого изменения оба пути дают одно и то же ядро.
typedef struct {
    const icm20948_config_t *cfg;
    esp_err_t err;
    SemaphoreHandle_t done;
} InitOnRtCore;

static void init_on_rt_core_task(void *arg) {
    InitOnRtCore *c = (InitOnRtCore *) arg;
    c->err = icm20948_init(c->cfg);
    xSemaphoreGive(c->done);
    vTaskDelete(NULL);
}

static esp_err_t icm20948_init_on_rt_core(const icm20948_config_t *cfg) {
    InitOnRtCore c = {.cfg = cfg, .err = ESP_FAIL, .done = xSemaphoreCreateBinary()};
    if (!c.done) {
        return ESP_ERR_NO_MEM;
    }
    // Стек с запасом: инициализация логирует и ждёт датчик.
    if (xTaskCreatePinnedToCore(init_on_rt_core_task, "fc_imu_init", 4096, &c,
                                tskIDLE_PRIORITY + 5, NULL, FC_CORE_REALTIME) != pdPASS) {
        vSemaphoreDelete(c.done);
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreTake(c.done, portMAX_DELAY);
    vSemaphoreDelete(c.done);
    return c.err;
}

bool fc_imu_rt_start(void) {
    void (*cb)(float *, float *, float *, float) = R.callback;
    memset(&R, 0, sizeof(R));
    R.callback = cb;

    R.cfg = icm20948_default_config();
#if FC_IMU_I2C_ISR_ON_CORE0
    // Диагностический вариант для A/B (ТЗ v0.9I §9): прежнее размещение,
    // прерывание I²C на ядре 0. Штатная сборка его не содержит.
    esp_err_t err = icm20948_init(&R.cfg);
#else
    esp_err_t err = icm20948_init_on_rt_core(&R.cfg);
#endif
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

    // Номинал нужен каналу не как дедлайн, а как масштаб гистограммы: без
    // него ширина корзины выходит 1 мкс, диапазон — 200 мкс, и все отсчёты
    // уходят в переполнение, отчего перцентиль показывал UINT32_MAX.
    // Транзакция выполняется ровно раз за итерацию, поэтому масштаб тот же.
    fc_timing_set_nominal(FC_TIMING_IMU_I2C, nominal_us);
    fc_timing_set_nominal(FC_TIMING_IMU_WAKE, nominal_us);

    // Зонды цикла (крючки тиков, чужое время внутри вызова I²C, перекрёстный
    // снимок пробуждения) — только в диагностической сборке (ТЗ v0.9I §15).
    // Измерено: вместе с аппаратным ритмом они съедают около 17 % ядра 1
    // (простой 8.65 % против 26.15 % без них). Своё дело они сделали — с их
    // помощью найдены обе причины этапа, — но в штатной сборке им не место.
    // Сборка с зондами: idf.py -B build_probes -DFC_DIAG_LOOP_PROBES=1 build
#if FC_DIAG_LOOP_PROBES
    esp_register_freertos_tick_hook_for_cpu(tick_hook_core0, 0);
    esp_register_freertos_tick_hook_for_cpu(tick_hook_core1, 1);
#endif

    xTaskCreatePinnedToCore(
        imu_rt_task, "fc_imu_rt", FC_IMU_RT_STACK_BYTES, NULL, FC_PRIO_IMU, &R.task,
        FC_CORE_REALTIME
    );
    return R.driver_ok;
}

void fc_imu_rt_stop(void) {
    R.run = false;
}

#if FC_LAB_DIAGNOSTICS
void fc_imu_rt_inject_accel_low(int count) {
    R.accel_low_n = count;
}
#endif
