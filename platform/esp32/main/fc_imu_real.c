// Задача чтения физического ICM-20948 (ТЗ v0.6A §6, §7).
//
// ВАЖНО, ограничение этапа v0.6A (§29): данные этого модуля НЕ передаются
// Refloat. Refloat продолжает работать от mock-IMU. Здесь только чтение,
// диагностика и статистика. Мост в VESC_IF появится отдельным этапом, после
// финального монтажа и проверки ориентации.
//
// Архитектура выбрана по измерению, а не по умолчанию: датчик настроен на
// 562.5 Гц, задача читает его на 500 Гц и держит последний семпл, откуда
// потребитель заберёт «самое свежее». Так период чтения не связан жёстко с
// периодом контура, и пропуск одного чтения не превращается в дыру данных.

#include "fc_platform.h"

#include "../drivers/icm20948.h"
#include "../../../compat/safety/fc_imu_health.h"
#include "../../../compat/safety/fc_supervisor.h"

#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

static const char *TAG = "imu";

// 500 Гц выбрано по измерению, а не по умолчанию (ТЗ v0.6A §6).
//
// Датчик выдаёт 562.5 Гц; читать реже — значит терять семплы. Проверено, что
// чтение на 250 Гц полностью убирает вклад этой задачи в джиттер контура
// (1.07 % опозданий против 3.97 % при 500 Гц и 1.04 % без задачи вовсе), но
// цена — пропуск каждого второго семпла, что для будущего контура
// неприемлемо.
//
// Причина вклада: две независимые задачи с одинаковым периодом 500 Гц на
// одном ядре систематически просыпаются на одном тике и конкурируют. Это
// временная конфигурация именно v0.6A, где mock и реальный датчик работают
// параллельно. В v0.6B они сольются в одну задачу — та же, что читает
// датчик, будет вызывать imu_ref_callback, — и конкуренция исчезнет по
// построению (docs/esp32_architecture.md §2).
#define FC_IMU_READ_HZ 500
#define FC_IMU_READ_PERIOD_TICKS (configTICK_RATE_HZ / FC_IMU_READ_HZ)

static struct {
    TaskHandle_t task;
    volatile bool run;
    bool driver_ok;
    icm20948_sample_t last;
    bool have_last;
    uint64_t task_iterations;
    uint32_t max_read_us;
} R;

bool fc_imu_real_available(void) {
    return R.driver_ok;
}

bool fc_imu_real_last_sample(icm20948_sample_t *out) {
    if (!R.have_last || !out) {
        return false;
    }
    *out = R.last;
    return true;
}

// Флаг «задача действительно жива», а не «её просили жить»: fc_imu_real_stop()
// лишь снимает run, а завершается задача на следующей итерации. Стресс-тест
// (fc_imu_stress.c) обязан дождаться именно фактического ухода, иначе на шине
// окажутся два читателя без мьютекса.
bool fc_imu_real_running(void) {
    return R.task != NULL;
}

uint64_t fc_imu_real_iterations(void) {
    return R.task_iterations;
}

uint32_t fc_imu_real_max_read_us(void) {
    return R.max_read_us;
}

static void imu_real_task(void *arg) {
    (void) arg;
    esp_task_wdt_add(NULL);

    TickType_t next = xTaskGetTickCount();
    while (R.run) {
        vTaskDelayUntil(&next, FC_IMU_READ_PERIOD_TICKS);

        int64_t t0 = esp_timer_get_time();
        icm20948_sample_t s;
        esp_err_t err = icm20948_read(&s);
        uint32_t dur = (uint32_t) (esp_timer_get_time() - t0);
        if (dur > R.max_read_us) {
            R.max_read_us = dur;
        }

        // Диагностика работает и на успехе, и на отказе: health-слой должен
        // видеть саму неудачу транзакции, а не только плохие числа.
        FcImuRawSample raw;
        memset(&raw, 0, sizeof(raw));
        if (err == ESP_OK) {
            memcpy(raw.accel_g, s.accel_g, sizeof(raw.accel_g));
            memcpy(raw.gyro_dps, s.gyro_dps, sizeof(raw.gyro_dps));
            raw.temperature_c = s.temperature_c;
            raw.timestamp_us = s.timestamp_us;
            raw.sample_counter = s.sample_counter;
            raw.valid = true;
            R.last = s;
            R.have_last = true;
        }

        uint64_t now = (uint64_t) esp_timer_get_time();
        FcImuHealthState hs = fc_imu_health_update(err == ESP_OK, &raw, now);
        if (hs == FC_IMU_OK) {
            fc_supervisor_report_imu_sample(now);
        }
        ++R.task_iterations;
        fc_timing_tick(FC_TIMING_IMU_READ);
        esp_task_wdt_reset();
    }

    esp_task_wdt_delete(NULL);
    R.task = NULL;
    vTaskDelete(NULL);
}

bool fc_imu_real_start(void) {
    memset(&R, 0, sizeof(R));

    icm20948_config_t cfg = icm20948_default_config();
    esp_err_t err = icm20948_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ICM-20948 не инициализирован: %s", esp_err_to_name(err));
        // Health-слой обязан отражать факт: датчика нет.
        FcImuHealthConfig hc = fc_imu_health_config_for(icm20948_accel_fs_g(cfg.accel_fs),
                                                        icm20948_gyro_fs_dps(cfg.gyro_fs));
        fc_imu_health_init(&hc);
        return false;
    }

    uint8_t who = 0;
    icm20948_who_am_i(&who);
    // Адрес берётся из активной конфигурации, а не из запрошенной: драйвер мог
    // разрешить его опросом (AD0 подтянут к питанию — тогда 0x69, а не 0x68).
    ESP_LOGI(TAG, "ICM-20948 на 0x%02x, WHO_AM_I = 0x%02x, шкалы ±%.0f g / ±%.0f °/с, ODR %.1f Гц",
             icm20948_active_config()->i2c_addr, who,
             (double) icm20948_accel_fs_g(cfg.accel_fs),
             (double) icm20948_gyro_fs_dps(cfg.gyro_fs), (double) icm20948_odr_hz(cfg.smplrt_div));

    FcImuHealthConfig hc = fc_imu_health_config_for(icm20948_accel_fs_g(cfg.accel_fs),
                                                    icm20948_gyro_fs_dps(cfg.gyro_fs));
    fc_imu_health_init(&hc);

    R.driver_ok = true;
    R.run = true;
    fc_timing_set_nominal(FC_TIMING_IMU_READ, 1000000 / FC_IMU_READ_HZ);
    // Приоритет ниже контура, но выше главного потока Refloat: свежесть
    // данных важнее телеметрии, но сам контур вытеснять нельзя.
    xTaskCreatePinnedToCore(imu_real_task, "fc_imu_hw", 4096, NULL, FC_PRIO_IMU_HW, &R.task,
                            FC_CORE_REALTIME);
    return true;
}

void fc_imu_real_stop(void) {
    R.run = false;
}

uint32_t fc_imu_real_stack_watermark(void) {
    return R.task ? (uint32_t) uxTaskGetStackHighWaterMark(R.task) : 0;
}
