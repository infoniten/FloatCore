// Mock-IMU для ESP32, этап v0.5 (ТЗ §8).
//
// Физический ICM-20948 не подключён. Модуль задаёт ритм контура управления и
// отдаёт Refloat строго покоящуюся ориентацию.
//
// Контракт выведен из самого upstream, а не придуман:
//
//   * imu_get_roll/pitch/yaw — РАДИАНЫ.
//     refloat-upstream/src/imu.c:36-40 оборачивает их в rad2deg().
//
//   * imu_get_gyro(g) и gyro в callback — РАД/С, порядок осей x,y,z.
//     balance_filter.c:114-125 интегрирует кватернион как q += 0.5*q*g*dt,
//     что корректно только для рад/с. Тот же массив уходит в imu.c:51 как
//     pitch_rate, где домножается на настраиваемый коэффициент kp2 (pid.c:69),
//     то есть масштаб там поглощается тюнингом.
//
//   * acc в callback — G (единицы ускорения свободного падения).
//     balance_filter.c:42-48 сравнивает |acc| с 1.0f.
//
//   * Ориентация покоя: кватернион (1,0,0,0) и acc = (0,0,+1).
//     При единичном кватернионе оценка направления гравитации в
//     balance_filter.c:100 равна halfv = (0, 0, +0.5), то есть +Z. Только
//     acc = (0,0,+1) даёт нулевую ошибку фильтра и устойчивые pitch=roll=0.
//     Любой другой вектор заставил бы Mahony «доворачивать» доску.
//
//   * dt — секунды, реальный интервал между семплами, а не номинал:
//     latency_tracker и frequency_tracker Refloat считают частоту по нему.

#include "fc_platform.h"

#include "../../../compat/safety/fc_supervisor.h"

#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

// Частота семплирования mock-IMU. 500 Гц — MVP-значение из
// docs/esp32_architecture.md §2 (на VESC аппаратно 832 Гц). Именно это число
// обязано вернуться в get_cfg_int(CFG_PARAM_IMU_sample_rate): при нуле Refloat
// решит, что это прошивка 6.02, и подставит выдуманные 620 Гц.
#define FC_IMU_RATE_HZ 500
#define FC_IMU_PERIOD_TICKS (configTICK_RATE_HZ / FC_IMU_RATE_HZ)

// Пауза перед imu_startup_done: на VESC это время калибровки гироскопа.
// Пока false, Refloat держит STATE_STARTUP — безопасное состояние.
#define FC_IMU_STARTUP_MS 1500

static struct {
    void (*callback)(float *acc, float *gyro, float *mag, float dt);
    TaskHandle_t task;
    volatile bool run;
    volatile bool startup_done;

    float accel[3];
    float gyro[3];
    float quat[4];
    float roll, pitch, yaw;
    volatile int stall_ms;  // диагностика: намеренная остановка контура
} g_imu;

int fc_imu_rate_hz(void) {
    return FC_IMU_RATE_HZ;
}

bool fc_imu_startup_done(void) {
    return g_imu.startup_done;
}

void fc_imu_set_callback(void (*cb)(float *acc, float *gyro, float *mag, float dt)) {
    g_imu.callback = cb;
}

void fc_imu_get_state(
    float *roll, float *pitch, float *yaw, float accel[3], float gyro[3], float quat[4]
) {
    if (roll) {
        *roll = g_imu.roll;
    }
    if (pitch) {
        *pitch = g_imu.pitch;
    }
    if (yaw) {
        *yaw = g_imu.yaw;
    }
    if (accel) {
        memcpy(accel, g_imu.accel, sizeof(g_imu.accel));
    }
    if (gyro) {
        memcpy(gyro, g_imu.gyro, sizeof(g_imu.gyro));
    }
    if (quat) {
        memcpy(quat, g_imu.quat, sizeof(g_imu.quat));
    }
}

static void imu_task(void *arg) {
    (void) arg;

    // Контур управления подписан на TWDT: его зависание обязано быть заметным
    // (ТЗ §11). Watchdog гасится ровно один раз за семпл, ниже по циклу.
    esp_task_wdt_add(NULL);

    // Стартовая калибровка: имитируем задержку готовности AHRS прошивки.
    vTaskDelay(pdMS_TO_TICKS(FC_IMU_STARTUP_MS));
    g_imu.startup_done = true;

    TickType_t next = xTaskGetTickCount();
    int64_t prev_us = esp_timer_get_time();

    while (g_imu.run) {
        vTaskDelayUntil(&next, FC_IMU_PERIOD_TICKS);

        // Отметка ставится в момент пробуждения, а не после обработки семпла:
        // измеряется период контура, а не период плюс время его работы.
        fc_timing_tick(FC_TIMING_CONTROL);

        int64_t now_us = esp_timer_get_time();
        float dt = (float) (now_us - prev_us) * 1e-6f;
        prev_us = now_us;

        if (g_imu.stall_ms) {
            // Проверка watchdog (ТЗ §11): контур намеренно перестаёт
            // отмечаться в TWDT. Задача засыпает, а не крутит CPU, поэтому
            // остальная система продолжает работать и печатает диагностику.
            int ms = g_imu.stall_ms;
            g_imu.stall_ms = 0;
            vTaskDelay(pdMS_TO_TICKS(ms));
            next = xTaskGetTickCount();
            prev_us = esp_timer_get_time();
            continue;
        }

        // Отметка живости контура для супервизора: он обязан узнать о
        // зависании раньше, чем сработает watchdog.
        fc_supervisor_report_loop_tick((uint64_t) now_us);

        void (*cb)(float *, float *, float *, float) = g_imu.callback;
        if (cb) {
            fc_timing_exec_begin(FC_TIMING_CONTROL);
            // Массивы копируются: Refloat получает буферы, которые не меняются
            // под ним во время обработки семпла.
            float acc[3], gyro[3];
            memcpy(acc, g_imu.accel, sizeof(acc));
            memcpy(gyro, g_imu.gyro, sizeof(gyro));
            cb(acc, gyro, NULL, dt);
            fc_timing_exec_end(FC_TIMING_CONTROL);
        }
        esp_task_wdt_reset();
    }
    esp_task_wdt_delete(NULL);
    g_imu.task = NULL;
    vTaskDelete(NULL);
}

void fc_imu_mock_start(void) {
    memset(&g_imu, 0, sizeof(g_imu));
    // Покой: гравитация по +Z, гироскоп в нуле, кватернион единичный.
    g_imu.accel[0] = 0.0f;
    g_imu.accel[1] = 0.0f;
    g_imu.accel[2] = 1.0f;
    g_imu.quat[0] = 1.0f;
    g_imu.run = true;

    fc_timing_set_nominal(FC_TIMING_CONTROL, 1000000 / FC_IMU_RATE_HZ);

    xTaskCreatePinnedToCore(
        imu_task, "fc_imu", 4096, NULL, FC_PRIO_IMU, &g_imu.task, FC_CORE_REALTIME
    );
}

void fc_imu_mock_stop(void) {
    g_imu.run = false;
}

void fc_imu_inject_stall(int ms) {
    g_imu.stall_ms = ms;
}

uint32_t fc_imu_stack_watermark(void) {
    return g_imu.task ? (uint32_t) uxTaskGetStackHighWaterMark(g_imu.task) : 0;
}
