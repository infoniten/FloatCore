// Задача Safety Supervisor на ESP32 (ТЗ v0.6A §8, §10, §27).
//
// Сам супервизор платформенно-нейтрален и живёт в compat/safety. Здесь —
// только его питание: сбор входов с платформы и периодический опрос
// таймаутов.
//
// Задача сознательно НЕ realtime: она не должна конкурировать с контуром.
// Её частота (100 Гц) выбрана так, чтобы реакция на протухание была заметно
// быстрее таймаутов супервизора (40–50 мс), но нагрузка оставалась
// пренебрежимой.

#include "fc_platform.h"

#include "../../../compat/safety/fc_imu_health.h"
#include "../../../compat/safety/fc_supervisor.h"

#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define FC_SUPERVISOR_HZ 100

static TaskHandle_t g_task;

uint32_t fc_supervisor_stack_watermark(void) {
    return g_task ? (uint32_t) uxTaskGetStackHighWaterMark(g_task) : 0;
}

static void supervisor_task(void *arg) {
    (void) arg;
    TickType_t next = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&next, configTICK_RATE_HZ / FC_SUPERVISOR_HZ);
        uint64_t now = (uint64_t) esp_timer_get_time();

        // Диагностика IMU: проверка возраста семпла делается здесь, а не в
        // задаче чтения — иначе замолчавшая задача перестала бы и проверять
        // саму себя.
        FcImuHealthState hs = fc_imu_health_poll(now);
        bool imu_ok = (hs == FC_IMU_OK);
        // NOT_INITIALIZED не объявляется отказом, но и здоровьем не
        // объявляется тоже: вход imu_healthy просто остаётся в исходном
        // false, а READY без него недостижим (fc_supervisor.c,
        // ready_conditions_met). С v0.6D датчик задаёт ритм контура, поэтому
        // «датчик не поднялся» и так означает, что контур не идёт, и
        // супервизор увидит это по таймауту тика. Отказом же считается
        // поломка живого датчика после того, как он однажды заработал.
        if (hs != FC_IMU_NOT_INITIALIZED) {
            fc_supervisor_report_imu_healthy(imu_ok, now);
        }

        // Watchdog: срабатывание TWDT видно по причине предыдущего сброса и
        // по счётчику, который ведёт обработчик. Здесь достаточно факта, что
        // текущая загрузка не была вызвана watchdog-ом.
        esp_reset_reason_t rr = esp_reset_reason();
        fc_supervisor_report_watchdog(rr != ESP_RST_TASK_WDT && rr != ESP_RST_INT_WDT &&
                                          rr != ESP_RST_WDT,
                                      now);

        fc_supervisor_poll(now);
    }
}

void fc_supervisor_task_start(void) {
    // Приоритет выше aux, но НИЖЕ контура и главного потока Refloat.
    //
    // Изначально здесь стояло FC_PRIO_REFLOAT + 2 = 14, то есть ровно
    // приоритет контура. Это дефект: при равных приоритетах и совпадающем
    // моменте пробуждения FreeRTOS ставит задачи в общую очередь, и контур
    // начинает ждать супервизора. Обнаружено при профилировании
    // (docs/realtime_timing.md §4, гипотеза 2).
    //
    // Супервизор всё равно успевает: его период 10 мс против 2 мс у контура,
    // а таймауты, которые он сторожит, — 40 и 50 мс.
    xTaskCreatePinnedToCore(supervisor_task, "fc_super", 4096, NULL, FC_PRIO_SUPERVISOR, &g_task,
                            FC_CORE_REALTIME);
}
