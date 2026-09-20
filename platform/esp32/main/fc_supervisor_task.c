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

#include "fc_gap_port.h"
#include "../../../compat/imu/fc_imu_pipeline.h"
#include "../../../compat/safety/fc_imu_policy.h"
#include "fc_platform.h"

#include "../../../compat/safety/fc_imu_health.h"
#include "../../../compat/imu/fc_imu_calibration.h"
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

        // Дорогие поля снимков зазоров: обход кучи и стеков стоит слишком
        // много для задачи реального времени, поэтому дописываются здесь.
        // Они помечены отдельным признаком — «снято позже», чтобы никто не
        // принял их за состояние в момент события.
        fc_gap_port_fill_deferred();

        // Диагностика IMU: проверка возраста семпла делается здесь, а не в
        // задаче чтения — иначе замолчавшая задача перестала бы и проверять
        // саму себя.
        FcImuHealthState hs = fc_imu_health_poll(now);
        // NOT_INITIALIZED не объявляется отказом, но и здоровьем не
        // объявляется тоже: вход imu_healthy просто остаётся в исходном
        // false, а READY без него недостижим (fc_supervisor.c,
        // ready_conditions_met). С v0.6D датчик задаёт ритм контура, поэтому
        // «датчик не поднялся» и так означает, что контур не идёт, и
        // супервизор увидит это по таймауту тика. Отказом же считается
        // поломка живого датчика после того, как он однажды заработал.
        if (hs != FC_IMU_NOT_INITIALIZED) {
            // Шкала берётся у её владельца — конвейера IMU, — а не хранится
            // здесь отдельной копией. Смысл в том, чтобы снимок отказа
            // содержал ровно те числа, которые видит трассировка зазоров.
            FcImuTimeline tl = fc_imu_pipeline_timeline();
            FcSupervisorImuTime st = {
                .last_valid_us = tl.last_valid_us,
                .prev_valid_us = tl.prev_valid_us,
                .last_gap_us = tl.last_gap_us,
                .sequence = tl.sequence,
                .last_verdict = tl.last_verdict,
                .last_poll_us = tl.last_poll_us,
            };
            // Решение принимает политика, а не состояние модуля здоровья.
            // Прежде одиночный сбой чтения или один отвергнутый семпл
            // защёлкивали отказ навсегда; теперь право на тягу зависит от
            // того, есть ли прямо сейчас свежая и достоверная ориентация
            // (docs/imu_health_policy.md).
            FcImuHealthStatus hst = fc_imu_health_status();
            FcImuPolicyConfig pc = fc_imu_policy_default_config();
            FcImuPolicyInputs pin = {
                .now_us = now,
                .last_valid_us = tl.last_valid_us,
                .have_valid = tl.sequence > 0,
                .consecutive_read_failures = hst.consecutive_read_errors,
                .consecutive_invalid = hst.consecutive_invalid,
                .consecutive_accel_low = hst.consecutive_accel_low,
                // Залипание шины и неудачу переинициализации платформа пока
                // не сообщает. Подставлять сюда false — значит утверждать,
                // что их не бывает; поэтому они остаются явными нулями с
                // этим комментарием, а не молчаливым умолчанием.
                .bus_stuck = false,
                .reinit_failed = false,
            };
            uint32_t reasons = 0;
            FcImuPermit permit = fc_imu_policy_evaluate(&pc, &pin, &reasons);
            fc_supervisor_report_imu_permit(permit, reasons, (uint32_t) hs, &st, now);
        }

        // Watchdog: срабатывание TWDT видно по причине предыдущего сброса и
        // по счётчику, который ведёт обработчик. Здесь достаточно факта, что
        // текущая загрузка не была вызвана watchdog-ом.
        esp_reset_reason_t rr = esp_reset_reason();
        fc_supervisor_report_watchdog(rr != ESP_RST_TASK_WDT && rr != ESP_RST_INT_WDT &&
                                          rr != ESP_RST_WDT,
                                      now);

        // Валидная калибровка ориентации — условие READY (ТЗ v0.6E §7, §24).
        // Без неё платформа не знает, как датчик стоит относительно доски, и
        // все углы, которые она отдаёт Refloat, — углы датчика, а не доски.
        fc_supervisor_report_calibration_valid(fc_imu_rt_cal_status() == FC_IMU_CAL_VALID, now);

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
