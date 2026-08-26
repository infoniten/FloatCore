// Логический мотор на ESP32, этап v0.5 — ЗАБЛОКИРОВАН (ТЗ §6, §19).
//
// Это реализация того же контракта compat/motor/logical_motor.h, что и
// host-mock. Отличие принципиальное и намеренное: здесь физически нет пути
// к выходу.
//
// Почему нельзя сформировать команду мотору:
//
//   1. Ни один вызов не доходит до передатчика: в этой единице трансляции нет
//      ни одного обращения к драйверу TWAI/CAN. Заголовок driver/twai.h не
//      включён, символы twai_* в прошивке не линкуются (проверяется тестом
//      tools/esp32_smoke.sh по таблице символов .elf).
//   2. Трансивер CAN к плате не подключён (ТЗ §19), а GPIO для него не
//      конфигурируются: ни gpio_set_direction, ни matrix routing.
//   3. Каждый запрос считается и отбрасывается; наружу уходит только счётчик.
//
// Снять блокировку одним параметром нельзя: нужен новый код передачи, ревизия
// и отдельный этап — см. docs/esp32_safety.md.

#include "fc_platform.h"

#include "../../../compat/motor/logical_motor.h"

#include "esp_log.h"
#include "esp_timer.h"

#include <math.h>
#include <string.h>

static const char *TAG = "motor";

// Ограничение частоты логов: контур 500 Гц уничтожил бы serial, печатая
// каждую блокировку (ТЗ §6). Печатаем не чаще одного раза в секунду на вид
// команды, добавляя число подавленных сообщений.
#define FC_MOTOR_LOG_PERIOD_US 1000000

static struct {
    FcMotorStats stats;
    uint64_t last_log_us[FC_MOTOR_CMD_COUNT];
    uint64_t since_log[FC_MOTOR_CMD_COUNT];
    LogicalMotorConfig config;
} g_motor;

static const char *const kKindName[FC_MOTOR_CMD_COUNT] = {
    [FC_MOTOR_CMD_CURRENT] = "set_current",
    [FC_MOTOR_CMD_BRAKE] = "set_brake_current",
    [FC_MOTOR_CMD_DUTY] = "set_duty",
    [FC_MOTOR_CMD_RPM] = "set_rpm",
};

static void block(FcMotorCmdKind kind, float value) {
    ++g_motor.stats.blocked[kind];
    ++g_motor.stats.total_blocked;
    g_motor.stats.last_value[kind] = value;

    uint64_t now = (uint64_t) esp_timer_get_time();
    ++g_motor.since_log[kind];
    if (now - g_motor.last_log_us[kind] >= FC_MOTOR_LOG_PERIOD_US) {
        uint64_t suppressed = g_motor.since_log[kind] - 1;
        g_motor.last_log_us[kind] = now;
        g_motor.since_log[kind] = 0;
        ESP_LOGW(
            TAG, "blocked command: %s(%.3f) — всего %llu, подавлено с прошлой записи %llu",
            kKindName[kind], (double) value,
            (unsigned long long) g_motor.stats.blocked[kind],
            (unsigned long long) suppressed
        );
    }
}

// ------------------------------------------------- реализация logical_motor.h

void logical_motor_init(const LogicalMotorConfig *config) {
    memset(&g_motor, 0, sizeof(g_motor));
    if (config) {
        g_motor.config = *config;
    }
    ESP_LOGW(TAG, "backend = blocked: выход на мотор недоступен (ТЗ v0.5 §6)");
}

void logical_motor_request_current(float amps) {
    block(FC_MOTOR_CMD_CURRENT, amps);
}

void logical_motor_request_brake_current(float amps) {
    block(FC_MOTOR_CMD_BRAKE, amps);
}

void logical_motor_request_duty(float duty) {
    block(FC_MOTOR_CMD_DUTY, duty);
}

void logical_motor_set_current_off_delay(float seconds) {
    (void) seconds;  // параметр следующей команды тока, которой не будет
}

void logical_motor_release(void) {
    // Уже освобождён: тяги нет и не было.
}

void logical_motor_keepalive(void) {
    ++g_motor.stats.keepalive_calls;
}

LogicalMotorTelemetry logical_motor_telemetry(void) {
    // Телеметрии нет: ни один ESC не подключён. Возвращается пустой снимок с
    // явными признаками offline — Refloat увидит нули и не тронет мотор.
    LogicalMotorTelemetry t;
    memset(&t, 0, sizeof(t));
    t.faults = LM_FAULT_ESC_A_TIMEOUT | LM_FAULT_ESC_B_TIMEOUT;
    t.esc_a_alive = false;
    t.esc_b_alive = false;
    t.timestamp_us = (uint64_t) esp_timer_get_time();
    return t;
}

bool logical_motor_healthy(void) {
    return false;
}

// ------------------------------------------------------------- наружу для CLI

FcMotorStats fc_motor_stats(void) {
    return g_motor.stats;
}

const char *fc_motor_backend_name(void) {
    return "blocked";
}

const char *fc_can_backend_name(void) {
    return "unavailable (TWAI не инициализирован, трансивер не подключён)";
}
