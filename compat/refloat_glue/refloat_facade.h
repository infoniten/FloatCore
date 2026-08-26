// Тонкая обёртка над Refloat для тестов.
//
// Изолирует заголовки Refloat от системных: файлы Refloat объявляют
// `typedef uint32_t time_t`, что конфликтует с newlib/libc. Поэтому весь код,
// включающий заголовки Refloat, живёт в refloat_facade.c, а тесты работают
// только с этим заголовком (stdint/stdbool/stddef).
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int state;            // RunState: 0 DISABLED, 1 STARTUP, 2 READY, 3 RUNNING
    int mode;             // Mode
    int sat;              // SetpointAdjustmentType
    int stop_condition;   // StopCondition
    int footpad_state;    // FootpadSensorState: 0 NONE, 1 LEFT, 2 RIGHT, 3 BOTH
    float adc_left, adc_right;
    // Пороги срабатывания footpad из конфигурации: нужны, чтобы доказать, что
    // отданное платформой напряжение действительно означает disengaged
    // (footpad_sensor.c:31 — порог 0 означает «датчик всегда нажат»).
    float fault_adc1, fault_adc2;
    float pitch, balance_pitch, roll, pitch_rate;
    float setpoint, setpoint_target;
    float balance_current;
    float motor_erpm, motor_duty, motor_current;
    // Пределы, которые Refloat прочитал через VESC_IF->get_cfg_float().
    // Нужны, чтобы проверить: Virtual mcConfig показывает UI ровно их.
    float motor_current_max, motor_current_min;
    float motor_batt_current_max, motor_batt_current_min;
    float mosfet_temp_max, motor_temp_max;
    float lv_threshold, hv_threshold;
    bool darkride;
    bool traction_control;
    float imu_frequency, main_frequency;
} RefloatSnapshot;

/** Инициализация Refloat (вызов его init()). Возвращает false при ошибке. */
bool refloat_facade_start(void);

/** Останов Refloat (его stop_fun). */
void refloat_facade_stop(void);

RefloatSnapshot refloat_facade_snapshot(void);

const char *refloat_facade_state_name(int state);
const char *refloat_facade_stop_name(int stop_condition);
const char *refloat_facade_footpad_name(int fs);

// --------------------------------------------------- проверка persistence
//
// Гоняет конфигурацию Refloat через его же путь сохранения — тот самый, что
// использует VESC Tool: зарегистрированный через conf_custom_add_config
// set_cfg() десериализует данные и вызывает write_cfg_to_eeprom().
//
// Меняется только `leds.status.brightness_headlights_off` — яркость фар в
// статус-баре. Параметр выбран потому, что он не участвует ни в одном
// вычислении контура, а лента к плате не подключена вовсе.
//
// Платформа обязана предоставить floatcore_config_apply(): вызов сохранённого
// указателя set_cfg.
bool refloat_facade_config_save_test(float value);
float refloat_facade_config_test_value(void);
