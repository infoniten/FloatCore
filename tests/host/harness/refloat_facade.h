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
    float pitch, balance_pitch, roll, pitch_rate;
    float setpoint, setpoint_target;
    float balance_current;
    float motor_erpm, motor_duty, motor_current;
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
