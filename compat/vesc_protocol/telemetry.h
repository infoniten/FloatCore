// Realtime-телеметрия для VESC Tool (COMM_GET_VALUES / COMM_GET_VALUES_SELECTIVE).
//
// Источник данных — LogicalMotorTelemetry, то есть агрегированные показания
// двух физических VESC. Правила агрегации — docs/motor_semantics.md.
#pragma once

#include "../motor/logical_motor.h"

#include <stddef.h>
#include <stdint.h>

/** Биты маски COMM_GET_VALUES_SELECTIVE (порядок задан vesc_tool/commands.cpp). */
typedef enum {
    VALUES_TEMP_MOS = 1u << 0,
    VALUES_TEMP_MOTOR = 1u << 1,
    VALUES_CURRENT_MOTOR = 1u << 2,
    VALUES_CURRENT_IN = 1u << 3,
    VALUES_ID = 1u << 4,
    VALUES_IQ = 1u << 5,
    VALUES_DUTY = 1u << 6,
    VALUES_RPM = 1u << 7,
    VALUES_V_IN = 1u << 8,
    VALUES_AMP_HOURS = 1u << 9,
    VALUES_AMP_HOURS_CHARGED = 1u << 10,
    VALUES_WATT_HOURS = 1u << 11,
    VALUES_WATT_HOURS_CHARGED = 1u << 12,
    VALUES_TACHOMETER = 1u << 13,
    VALUES_TACHOMETER_ABS = 1u << 14,
    VALUES_FAULT = 1u << 15,
    VALUES_POSITION = 1u << 16,
    VALUES_VESC_ID = 1u << 17,
    VALUES_TEMP_MOS_123 = 1u << 18,
    VALUES_VD = 1u << 19,
    VALUES_VQ = 1u << 20,
    VALUES_STATUS = 1u << 21,
} VescValuesMask;

typedef struct {
    float temp_mos;
    float temp_motor;
    float current_motor;
    float current_in;
    float id;
    float iq;
    float duty;
    float rpm;
    float v_in;
    float amp_hours;
    float amp_hours_charged;
    float watt_hours;
    float watt_hours_charged;
    int32_t tachometer;
    int32_t tachometer_abs;
    uint8_t fault_code;
    float position;
    uint8_t vesc_id;
    float vd;
    float vq;
    bool has_timeout;
    bool kill_sw_active;
} VescValues;

/** Заполнить VescValues из агрегированной телеметрии логического мотора. */
void telemetry_from_logical_motor(
    const LogicalMotorTelemetry *lm, float amp_hours, float watt_hours, int32_t tachometer,
    VescValues *out
);

/**
 * Закодировать ответ (включая байт команды).
 * `mask` == 0xFFFFFFFF и `selective` == false → COMM_GET_VALUES.
 * Возвращает длину или 0 при нехватке места.
 */
size_t telemetry_encode(
    const VescValues *v, uint32_t mask, bool selective, uint8_t *out, size_t cap
);
