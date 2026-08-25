// Mock-реализация LogicalMotor (ТЗ v0.2 §4).
//
// НИКАКОГО вывода на CAN. Все запросы попадают в кольцевой буфер и в счётчики,
// телеметрия задаётся тестом. Здесь же живут проверки безопасности, которые в
// боевой реализации будут стоять перед отправкой CAN-кадров.

#include "../../../compat/motor/logical_motor.h"
#include "logical_motor_mock.h"

#include <math.h>
#include <string.h>

static LogicalMotorConfig cfg;
static LogicalMotorTelemetry tele;
static LogicalMotorMockStats stats;
static float last_request[LM_ESC_COUNT];
static uint64_t (*clock_us)(void) = NULL;

void logical_motor_mock_set_clock(uint64_t (*fn)(void)) {
    clock_us = fn;
}

static uint64_t now(void) {
    return clock_us ? clock_us() : 0;
}

void logical_motor_init(const LogicalMotorConfig *config) {
    memset(&stats, 0, sizeof(stats));
    memset(&tele, 0, sizeof(tele));
    memset(last_request, 0, sizeof(last_request));
    if (config) {
        cfg = *config;
    } else {
        cfg = (LogicalMotorConfig){
            .esc = {{.can_id = 1, .invert = false, .scale = 1.0f, .current_limit = 60.0f},
                    {.can_id = 2, .invert = true, .scale = 1.0f, .current_limit = 60.0f}},
            .telemetry_timeout_s = 0.1f,
            .erpm_mismatch_limit = 500.0f,
            .voltage_mismatch_limit = 1.0f,
        };
    }
    tele.esc_a_alive = true;
    tele.esc_b_alive = true;
}

/** Общая для всех запросов проверка валидности. */
static bool validate(float value, float limit) {
    if (!isfinite(value)) {
        ++stats.invalid_requests;
        tele.faults |= LM_FAULT_INVALID_REQUEST;
        return false;
    }
    if (fabsf(value) > limit) {
        ++stats.clamped_requests;
        tele.faults |= LM_FAULT_INVALID_REQUEST;
        return false;
    }
    return true;
}

void logical_motor_request_current(float amps) {
    ++stats.current_requests;
    stats.last_current = amps;
    stats.last_request_us = now();
    if (!validate(amps, cfg.esc[LM_ESC_A].current_limit)) {
        return;
    }
    for (size_t i = 0; i < LM_ESC_COUNT; ++i) {
        float v = amps * cfg.esc[i].scale;
        last_request[i] = cfg.esc[i].invert ? -v : v;
    }
    // Боевая реализация здесь отправила бы CAN_PACKET_SET_CURRENT на оба ESC.
    ++stats.would_send_can_frames;
    stats.would_send_can_frames += LM_ESC_COUNT - 1;
}

void logical_motor_request_brake_current(float amps) {
    ++stats.brake_requests;
    stats.last_brake_current = amps;
    stats.last_request_us = now();
    if (!validate(amps, cfg.esc[LM_ESC_A].current_limit)) {
        return;
    }
    stats.would_send_can_frames += LM_ESC_COUNT;
}

void logical_motor_request_duty(float duty) {
    ++stats.duty_requests;
    stats.last_duty = duty;
    stats.last_request_us = now();
    if (!validate(duty, 1.0f)) {
        return;
    }
    stats.would_send_can_frames += LM_ESC_COUNT;
}

void logical_motor_set_current_off_delay(float seconds) {
    stats.last_off_delay = seconds;
}

void logical_motor_release(void) {
    ++stats.release_requests;
    memset(last_request, 0, sizeof(last_request));
}

void logical_motor_keepalive(void) {
    ++stats.keepalives;
    stats.last_keepalive_us = now();
}

LogicalMotorTelemetry logical_motor_telemetry(void) {
    LogicalMotorTelemetry t = tele;
    t.timestamp_us = now();
    return t;
}

bool logical_motor_healthy(void) {
    return tele.esc_a_alive && tele.esc_b_alive && tele.faults == LM_FAULT_NONE;
}

// ------------------------------------------------------------- управление из тестов

void logical_motor_mock_set_telemetry(const LogicalMotorTelemetry *t) {
    tele = *t;
}

void logical_motor_mock_set_esc_alive(int esc, bool alive) {
    if (esc == LM_ESC_A) {
        tele.esc_a_alive = alive;
        if (!alive) {
            tele.faults |= LM_FAULT_ESC_A_TIMEOUT;
        } else {
            tele.faults &= ~(uint32_t) LM_FAULT_ESC_A_TIMEOUT;
        }
    } else {
        tele.esc_b_alive = alive;
        if (!alive) {
            tele.faults |= LM_FAULT_ESC_B_TIMEOUT;
        } else {
            tele.faults &= ~(uint32_t) LM_FAULT_ESC_B_TIMEOUT;
        }
    }
}

void logical_motor_mock_set_esc_fault(int esc, uint8_t fault_code) {
    tele.esc_fault_code[esc] = fault_code;
    uint32_t bit = (esc == LM_ESC_A) ? LM_FAULT_ESC_A_FW : LM_FAULT_ESC_B_FW;
    if (fault_code) {
        tele.faults |= bit;
    } else {
        tele.faults &= ~bit;
    }
}

LogicalMotorMockStats logical_motor_mock_stats(void) {
    return stats;
}

void logical_motor_mock_reset_stats(void) {
    memset(&stats, 0, sizeof(stats));
}

float logical_motor_mock_last_esc_request(int esc) {
    return last_request[esc];
}
