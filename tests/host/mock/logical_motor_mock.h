// Тестовая надстройка над mock-реализацией LogicalMotor.
#pragma once

#include "../../../compat/motor/logical_motor.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t current_requests;
    uint32_t brake_requests;
    uint32_t duty_requests;
    uint32_t release_requests;
    uint32_t keepalives;
    uint32_t invalid_requests;   // NaN/Inf
    uint32_t clamped_requests;   // выход за current_limit
    uint32_t would_send_can_frames;  // сколько кадров ушло бы в боевом режиме
    float last_current;
    float last_brake_current;
    float last_duty;
    float last_off_delay;
    uint64_t last_request_us;
    uint64_t last_keepalive_us;
} LogicalMotorMockStats;

void logical_motor_mock_set_clock(uint64_t (*fn)(void));
void logical_motor_mock_set_esc_alive(int esc, bool alive);
void logical_motor_mock_set_esc_fault(int esc, uint8_t fault_code);
LogicalMotorMockStats logical_motor_mock_stats(void);
void logical_motor_mock_reset_stats(void);
float logical_motor_mock_last_esc_request(int esc);

void logical_motor_mock_set_telemetry(const LogicalMotorTelemetry *t);
