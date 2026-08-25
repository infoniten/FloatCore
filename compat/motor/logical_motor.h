// Copyright 2026 ESP32 Refloat Compatibility Controller
//
// Контракт «один логический мотор» (ТЗ v0.2 §4).
//
// Refloat считает, что управляет одним мотором. Реализация этого интерфейса
// разворачивает запрос в команды двум физическим VESC по CAN. На этапе 0.2
// существует только mock-реализация: запросы записываются в буфер, CAN не трогается.
//
// Интерфейс намеренно на C: и Refloat, и ESP-IDF — C, а C++-обёртка (класс из ТЗ)
// при необходимости строится поверх без изменения ABI.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define LM_ESC_A 0
#define LM_ESC_B 1
#define LM_ESC_COUNT 2

/** Флаги неисправностей логического мотора (битовая маска поля `faults`). */
typedef enum {
    LM_FAULT_NONE = 0,
    LM_FAULT_ESC_A_TIMEOUT = 1 << 0,
    LM_FAULT_ESC_B_TIMEOUT = 1 << 1,
    LM_FAULT_ESC_A_FW = 1 << 2,  // ненулевой mc_fault_code от VESC A
    LM_FAULT_ESC_B_FW = 1 << 3,  // ненулевой mc_fault_code от VESC B
    LM_FAULT_ERPM_MISMATCH = 1 << 4,
    LM_FAULT_VOLTAGE_MISMATCH = 1 << 5,
    LM_FAULT_INVALID_REQUEST = 1 << 6,  // NaN/Inf или выход за лимит
    LM_FAULT_BUS_OFF = 1 << 7,
} LogicalMotorFault;

/** Агрегированная телеметрия. Правила агрегации — docs/motor_semantics.md. */
typedef struct {
    float rpm;             // ERPM, среднее A/B
    float duty;            // max(|A|,|B|), безразмерный 0..1
    float motor_current;   // А, среднее A/B (НЕ сумма, см. motor_semantics.md)
    float input_current;   // А, среднее A/B
    float input_voltage;   // В, среднее A/B
    float fet_temp;        // °C, max(A,B)
    float motor_temp;      // °C, max(A,B)
    uint32_t faults;       // битовая маска LogicalMotorFault
    uint8_t esc_fault_code[LM_ESC_COUNT];  // сырые mc_fault_code от каждого ESC
    bool esc_a_alive;
    bool esc_b_alive;
    uint64_t timestamp_us;  // время формирования снимка
} LogicalMotorTelemetry;

/** Конфигурация одного физического ESC. */
typedef struct {
    uint8_t can_id;
    bool invert;          // инверсия направления/знака тока
    float scale;          // масштаб запрошенного тока, 0..1
    float current_limit;  // А, лимит на этот мотор
} LogicalMotorEscConfig;

typedef struct {
    LogicalMotorEscConfig esc[LM_ESC_COUNT];
    float telemetry_timeout_s;  // старше — ESC считается offline
    float erpm_mismatch_limit;  // допустимое расхождение ERPM
    float voltage_mismatch_limit;  // В
} LogicalMotorConfig;

void logical_motor_init(const LogicalMotorConfig *config);

/**
 * Запрос тока. Значение — ток ОДНОГО мотора; каждому ESC уходит одно и то же
 * значение (с учётом scale/invert/лимита).
 *
 * Отвергает NaN/Inf и значения вне лимита, выставляя LM_FAULT_INVALID_REQUEST.
 */
void logical_motor_request_current(float amps);

/** Запрос тормозного тока (положительное значение). */
void logical_motor_request_brake_current(float amps);

/** Запрос режима duty (используется Refloat как парковочный тормоз). */
void logical_motor_request_duty(float duty);

/** Удержание модуляции при токе около нуля (mc_set_current_off_delay). */
void logical_motor_set_current_off_delay(float seconds);

/** Освобождение мотора: прекращение тяги. */
void logical_motor_release(void);

/** Сброс watchdog-таймера (аналог VESC_IF->timeout_reset). */
void logical_motor_keepalive(void);

LogicalMotorTelemetry logical_motor_telemetry(void);

/** true, если оба ESC на связи и нет активных фолтов. */
bool logical_motor_healthy(void);
