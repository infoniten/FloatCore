// Разбор телеметрии VESC: ответ COMM_GET_VALUES и широковещательный STATUS
// (ТЗ v0.9J §11, §12).
//
// Раскладка GET_VALUES взята из tools/vesc_values.py, который уже сверялся с
// живыми половинами, а не написана заново: на v0.9A самодельный разбор этого
// же пакета ошибся на байт (забыт байт COMM-идентификатора) и показал «0 А при
// звучащем моторе». Поэтому смещения здесь — от байта COMM, и функция сама
// проверяет, что он на месте, вместо того чтобы на это полагаться.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FC_COMM_GET_VALUES_ID 4u
#define FC_CAN_PACKET_STATUS 9u

typedef struct {
    bool valid;
    float temp_fet_c;
    float temp_motor_c;
    float current_motor_a;
    float current_in_a;
    float duty;
    int32_t erpm;
    float v_in;
    int32_t tachometer;
    int32_t tachometer_abs;
    uint8_t fault_code;   // mc_fault_code, 0 = FAULT_CODE_NONE
    bool has_fault;       // хватило ли длины для кода отказа
} FcVescValues;

/**
 * Разобрать ответ GET_VALUES. buf может начинаться с байта COMM (4) или сразу с
 * тела — это определяется по первому байту и по правдоподобию напряжения.
 */
FcVescValues fc_vesc_values_decode(const uint8_t *buf, size_t len);

typedef struct {
    bool valid;
    int32_t erpm;
    float current_a;
    float duty;
} FcVescStatus1;

/** Полезная нагрузка кадра STATUS (8 байт): ERPM, ток ×10, скважность ×1000. */
FcVescStatus1 fc_vesc_status1_decode(const uint8_t *data, uint8_t dlc);

const char *fc_vesc_fault_name(uint8_t code);

// Таймаут команды из appconf (ТЗ v0.9J §9, §12). Раскладка — из
// tools/vesc_layout/release_6_06.json: controller_id на 4, timeout_msec на 5,
// timeout_brake_current на 9, смещения от начала тела (после байта COMM).
//
// Выравнивание проверяется, а не предполагается: controller_id в теле обязан
// совпасть с номером опрошенной половины. Сдвиг на байт даёт другое число, и
// ответ отвергается.
#define FC_COMM_GET_APPCONF_ID 17u

typedef struct {
    bool valid;
    uint8_t controller_id;
    uint32_t timeout_ms;
    float timeout_brake_current_a;
} FcVescAppTimeout;

FcVescAppTimeout fc_vesc_appconf_timeout(const uint8_t *buf, size_t len, uint8_t expect_id);
