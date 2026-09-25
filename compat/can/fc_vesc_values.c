#include "fc_vesc_values.h"

#include "fc_vesc_mcconf.h"

static int16_t be16(const uint8_t *p) {
    return (int16_t) (((uint16_t) p[0] << 8) | p[1]);
}

static int32_t be32(const uint8_t *p) {
    return (int32_t) (((uint32_t) p[0] << 24) | ((uint32_t) p[1] << 16) | ((uint32_t) p[2] << 8) |
                      p[3]);
}

// Смещения от байта COMM (tools/vesc_values.py, bldc release_6_06).
#define OFF_TEMP_FET 1
#define OFF_TEMP_MOTOR 3
#define OFF_CURRENT_MOTOR 5
#define OFF_CURRENT_IN 9
#define OFF_DUTY 21
#define OFF_RPM 23
#define OFF_V_IN 27
#define OFF_TACH 45
#define OFF_TACH_ABS 49
#define OFF_FAULT 53
#define MIN_LEN 53  // до тахометра включительно

static FcVescValues decode_at(const uint8_t *b, size_t len) {
    FcVescValues v = {0};
    if (len < MIN_LEN) {
        return v;
    }
    v.temp_fet_c = be16(b + OFF_TEMP_FET) / 10.0f;
    v.temp_motor_c = be16(b + OFF_TEMP_MOTOR) / 10.0f;
    v.current_motor_a = be32(b + OFF_CURRENT_MOTOR) / 100.0f;
    v.current_in_a = be32(b + OFF_CURRENT_IN) / 100.0f;
    v.duty = be16(b + OFF_DUTY) / 1000.0f;
    v.erpm = be32(b + OFF_RPM);
    v.v_in = be16(b + OFF_V_IN) / 10.0f;
    v.tachometer = be32(b + OFF_TACH);
    v.tachometer_abs = be32(b + OFF_TACH_ABS);
    if (len > OFF_FAULT) {
        v.fault_code = b[OFF_FAULT];
        v.has_fault = true;
    }
    v.valid = true;
    return v;
}

FcVescValues fc_vesc_values_decode(const uint8_t *buf, size_t len) {
    FcVescValues none = {0};
    if (!buf || len == 0) {
        return none;
    }
    // Ответ с байтом COMM — основной случай. Тело без него разбирается со
    // сдвигом на один байт назад: для этого перед ним нужен виртуальный байт,
    // поэтому второй вариант проверяется только если первый неправдоподобен.
    if (buf[0] == FC_COMM_GET_VALUES_ID) {
        FcVescValues v = decode_at(buf, len);
        if (v.valid && v.v_in > 5.0f && v.v_in < 100.0f) {
            return v;
        }
    }
    // Тело без байта COMM: смещения уменьшаются на единицу.
    if (len + 1 >= MIN_LEN) {
        static uint8_t tmp[128];
        size_t n = len + 1 < sizeof(tmp) ? len + 1 : sizeof(tmp);
        tmp[0] = FC_COMM_GET_VALUES_ID;
        for (size_t i = 1; i < n; ++i) {
            tmp[i] = buf[i - 1];
        }
        FcVescValues v = decode_at(tmp, n);
        if (v.valid && v.v_in > 5.0f && v.v_in < 100.0f) {
            return v;
        }
    }
    // Ни одно выравнивание не дало правдоподобного напряжения: отдать
    // «недействительно», а не числа, которые могут оказаться сдвинутыми.
    return none;
}

FcVescStatus1 fc_vesc_status1_decode(const uint8_t *d, uint8_t dlc) {
    FcVescStatus1 s = {0};
    if (!d || dlc < 8) {
        return s;
    }
    s.erpm = be32(d);
    s.current_a = be16(d + 4) / 10.0f;
    s.duty = be16(d + 6) / 1000.0f;
    s.valid = true;
    return s;
}

const char *fc_vesc_fault_name(uint8_t code) {
    static const char *const N[] = {
        "NONE", "OVER_VOLTAGE", "UNDER_VOLTAGE", "DRV", "ABS_OVER_CURRENT",
        "OVER_TEMP_FET", "OVER_TEMP_MOTOR", "GATE_DRIVER_OVER_VOLTAGE",
        "GATE_DRIVER_UNDER_VOLTAGE", "MCU_UNDER_VOLTAGE", "BOOTING_FROM_WATCHDOG_RESET",
        "ENCODER_SPI", "ENCODER_SINCOS_BELOW_MIN_AMPLITUDE", "ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE",
        "FLASH_CORRUPTION", "HIGH_OFFSET_CURRENT_SENSOR_1", "HIGH_OFFSET_CURRENT_SENSOR_2",
        "HIGH_OFFSET_CURRENT_SENSOR_3", "UNBALANCED_CURRENTS", "BRK", "RESOLVER_LOT",
        "RESOLVER_DOS", "RESOLVER_LOS", "FLASH_CORRUPTION_APP_CFG", "FLASH_CORRUPTION_MC_CFG",
        "ENCODER_NO_MAGNET", "ENCODER_MAGNET_TOO_STRONG", "PHASE_FILTER", "ENCODER_FAULT",
        "LV_OUTPUT_FAULT",
    };
    return code < sizeof(N) / sizeof(N[0]) ? N[code] : "?";
}

FcVescAppTimeout fc_vesc_appconf_timeout(const uint8_t *buf, size_t len, uint8_t expect_id) {
    FcVescAppTimeout r = {0};
    if (!buf || len < 14) {
        return r;
    }
    // Два варианта начала: с байтом COMM и без него. Годен только тот, где
    // controller_id совпал с опрошенной половиной.
    for (int skip = 1; skip >= 0; --skip) {
        if (skip == 1 && buf[0] != FC_COMM_GET_APPCONF_ID) {
            continue;
        }
        const uint8_t *b = buf + skip;
        if (len < (size_t) skip + 13u) {
            continue;
        }
        if (b[4] != expect_id) {
            continue;
        }
        r.controller_id = b[4];
        r.timeout_ms = (uint32_t) be32(b + 5);
        r.timeout_brake_current_a = fc_vesc_float32_auto(b + 9);
        r.valid = true;
        return r;
    }
    return r;
}
