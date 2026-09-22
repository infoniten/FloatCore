#include "fc_vesc_mcconf.h"

#include <string.h>

// Смещения в сериализованном mcconf прошивки release_6_06, отсчитанные от
// начала ТЕЛА конфигурации (то есть от сигнатуры), а не от начала кадра
// ответа.
//
// Разница на единицу, и на ней я уже ошибся: tools/vesc_diff.py печатает
// смещения В КАДРЕ, где первый байт — номер пакета COMM, а раскладка
// confgenerator.c отсчитывает от тела. Ошибку поймала проверка
// правдоподобия ниже — при верной сигнатуре числа перестали быть похожими
// на токи. Ради этого она и написана.
//
// Проверено на реальных резервных копиях обеих половин:
//   тело[8..11]  = 40 a0 00 00 = +5.0 А
//   тело[12..15] = c0 40 00 00 = -3.0 А
//   тело[16..19] = 40 a0 00 00 = +5.0 А
//   тело[20..23] = bf 80 00 00 = -1.0 А
#define OFF_CURRENT_MAX 8u
#define OFF_CURRENT_MIN 12u
#define OFF_IN_CURRENT_MAX 16u
#define OFF_IN_CURRENT_MIN 20u
#define OFF_FLUX_LINKAGE 169u
#define OFF_MOTOR_POLES 447u

// buffer_get_float32_auto из bldc/util/buffer.c: мантисса и порядок
// упакованы в 32 бита нестандартно, поэтому обычное чтение float здесь
// неверно.
static float f32_auto(const uint8_t *b) {
    uint32_t v = ((uint32_t) b[0] << 24) | ((uint32_t) b[1] << 16) | ((uint32_t) b[2] << 8) |
                 (uint32_t) b[3];
    int e = (int) ((v >> 23) & 0xFF);
    uint32_t sig_i = v & ((1u << 23) - 1u);
    bool neg = (v & (1u << 31)) != 0u;

    float sig = (float) sig_i / (float) (1u << 23);
    if (e != 0 || sig != 0.0f) {
        sig += 1.0f;
        e -= 127;
    }

    float res = sig;
    while (e > 0) {
        res *= 2.0f;
        --e;
    }
    while (e < 0) {
        res /= 2.0f;
        ++e;
    }
    return neg ? -res : res;
}

FcMcconfLimits fc_vesc_mcconf_limits(const uint8_t *payload, uint16_t len) {
    FcMcconfLimits r;
    memset(&r, 0, sizeof r);

    if (payload == NULL || len < OFF_MOTOR_POLES + 1u) {
        return r;
    }

    uint32_t sig = ((uint32_t) payload[0] << 24) | ((uint32_t) payload[1] << 16) |
                   ((uint32_t) payload[2] << 8) | (uint32_t) payload[3];
    if (sig != FC_MCCONF_SIGNATURE) {
        // Чужая раскладка. Считать смещения от неё хуже, чем не считать.
        return r;
    }

    r.current_max = f32_auto(payload + OFF_CURRENT_MAX);
    r.current_min = f32_auto(payload + OFF_CURRENT_MIN);
    r.in_current_max = f32_auto(payload + OFF_IN_CURRENT_MAX);
    r.in_current_min = f32_auto(payload + OFF_IN_CURRENT_MIN);
    r.flux_linkage = f32_auto(payload + OFF_FLUX_LINKAGE);
    r.motor_poles = payload[OFF_MOTOR_POLES];

    // Правдоподобие. Ловит не «неверное значение», а разъехавшуюся
    // раскладку: если смещения сдвинулись, числа перестают быть похожими на
    // токи, и принимать их нельзя даже при верной сигнатуре.
    if (r.current_max <= 0.0f || r.current_max > 200.0f || r.current_min >= 0.0f ||
        r.current_min < -200.0f || r.in_current_max <= 0.0f || r.in_current_max > 200.0f ||
        r.in_current_min > 0.0f || r.in_current_min < -200.0f ||
        r.flux_linkage <= 0.0f || r.flux_linkage > 1.0f || r.motor_poles == 0 ||
        r.motor_poles > 200) {
        memset(&r, 0, sizeof r);
        return r;
    }

    r.valid = true;
    return r;
}

float fc_vesc_torque_constant(uint8_t poles, float flux_linkage) {
    // Kt = 1.5 · пар полюсов · λ. Пар полюсов вдвое меньше числа полюсов,
    // отсюда 0.5 — ровно как в motor_data.c:106.
    return 1.5f * 0.5f * (float) poles * flux_linkage;
}

FcMcconfLimits fc_vesc_mcconf_intersect(const FcMcconfLimits *a, const FcMcconfLimits *b) {
    FcMcconfLimits r;
    memset(&r, 0, sizeof r);
    if (a == NULL || b == NULL || !a->valid || !b->valid) {
        // Пересечение с неизвестным не определено. Возвращать одну из
        // половин означало бы разрешить то, что вторая может не потянуть.
        return r;
    }
    r.current_max = a->current_max < b->current_max ? a->current_max : b->current_max;
    r.current_min = a->current_min > b->current_min ? a->current_min : b->current_min;
    r.in_current_max = a->in_current_max < b->in_current_max ? a->in_current_max : b->in_current_max;
    r.in_current_min = a->in_current_min > b->in_current_min ? a->in_current_min : b->in_current_min;

    // Параметры мотора НЕ пересекаются по минимуму: половины крутят один вал,
    // и Refloat управляет ими как одним мотором. Его моментная постоянная —
    // СУММА постоянных двух моторов, потому что координатор посылает одну и
    // ту же величину обеим (docs/control_scale_audit.md §5).
    //
    // Выражаем это через эквивалентное потокосцепление: Kt складывается
    // линейно по λ при одинаковом числе полюсов, поэтому λ_экв = λ_A + λ_B.
    // Разное число полюсов означало бы разные моторы, и тогда складывать
    // нечего — такой случай отвергается.
    if (a->motor_poles != b->motor_poles) {
        r.valid = false;
        return r;
    }
    r.motor_poles = a->motor_poles;
    r.flux_linkage = a->flux_linkage + b->flux_linkage;
    r.valid = true;
    return r;
}
