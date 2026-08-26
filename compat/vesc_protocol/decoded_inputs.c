#include "decoded_inputs.h"

#include "commands.h"
#include "vesc_buffer.h"

#include <math.h>
#include <string.h>

/**
 * Ровно то же преобразование, что в прошивке:
 * `buffer_append_int32(buf, (int32_t)(value * 1000000.0), &ind)`.
 *
 * Именно поэтому здесь не используется vb_append_float32(): тот округляет
 * (lrintf), а прошивка отбрасывает дробную часть в double. Для побайтного
 * совпадения с оригиналом важно повторить и тип промежуточного вычисления,
 * и способ усечения.
 *
 * Отличие одно: диапазон ограничивается явно. В прошивке переполнение int32
 * было бы неопределённым поведением; у нас на входе значения из mock-а и
 * телеметрии, и молчаливое UB недопустимо.
 */
static void append_scaled(uint8_t *b, float v, size_t *i) {
    double d = isfinite(v) ? (double) v * 1000000.0 : 0.0;
    if (d > 2147483647.0) {
        d = 2147483647.0;
    } else if (d < -2147483648.0) {
        d = -2147483648.0;
    }
    vb_append_int32(b, (int32_t) d, i);
}

void decoded_inputs_neutral(VescDecodedInputs *out) {
    memset(out, 0, sizeof(*out));
}

void decoded_inputs_from_adc(float adc1_volts, float adc2_volts, VescDecodedInputs *out) {
    // io_read_analog() отдаёт −1.0, если вывода на плате нет. Такое значение
    // означает «канала не существует», а не «отрицательное напряжение».
    float v1 = isfinite(adc1_volts) && adc1_volts > 0.0f ? adc1_volts : 0.0f;
    float v2 = isfinite(adc2_volts) && adc2_volts > 0.0f ? adc2_volts : 0.0f;

    out->adc_voltage = v1;
    out->adc_voltage2 = v2;
    out->adc_level = v1 / DECODED_ADC_FULL_SCALE_V;
    out->adc_level2 = v2 / DECODED_ADC_FULL_SCALE_V;

    if (out->adc_level > 1.0f) {
        out->adc_level = 1.0f;
    }
    if (out->adc_level2 > 1.0f) {
        out->adc_level2 = 1.0f;
    }
}

size_t decoded_ppm_encode(const VescDecodedInputs *in, uint8_t *out, size_t cap) {
    if (cap < DECODED_PPM_REPLY_LEN) {
        return 0;
    }
    size_t ind = 0;
    vb_append_uint8(out, COMM_GET_DECODED_PPM, &ind);
    append_scaled(out, in->ppm_level, &ind);
    append_scaled(out, in->ppm_pulse_len, &ind);
    return ind;
}

size_t decoded_adc_encode(const VescDecodedInputs *in, uint8_t *out, size_t cap) {
    if (cap < DECODED_ADC_REPLY_LEN) {
        return 0;
    }
    size_t ind = 0;
    vb_append_uint8(out, COMM_GET_DECODED_ADC, &ind);
    append_scaled(out, in->adc_level, &ind);
    append_scaled(out, in->adc_voltage, &ind);
    append_scaled(out, in->adc_level2, &ind);
    append_scaled(out, in->adc_voltage2, &ind);
    return ind;
}

size_t decoded_chuk_encode(const VescDecodedInputs *in, uint8_t *out, size_t cap) {
    if (cap < DECODED_CHUK_REPLY_LEN) {
        return 0;
    }
    size_t ind = 0;
    vb_append_uint8(out, COMM_GET_DECODED_CHUK, &ind);
    append_scaled(out, in->chuk_level, &ind);
    return ind;
}
