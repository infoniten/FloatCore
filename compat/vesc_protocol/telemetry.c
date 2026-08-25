#include "telemetry.h"

#include "commands.h"
#include "vesc_buffer.h"

#include <string.h>

void telemetry_from_logical_motor(
    const LogicalMotorTelemetry *lm, float amp_hours, float watt_hours, int32_t tachometer,
    VescValues *out
) {
    memset(out, 0, sizeof(*out));

    out->temp_mos = lm->fet_temp;
    out->temp_motor = lm->motor_temp;
    out->current_motor = lm->motor_current;
    out->current_in = lm->input_current;
    out->duty = lm->duty;
    out->rpm = lm->rpm;
    out->v_in = lm->input_voltage;

    // Id/Iq по CAN недоступны: STATUS-сообщения VESC их не содержат.
    // Отдаём 0, а не выдуманное значение.
    out->id = 0.0f;
    out->iq = 0.0f;

    out->amp_hours = amp_hours;
    out->watt_hours = watt_hours;
    out->tachometer = tachometer;
    out->tachometer_abs = tachometer < 0 ? -tachometer : tachometer;

    // Код фолта: первый ненулевой из A/B (docs/motor_semantics.md §2.8).
    out->fault_code = lm->esc_fault_code[LM_ESC_A] ? lm->esc_fault_code[LM_ESC_A]
                                                   : lm->esc_fault_code[LM_ESC_B];

    out->position = -1.0f;
    out->vesc_id = 255;
    out->has_timeout = !lm->esc_a_alive || !lm->esc_b_alive;
    out->kill_sw_active = false;
}

size_t telemetry_encode(
    const VescValues *v, uint32_t mask, bool selective, uint8_t *out, size_t cap
) {
    if (cap < 80) {
        return 0;
    }

    size_t ind = 0;
    vb_append_uint8(out, selective ? COMM_GET_VALUES_SELECTIVE : COMM_GET_VALUES, &ind);
    if (selective) {
        vb_append_uint32(out, mask, &ind);
    }

    if (mask & VALUES_TEMP_MOS) {
        vb_append_float16(out, v->temp_mos, 1e1f, &ind);
    }
    if (mask & VALUES_TEMP_MOTOR) {
        vb_append_float16(out, v->temp_motor, 1e1f, &ind);
    }
    if (mask & VALUES_CURRENT_MOTOR) {
        vb_append_float32(out, v->current_motor, 1e2f, &ind);
    }
    if (mask & VALUES_CURRENT_IN) {
        vb_append_float32(out, v->current_in, 1e2f, &ind);
    }
    if (mask & VALUES_ID) {
        vb_append_float32(out, v->id, 1e2f, &ind);
    }
    if (mask & VALUES_IQ) {
        vb_append_float32(out, v->iq, 1e2f, &ind);
    }
    if (mask & VALUES_DUTY) {
        vb_append_float16(out, v->duty, 1e3f, &ind);
    }
    if (mask & VALUES_RPM) {
        vb_append_float32(out, v->rpm, 1e0f, &ind);
    }
    if (mask & VALUES_V_IN) {
        vb_append_float16(out, v->v_in, 1e1f, &ind);
    }
    if (mask & VALUES_AMP_HOURS) {
        vb_append_float32(out, v->amp_hours, 1e4f, &ind);
    }
    if (mask & VALUES_AMP_HOURS_CHARGED) {
        vb_append_float32(out, v->amp_hours_charged, 1e4f, &ind);
    }
    if (mask & VALUES_WATT_HOURS) {
        vb_append_float32(out, v->watt_hours, 1e4f, &ind);
    }
    if (mask & VALUES_WATT_HOURS_CHARGED) {
        vb_append_float32(out, v->watt_hours_charged, 1e4f, &ind);
    }
    if (mask & VALUES_TACHOMETER) {
        vb_append_int32(out, v->tachometer, &ind);
    }
    if (mask & VALUES_TACHOMETER_ABS) {
        vb_append_int32(out, v->tachometer_abs, &ind);
    }
    if (mask & VALUES_FAULT) {
        vb_append_uint8(out, v->fault_code, &ind);
    }
    if (mask & VALUES_POSITION) {
        vb_append_float32(out, v->position, 1e6f, &ind);
    }
    if (mask & VALUES_VESC_ID) {
        vb_append_uint8(out, v->vesc_id, &ind);
    }
    if (mask & VALUES_TEMP_MOS_123) {
        vb_append_float16(out, v->temp_mos, 1e1f, &ind);
        vb_append_float16(out, v->temp_mos, 1e1f, &ind);
        vb_append_float16(out, v->temp_mos, 1e1f, &ind);
    }
    if (mask & VALUES_VD) {
        vb_append_float32(out, v->vd, 1e3f, &ind);
    }
    if (mask & VALUES_VQ) {
        vb_append_float32(out, v->vq, 1e3f, &ind);
    }
    if (mask & VALUES_STATUS) {
        uint8_t status = (uint8_t) ((v->has_timeout ? 1 : 0) | (v->kill_sw_active ? 2 : 0));
        vb_append_uint8(out, status, &ind);
    }

    return ind;
}
