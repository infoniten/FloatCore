#include "fc_vesc_can_motor.h"

#if FC_CAN_TX_AVAILABLE

#include <math.h>
#include <string.h>

FcMotorFrameResult fc_vesc_can_motor_build_current(uint8_t target_id, float amps,
                                                   FcMotorFrame *out) {
    if (out == NULL) {
        return FC_MOTOR_FRAME_BAD_VALUE;
    }
    memset(out, 0, sizeof *out);

    // Не зажимаем, а отвергаем. Значение, дошедшее сюда неконечным, означает
    // дыру в проверках выше, и её надо увидеть, а не замазать.
    if (isnan(amps) || isinf(amps)) {
        return FC_MOTOR_FRAME_BAD_VALUE;
    }
    if (target_id == 255u || target_id == 50u) {
        return FC_MOTOR_FRAME_BAD_TARGET;
    }

    int32_t ma = (int32_t) lrintf(amps * 1000.0f);

    out->eid = (uint32_t) target_id | (FC_CAN_PACKET_SET_CURRENT << 8);
    out->data[0] = (uint8_t) ((ma >> 24) & 0xFF);
    out->data[1] = (uint8_t) ((ma >> 16) & 0xFF);
    out->data[2] = (uint8_t) ((ma >> 8) & 0xFF);
    out->data[3] = (uint8_t) (ma & 0xFF);
    out->len = 4;
    return FC_MOTOR_FRAME_OK;
}

bool fc_vesc_can_motor_decode_current(const FcMotorFrame *f, uint8_t *target_id, float *amps) {
    if (f == NULL || f->len != 4 || (f->eid >> 8) != FC_CAN_PACKET_SET_CURRENT) {
        return false;
    }
    int32_t ma = (int32_t) (((uint32_t) f->data[0] << 24) | ((uint32_t) f->data[1] << 16) |
                            ((uint32_t) f->data[2] << 8) | (uint32_t) f->data[3]);
    if (target_id) {
        *target_id = (uint8_t) (f->eid & 0xFF);
    }
    if (amps) {
        *amps = (float) ma / 1000.0f;
    }
    return true;
}

#endif // FC_CAN_TX_AVAILABLE
