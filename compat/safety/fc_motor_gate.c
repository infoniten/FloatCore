// Реализация Motor Gate (ТЗ v0.6A §11, §12).
//
// Единственный путь наружу. Обойти нельзя по построению: указатель на backend
// приватен для этой единицы трансляции, а в сборке LAB_SAFE функция его
// установки вообще не компилируется — см. #ifdef ниже и fc_build_profile.h.

#include "fc_motor_gate.h"
#include "fc_supervisor.h"

#include <math.h>
#include <string.h>

static struct {
    FcMotorGateStats stats;
    const FcMotorBackend *backend;
    float value_limit;
} G;

// Значение по умолчанию — политика FloatCore, а не число из datasheet.
// Выбрано заведомо больше любого осмысленного запроса Refloat при
// консервативных лимитах ESC и одновременно конечным, чтобы NaN/Inf и
// абсурдные величины отсекались до всякой политики состояний.
#define FC_GATE_DEFAULT_VALUE_LIMIT 200.0f

void fc_motor_gate_init(void) {
    memset(&G, 0, sizeof(G));
    G.value_limit = FC_GATE_DEFAULT_VALUE_LIMIT;
}

void fc_motor_gate_set_value_limit(float limit) {
    if (isfinite(limit) && limit > 0.0f) {
        G.value_limit = limit;
    }
}

#if FC_MOTOR_BACKEND_AVAILABLE
void fc_motor_gate_set_backend(const FcMotorBackend *backend) {
    G.backend = backend;
}
#endif

const char *fc_motor_gate_backend_name(void) {
    return G.backend && G.backend->name ? G.backend->name : "none (blocked)";
}

FcGateVerdict fc_motor_gate_request(FcMotorRequestKind kind, float value, uint64_t now_us) {
    if ((unsigned) kind >= FC_MOTOR_REQ_KIND_COUNT) {
        ++G.stats.requests_total;
        ++G.stats.rejected_invalid;
        return FC_GATE_REJECTED_INVALID;
    }

    ++G.stats.requests_total;
    ++G.stats.by_kind[kind];
    G.stats.last_value[kind] = value;
    G.stats.last_request_us = now_us;

    // 1. Санитарная проверка значения — раньше всякой политики состояний.
    //    Проверено сценарием 10 host-харнесса: NaN в гироскопе доходит до
    //    запроса тока, и Refloat этого не замечает.
    if (!isfinite(value) || fabsf(value) > G.value_limit) {
        ++G.stats.rejected_invalid;
        return FC_GATE_REJECTED_INVALID;
    }

    // 2. Отказ супервизора важнее всего остального.
    if (fc_supervisor_state() == FC_SUP_FAULT) {
        ++G.stats.rejected_fault;
        return FC_GATE_REJECTED_FAULT;
    }

    // 3. Разрешает ли состояние вообще выход на мотор.
    if (!fc_supervisor_motor_output_permitted()) {
        ++G.stats.rejected_disarmed;
        return FC_GATE_REJECTED_DISARMED;
    }

    ++G.stats.allowed_by_policy;

    // 4. Даже когда политика разрешила, отправлять может быть некому.
    if (!G.backend || !G.backend->send) {
        ++G.stats.rejected_no_backend;
        return FC_GATE_REJECTED_NO_BACKEND;
    }

    ++G.stats.delivered_to_backend;
    if (G.backend->send(kind, value, G.backend->ctx)) {
        ++G.stats.physically_sent;
    }
    return FC_GATE_ALLOWED;
}

void fc_motor_gate_keepalive(void) {
    ++G.stats.keepalive_calls;
}

FcMotorGateStats fc_motor_gate_stats(void) {
    return G.stats;
}

const char *fc_motor_gate_kind_name(FcMotorRequestKind kind) {
    switch (kind) {
    case FC_MOTOR_REQ_CURRENT:
        return "set_current";
    case FC_MOTOR_REQ_BRAKE_CURRENT:
        return "set_brake_current";
    case FC_MOTOR_REQ_CURRENT_REL:
        return "set_current_rel";
    case FC_MOTOR_REQ_BRAKE_CURRENT_REL:
        return "set_brake_current_rel";
    case FC_MOTOR_REQ_DUTY:
        return "set_duty";
    case FC_MOTOR_REQ_DUTY_NORAMP:
        return "set_duty_noramp";
    case FC_MOTOR_REQ_PID_SPEED:
        return "set_pid_speed";
    case FC_MOTOR_REQ_PID_POS:
        return "set_pid_pos";
    case FC_MOTOR_REQ_HANDBRAKE:
        return "set_handbrake";
    case FC_MOTOR_REQ_HANDBRAKE_REL:
        return "set_handbrake_rel";
    case FC_MOTOR_REQ_RELEASE:
        return "release_motor";
    case FC_MOTOR_REQ_TONE:
        return "foc_play_tone";
    default:
        return "?";
    }
}

const char *fc_motor_gate_verdict_name(FcGateVerdict v) {
    switch (v) {
    case FC_GATE_ALLOWED:
        return "allowed";
    case FC_GATE_REJECTED_DISARMED:
        return "rejected_disarmed";
    case FC_GATE_REJECTED_FAULT:
        return "rejected_fault";
    case FC_GATE_REJECTED_INVALID:
        return "rejected_invalid";
    case FC_GATE_REJECTED_NO_BACKEND:
        return "rejected_no_backend";
    default:
        return "?";
    }
}
