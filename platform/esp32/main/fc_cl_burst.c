#include "fc_cl_burst.h"

#if FC_CLOSED_LOOP_AVAILABLE

#include "fc_motor_experiment.h"
#include "fc_platform.h"

#include "../../../compat/safety/fc_supervisor.h"

#include <string.h>

// Писатель один — поток Refloat. Читатель — консоль, и читает только после
// fc_cl_log_stop(), поэтому блокировки не нужны: флаги volatile, записи
// окончены до того, как журнал выключен.
static struct {
    volatile bool pre_on;
    volatile bool main_on;
    volatile bool dry;
    uint64_t start_us, deadline_us, end_us;
    uint32_t seq;
    uint32_t pre_head, pre_total;
    uint32_t main_n, dropped;
    FcClRecord pre[FC_CL_LOG_PRE];
    FcClRecord main[FC_CL_LOG_MAX];
} L;

void fc_cl_log_arm(void) {
    L.main_on = false;
    L.pre_on = false;
    L.start_us = L.deadline_us = L.end_us = 0;
    L.seq = 0;
    L.pre_head = L.pre_total = 0;
    L.main_n = L.dropped = 0;
    L.pre_on = true;
}

void fc_cl_log_start(uint64_t start_us, uint64_t deadline_us, uint64_t tail_us) {
    L.start_us = start_us;
    L.deadline_us = deadline_us;
    L.end_us = deadline_us + tail_us;
    L.main_on = true;
}

void fc_cl_log_stop(void) {
    L.pre_on = false;
    L.main_on = false;
    L.dry = false;
}

void fc_cl_log_set_dry(bool on) {
    L.dry = on;
}

bool fc_cl_log_active(void) {
    return L.pre_on || L.main_on;
}

FcGateVerdict fc_cl_request_current(const RefloatShadowFields *rf, float current) {
    uint64_t now = fc_uptime_us();
    // Свежесть команды для координатора: Refloat просит именно сейчас.
    fc_motor_experiment_note_command(now);

    if (!L.pre_on && !L.main_on) {
        return fc_motor_gate_request(FC_MOTOR_REQ_CURRENT, current, now);
    }

    FcMotorTxTrace before, after;
    fc_motor_experiment_tx_trace(&before);
    FcGateVerdict v = fc_motor_gate_request(FC_MOTOR_REQ_CURRENT, current, now);
    uint64_t t1 = fc_uptime_us();
    fc_motor_experiment_tx_trace(&after);

    FcClRecord *r;
    if (L.main_on && now >= L.start_us) {
        if (now > L.end_us) {
            L.main_on = false;
            return v;
        }
        if (L.main_n >= FC_CL_LOG_MAX) {
            ++L.dropped;
            return v;
        }
        r = &L.main[L.main_n++];
    } else if (L.pre_on) {
        r = &L.pre[L.pre_head];
        L.pre_head = (L.pre_head + 1u) % FC_CL_LOG_PRE;
        ++L.pre_total;
    } else {
        return v;
    }

    r->seq = L.seq++;
    r->t_us = now;
    r->pitch = rf->pitch;
    r->balance_pitch = rf->balance_pitch;
    r->pitch_rate = rf->pitch_rate;
    r->setpoint = rf->setpoint;
    r->raw_a = current;
    r->verdict = (uint8_t) v;
    r->gate_us = (uint16_t) ((t1 - now) > 0xFFFFu ? 0xFFFFu : (t1 - now));
    r->sup_state = (uint8_t) fc_supervisor_state();
    r->imu_permit = (uint8_t) fc_supervisor_last_imu_permit();
    r->refloat_state = (uint8_t) rf->state;
    r->deny_mask = after.deny_mask;

    uint8_t f = 0;
    if (after.pairs_sent != before.pairs_sent) {
        f |= FC_CL_F_PAIR;
    }
    if (after.pairs_partial != before.pairs_partial) {
        f |= FC_CL_F_PARTIAL;
    }
    if (after.pairs_denied != before.pairs_denied) {
        f |= FC_CL_F_DENIED;
    }
    // Кадр «этого цикла» — тот, чьё время сдвинулось за время вызова.
    for (int k = 0; k < 2; ++k) {
        bool fresh = after.tx_us[k] != before.tx_us[k];
        r->tx_us[k] = fresh ? after.tx_us[k] : 0;
        r->tx_a[k] = fresh ? after.tx_amps[k] : 0.0f;
        if (fresh && after.tx_ok[k]) {
            f |= k == 0 ? FC_CL_F_TX_A_OK : FC_CL_F_TX_B_OK;
        }
    }
    r->skew_us = (f & FC_CL_F_PAIR) ? (uint16_t) (after.skew_us > 0xFFFFu ? 0xFFFFu : after.skew_us)
                                    : 0;
    r->delivered_a = (f & FC_CL_F_PAIR) ? after.tx_amps[0] : 0.0f;
    r->flags = f;
    r->dry_us = 0;
    if (L.dry) {
        uint32_t d = fc_motor_experiment_dry_plan_us();
        r->dry_us = (uint16_t) (d > 0xFFFFu ? 0xFFFFu : d);
    }
    return v;
}

FcClLogInfo fc_cl_log_info(void) {
    FcClLogInfo i = {
        .start_us = L.start_us,
        .deadline_us = L.deadline_us,
        .end_us = L.end_us,
        .pre_n = L.pre_total < FC_CL_LOG_PRE ? L.pre_total : FC_CL_LOG_PRE,
        .pre_total = L.pre_total,
        .main_n = L.main_n,
        .dropped = L.dropped,
    };
    return i;
}

uint32_t fc_cl_log_pre(FcClRecord *out, uint32_t max) {
    uint32_t n = L.pre_total < FC_CL_LOG_PRE ? L.pre_total : FC_CL_LOG_PRE;
    if (n > max) {
        n = max;
    }
    // Самая старая запись — там, куда писалась бы следующая, если кольцо полно.
    uint32_t first = L.pre_total < FC_CL_LOG_PRE ? 0 : L.pre_head;
    for (uint32_t i = 0; i < n; ++i) {
        out[i] = L.pre[(first + i) % FC_CL_LOG_PRE];
    }
    return n;
}

const FcClRecord *fc_cl_log_main(uint32_t *n) {
    *n = L.main_n;
    return L.main;
}

#endif // FC_CLOSED_LOOP_AVAILABLE
