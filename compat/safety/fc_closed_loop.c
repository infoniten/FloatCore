#include "fc_closed_loop.h"

#include "fc_build_profile.h"

#include <math.h>

// Оболочка не может быть сколь угодно большой: предел ESC на торможении 3 А,
// и оболочка обязана лежать строго ниже него. Верхняя граница здесь — не
// «разумное значение», а запрет молча превысить то, что ESC всё равно обрежет.
#define FC_CL_ENVELOPE_MAX_A 3.0f

static void deny(uint32_t *mask, FcClosedLoopDeny r) {
    if (mask) {
        *mask |= (1u << r);
    }
}

FcClosedLoopWindows fc_closed_loop_windows(const FcClosedLoopConfig *cfg) {
    FcClosedLoopWindows w = {0.0f, 0.0f};
    if (!cfg) {
        return w;
    }
    // Угол, при котором пропорциональная часть одна выбирает всю оболочку.
    if (cfg->amps_per_deg > 0.0f) {
        w.pitch_window_deg = cfg->envelope_a / cfg->amps_per_deg;
    }
    // Скорость, при которой дифференциальная часть одна выбирает всю оболочку.
    if (cfg->amps_per_deg_per_s > 0.0f) {
        w.rate_window_dps = cfg->envelope_a / cfg->amps_per_deg_per_s;
    }
    // Собственный порог Refloat — потолок, а не цель. Если наш расчёт дал
    // больше, побеждает Refloat: ослаблять его условие engage мы не вправе.
    if (cfg->refloat_pitch_tolerance_deg > 0.0f &&
        w.pitch_window_deg > cfg->refloat_pitch_tolerance_deg) {
        w.pitch_window_deg = cfg->refloat_pitch_tolerance_deg;
    }
    return w;
}

bool fc_closed_loop_may_enter(const FcClosedLoopConfig *cfg, const FcClosedLoopInputs *in,
                              uint32_t *deny_mask) {
    if (deny_mask) {
        *deny_mask = 0;
    }
    if (!cfg || !in) {
        deny(deny_mask, FC_CL_DENY_ENVELOPE);
        return false;
    }

#if !FC_CLOSED_LOOP_AVAILABLE
    // В сборке без замкнутого контура функция существует, но разрешения не
    // даёт никогда. Так её можно проверять тестами в любом профиле, не
    // заводя второй путь исполнения.
    deny(deny_mask, FC_CL_DENY_PROFILE);
#endif

    if (!(cfg->envelope_a > 0.0f) || cfg->envelope_a > FC_CL_ENVELOPE_MAX_A ||
        !isfinite(cfg->envelope_a)) {
        deny(deny_mask, FC_CL_DENY_ENVELOPE);
    }
    if (!(cfg->amps_per_deg > 0.0f) || !(cfg->amps_per_deg_per_s > 0.0f)) {
        deny(deny_mask, FC_CL_DENY_ENVELOPE);
    }

    if (!in->gate_closed_loop_enabled) {
        deny(deny_mask, FC_CL_DENY_GATE_DISABLED);
    }
    if (!in->coordinator_armed) {
        deny(deny_mask, FC_CL_DENY_NOT_ARMED);
    }
    if (!in->supervisor_entry_allowed) {
        deny(deny_mask, FC_CL_DENY_SUPERVISOR);
    }
    if (in->imu_permit != 0u) {
        deny(deny_mask, FC_CL_DENY_IMU_PERMIT);
    }
    if (!in->calibration_valid) {
        deny(deny_mask, FC_CL_DENY_CALIBRATION);
    }
    if (!in->node_a_fresh || !in->node_b_fresh) {
        deny(deny_mask, FC_CL_DENY_NODE_STALE);
    }
    if (!in->can_healthy) {
        deny(deny_mask, FC_CL_DENY_CAN_UNHEALTHY);
    }
    if (!in->motor_model_valid) {
        deny(deny_mask, FC_CL_DENY_MOTOR_MODEL);
    }
    if (!in->battery_model_valid) {
        deny(deny_mask, FC_CL_DENY_BATTERY_MODEL);
    }
    if (!in->limits_synchronized) {
        deny(deny_mask, FC_CL_DENY_LIMITS);
    }
    if (!in->realtime_qualified) {
        deny(deny_mask, FC_CL_DENY_REALTIME);
    }
    if (!in->test_condition_explicit) {
        deny(deny_mask, FC_CL_DENY_TEST_CONDITION);
    }

    FcClosedLoopWindows w = fc_closed_loop_windows(cfg);
    // NaN обязан отвергаться, а не проходить: сравнение с NaN ложно в обе
    // стороны, поэтому проверяется конечность, а не только величина.
    if (!isfinite(in->balance_pitch_deg) || !(fabsf(in->balance_pitch_deg) < w.pitch_window_deg)) {
        deny(deny_mask, FC_CL_DENY_PITCH_WINDOW);
    }
    if (!isfinite(in->pitch_rate_dps) || !(fabsf(in->pitch_rate_dps) < w.rate_window_dps)) {
        deny(deny_mask, FC_CL_DENY_RATE_WINDOW);
    }

    return deny_mask ? (*deny_mask == 0u) : false;
}

const char *fc_closed_loop_deny_name(FcClosedLoopDeny r) {
    switch (r) {
    case FC_CL_DENY_PROFILE:
        return "сборка без замкнутого контура";
    case FC_CL_DENY_GATE_DISABLED:
        return "источник Refloat не впущен в маску гейта";
    case FC_CL_DENY_NOT_ARMED:
        return "координатор не вооружён";
    case FC_CL_DENY_SUPERVISOR:
        return "супервизор не в состоянии входа";
    case FC_CL_DENY_IMU_PERMIT:
        return "политика IMU не даёт права на тягу";
    case FC_CL_DENY_CALIBRATION:
        return "ориентация датчика не откалибрована";
    case FC_CL_DENY_NODE_STALE:
        return "половина молчит";
    case FC_CL_DENY_CAN_UNHEALTHY:
        return "шина CAN в отказе";
    case FC_CL_DENY_MOTOR_MODEL:
        return "модель мотора недостоверна";
    case FC_CL_DENY_BATTERY_MODEL:
        return "модель батареи недостоверна";
    case FC_CL_DENY_LIMITS:
        return "пределы тока не синхронизированы с ESC";
    case FC_CL_DENY_REALTIME:
        return "реальное время не квалифицировано";
    case FC_CL_DENY_TEST_CONDITION:
        return "состояние footpad не задано явно";
    case FC_CL_DENY_PITCH_WINDOW:
        return "угол вне окна входа";
    case FC_CL_DENY_RATE_WINDOW:
        return "угловая скорость вне окна входа";
    case FC_CL_DENY_ENVELOPE:
        return "оболочка тока не задана или вне допустимого";
    default:
        return "?";
    }
}
