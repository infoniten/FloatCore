// Реализация Safety Supervisor (ТЗ v0.6A §8–§10).
//
// Правила, которые нельзя нарушить:
//   1. FAULT залипает. Выход только явным fc_supervisor_clear_fault() и
//      только когда причина исчезла. Автоматического восстановления нет:
//      контур, который «сам починился», опаснее остановленного.
//   2. Отсутствие условия и отказ — разные вещи. Ноги на доске не отказ:
//      это просто не даёт перейти в READY. Протухший IMU — отказ.
//   3. ARMED/RUNNING в лабораторной сборке недостижимы: переход в них
//      закрыт на этапе компиляции (см. ниже).

#include "fc_supervisor.h"

#include <string.h>

static struct {
    FcSupervisorState state;
    uint32_t faults;
    uint32_t faults_latched;
    uint64_t state_since_us;
    uint64_t last_loop_tick_us;
    uint64_t last_imu_sample_us;
    uint32_t transitions;
    uint32_t fault_entries;
    uint32_t imu_stale_age_us;
    uint32_t imu_worst_age_us;
    FcSupervisorImuTime imu_time;
    FcImuFaultSnapshot imu_fault;
    FcSupervisorInputs in;
    bool loop_tick_seen;
    bool imu_sample_seen;
} S;

static void transition(FcSupervisorState next, uint64_t now_us) {
    if (S.state == next) {
        return;
    }
    S.state = next;
    S.state_since_us = now_us;
    ++S.transitions;
    if (next == FC_SUP_FAULT) {
        ++S.fault_entries;
    }
}

static void enter_fault(uint32_t mask, uint64_t now_us) {
    S.faults |= mask;
    S.faults_latched |= mask;
    transition(FC_SUP_FAULT, now_us);
}

void fc_supervisor_init(uint64_t now_us) {
    memset(&S, 0, sizeof(S));
    S.state = FC_SUP_BOOT;
    S.state_since_us = now_us;
    // Значения по умолчанию для входов, которых на v0.6A физически нет.
    S.in.battery_ok = true;
    S.in.thermal_ok = true;
    S.in.watchdog_healthy = true;
}

void fc_supervisor_begin_self_test(uint64_t now_us) {
    if (S.state == FC_SUP_BOOT) {
        transition(FC_SUP_SELF_TEST, now_us);
    }
}

void fc_supervisor_self_test_result(bool passed, uint64_t now_us) {
    if (S.state != FC_SUP_SELF_TEST) {
        return;
    }
    if (passed) {
        transition(FC_SUP_DISARMED, now_us);
    } else {
        enter_fault(FC_FAULT_SELF_TEST, now_us);
    }
}

// Условия, без которых READY невозможен. Отсутствие любого из них — не отказ,
// а просто отказ в переходе.
static bool ready_conditions_met(void) {
    return S.in.platform_initialized && S.in.config_valid && S.in.loop_alive &&
           S.in.imu_healthy && S.in.watchdog_healthy && !S.in.footpad_engaged &&
           S.in.calibration_valid;
}

bool fc_supervisor_request_ready(uint64_t now_us) {
    if (S.state != FC_SUP_DISARMED) {
        return false;
    }
    if (S.faults != FC_FAULT_NONE) {
        return false;
    }
    if (!ready_conditions_met()) {
        return false;
    }
    transition(FC_SUP_READY, now_us);
    return true;
}

void fc_supervisor_disarm(uint64_t now_us) {
    // Снятие готовности разрешено из любого состояния, кроме FAULT:
    // из FAULT выход только через явное снятие отказа.
    if (S.state == FC_SUP_READY || S.state == FC_SUP_ARMED || S.state == FC_SUP_RUNNING) {
        transition(FC_SUP_DISARMED, now_us);
    }
}

void fc_supervisor_report_loop_tick(uint64_t now_us) {
    S.last_loop_tick_us = now_us;
    S.loop_tick_seen = true;
    S.in.loop_alive = true;
}

void fc_supervisor_report_imu_sample(uint64_t now_us) {
    S.last_imu_sample_us = now_us;
    S.imu_sample_seen = true;
}

// Снимок пишется ТОЛЬКО для первого отказа. Последующие ничего не добавляют:
// после защёлкивания состояние уже не то, в котором отказ возник, и перезапись
// снимка стёрла бы единственное свидетельство причины.
static void capture_imu_fault(FcImuFaultCause cause, uint32_t health_state, uint64_t now_us,
                              uint32_t age_us) {
    if (S.imu_fault.cause != FC_IMU_FAULT_CAUSE_NONE) {
        return;
    }
    S.imu_fault.cause = cause;
    S.imu_fault.health_state = health_state;
    S.imu_fault.now_us = now_us;
    S.imu_fault.computed_age_us = age_us;
    S.imu_fault.time = S.imu_time;
}

void fc_supervisor_report_imu(bool healthy, uint32_t health_state,
                              const FcSupervisorImuTime *t, uint64_t now_us) {
    if (t) {
        S.imu_time = *t;
    }
    S.in.imu_healthy = healthy;
    if (!healthy) {
        uint64_t age = (S.imu_time.last_valid_us && now_us > S.imu_time.last_valid_us)
                           ? now_us - S.imu_time.last_valid_us
                           : 0;
        if (age > 0xFFFFFFFFull) {
            age = 0xFFFFFFFFull;
        }
        capture_imu_fault(FC_IMU_FAULT_CAUSE_HEALTH_STATE, health_state, now_us, (uint32_t) age);
        enter_fault(FC_FAULT_IMU_UNHEALTHY, now_us);
    }
}

void fc_supervisor_report_imu_permit(FcImuPermit permit, uint32_t reasons, uint32_t health_state,
                                     const FcSupervisorImuTime *t, uint64_t now_us) {
    if (t) {
        S.imu_time = *t;
    }
    // OK — ориентации можно доверять. HOLD и LOST одинаково снимают вход
    // imu_healthy, но расходятся в главном: HOLD не защёлкивает ничего и
    // проходит сам, когда семпл снова станет свежим.
    S.in.imu_healthy = (permit == FC_IMU_PERMIT_OK);

    if (permit != FC_IMU_PERMIT_LOST) {
        return;
    }

    uint64_t age = (S.imu_time.last_valid_us && now_us > S.imu_time.last_valid_us)
                       ? now_us - S.imu_time.last_valid_us
                       : 0;
    if (age > 0xFFFFFFFFull) {
        age = 0xFFFFFFFFull;
    }
    capture_imu_fault(FC_IMU_FAULT_CAUSE_HEALTH_STATE, health_state, now_us, (uint32_t) age);
    S.imu_fault.policy_permit = (uint32_t) permit;
    S.imu_fault.policy_reasons = reasons;
    enter_fault(FC_FAULT_IMU_UNHEALTHY, now_us);
}

void fc_supervisor_report_imu_healthy(bool healthy, uint64_t now_us) {
    fc_supervisor_report_imu(healthy, 0, NULL, now_us);
}

const char *fc_imu_fault_cause_name(FcImuFaultCause c) {
    switch (c) {
    case FC_IMU_FAULT_CAUSE_AGE: return "возраст семпла превысил порог";
    case FC_IMU_FAULT_CAUSE_HEALTH_STATE: return "модуль здоровья вернул не OK";
    default: return "отказа IMU не было";
    }
}

void fc_supervisor_report_config_valid(bool valid, uint64_t now_us) {
    S.in.config_valid = valid;
    if (!valid) {
        enter_fault(FC_FAULT_CONFIG_INVALID, now_us);
    }
}

void fc_supervisor_report_platform_ready(bool ready, uint64_t now_us) {
    (void) now_us;
    S.in.platform_initialized = ready;
}

void fc_supervisor_report_watchdog(bool healthy, uint64_t now_us) {
    S.in.watchdog_healthy = healthy;
    if (!healthy) {
        enter_fault(FC_FAULT_WATCHDOG, now_us);
    }
}

void fc_supervisor_report_calibration_valid(bool valid, uint64_t now_us) {
    S.in.calibration_valid = valid;
    // Потеря калибровки не отказ, но и READY при ней удерживать нельзя.
    if (!valid && S.state == FC_SUP_READY) {
        transition(FC_SUP_DISARMED, now_us);
    }
}

void fc_supervisor_report_footpad(bool engaged, uint64_t now_us) {
    S.in.footpad_engaged = engaged;
    // Ноги на доске — не отказ. Но и READY при этом удерживать нельзя:
    // готовность объявляется только на свободной доске.
    if (engaged && S.state == FC_SUP_READY) {
        transition(FC_SUP_DISARMED, now_us);
    }
}

void fc_supervisor_raise_fault(uint32_t fault_mask, uint64_t now_us) {
    if (fault_mask != FC_FAULT_NONE) {
        enter_fault(fault_mask, now_us);
    }
}

void fc_supervisor_poll(uint64_t now_us) {
    // Протухание контура. Проверяется только после того, как контур хотя бы
    // раз отметился: до старта задач «протухания» ещё не существует.
    if (S.loop_tick_seen && now_us - S.last_loop_tick_us > FC_SUP_LOOP_TIMEOUT_US) {
        S.in.loop_alive = false;
        if (S.state == FC_SUP_DISARMED || S.state == FC_SUP_READY || S.state == FC_SUP_ARMED ||
            S.state == FC_SUP_RUNNING) {
            enter_fault(FC_FAULT_LOOP_STALL, now_us);
        }
    }

    // Протухание IMU. То же правило: считаем только после первого семпла.
    if (S.imu_sample_seen) {
        uint64_t age = now_us > S.last_imu_sample_us ? now_us - S.last_imu_sample_us : 0;
        if (age > 0xFFFFFFFFull) {
            age = 0xFFFFFFFFull;
        }
        if ((uint32_t) age > S.imu_worst_age_us) {
            S.imu_worst_age_us = (uint32_t) age;
        }
    }
    if (S.imu_sample_seen && now_us - S.last_imu_sample_us > FC_SUP_IMU_TIMEOUT_US) {
        S.imu_stale_age_us = (uint32_t) (now_us - S.last_imu_sample_us);
        capture_imu_fault(FC_IMU_FAULT_CAUSE_AGE, 0, now_us, S.imu_stale_age_us);
        S.in.imu_healthy = false;
        if (S.state == FC_SUP_DISARMED || S.state == FC_SUP_READY || S.state == FC_SUP_ARMED ||
            S.state == FC_SUP_RUNNING) {
            enter_fault(FC_FAULT_IMU_UNHEALTHY, now_us);
        }
    }

    // Условия READY могли пропасть без отказа — тогда просто снимаем готовность.
    if (S.state == FC_SUP_READY && !ready_conditions_met()) {
        transition(FC_SUP_DISARMED, now_us);
    }
}

bool fc_supervisor_clear_fault(uint64_t now_us) {
    // Снимок описывает ЭПИЗОД отказа. Снятие отказа эпизод закрывает, значит
    // снимок надо освободить — иначе следующий, настоящий отказ окажется
    // нечем объяснить, а место будет занято прошлым (ровно это и случилось
    // на v0.9B: снимок держал мой же тест с задержкой 50 мс).
    memset(&S.imu_fault, 0, sizeof S.imu_fault);
    if (S.state != FC_SUP_FAULT) {
        return true;
    }
    // Пересобираем активные причины из текущих входов: отказ снимается только
    // тогда, когда его причины действительно исчезли.
    uint32_t still = FC_FAULT_NONE;
    if (!S.in.config_valid) {
        still |= FC_FAULT_CONFIG_INVALID;
    }
    if (!S.in.imu_healthy) {
        still |= FC_FAULT_IMU_UNHEALTHY;
    }
    if (!S.in.loop_alive) {
        still |= FC_FAULT_LOOP_STALL;
    }
    if (!S.in.watchdog_healthy) {
        still |= FC_FAULT_WATCHDOG;
    }
    // FC_FAULT_SELF_TEST и FC_FAULT_INTERNAL по входам не восстанавливаются:
    // они означают, что проверка не пройдена или нарушен инвариант кода.
    still |= S.faults & (FC_FAULT_SELF_TEST | FC_FAULT_INTERNAL | FC_FAULT_STORAGE);

    if (still != FC_FAULT_NONE) {
        S.faults = still;
        return false;
    }
    S.faults = FC_FAULT_NONE;
    transition(FC_SUP_DISARMED, now_us);
    return true;
}

FcSupervisorStatus fc_supervisor_status(void) {
    FcSupervisorStatus st;
    memset(&st, 0, sizeof(st));
    st.state = S.state;
    st.faults = S.faults;
    st.faults_latched = S.faults_latched;
    st.state_since_us = S.state_since_us;
    st.last_loop_tick_us = S.last_loop_tick_us;
    st.last_imu_sample_us = S.last_imu_sample_us;
    st.transitions = S.transitions;
    st.fault_entries = S.fault_entries;
    st.imu_stale_age_us = S.imu_stale_age_us;
    st.imu_worst_age_us = S.imu_worst_age_us;
    st.imu_fault = S.imu_fault;
    st.inputs = S.in;
    return st;
}

FcSupervisorState fc_supervisor_state(void) {
    return S.state;
}

bool fc_supervisor_motor_output_permitted(void) {
#if !FC_MOTOR_BACKEND_AVAILABLE
    // Лабораторная сборка: разрешения нет ни в одном состоянии, и это не
    // проверка во время работы, а константа, известная компилятору.
    return false;
#else
    return S.state == FC_SUP_ARMED || S.state == FC_SUP_RUNNING;
#endif
}

bool fc_supervisor_config_write_allowed(void) {
    // Запись во flash останавливает контур на время операции (измерено:
    // до ~14 мс при периоде 2 мс, docs/realtime_timing.md). Поэтому она
    // разрешена ровно в том состоянии, где остановка контура безвредна.
    return S.state == FC_SUP_DISARMED;
}

const char *fc_supervisor_state_name(FcSupervisorState s) {
    switch (s) {
    case FC_SUP_BOOT:
        return "BOOT";
    case FC_SUP_SELF_TEST:
        return "SELF_TEST";
    case FC_SUP_DISARMED:
        return "DISARMED";
    case FC_SUP_READY:
        return "READY";
    case FC_SUP_ARMED:
        return "ARMED";
    case FC_SUP_RUNNING:
        return "RUNNING";
    case FC_SUP_FAULT:
        return "FAULT";
    default:
        return "?";
    }
}

const char *fc_supervisor_fault_name(uint32_t mask) {
    if (mask & FC_FAULT_SELF_TEST) {
        return "SELF_TEST";
    }
    if (mask & FC_FAULT_CONFIG_INVALID) {
        return "CONFIG_INVALID";
    }
    if (mask & FC_FAULT_LOOP_STALL) {
        return "LOOP_STALL";
    }
    if (mask & FC_FAULT_IMU_UNHEALTHY) {
        return "IMU_UNHEALTHY";
    }
    if (mask & FC_FAULT_WATCHDOG) {
        return "WATCHDOG";
    }
    if (mask & FC_FAULT_STORAGE) {
        return "STORAGE";
    }
    if (mask & FC_FAULT_INTERNAL) {
        return "INTERNAL";
    }
    return "none";
}
