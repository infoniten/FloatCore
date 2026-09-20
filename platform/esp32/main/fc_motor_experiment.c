#include "fc_motor_experiment.h"

#if FC_MOTOR_BACKEND_AVAILABLE

#include "fc_can_bus.h"
#include "fc_log_port.h"
#include "fc_platform.h"

#include "../../../compat/motor/fc_dual_motor.h"
#include "../../../compat/motor/fc_motor_model.h"
#include "../../../compat/motor/fc_motor_transport.h"
#include "../../../compat/safety/fc_battery_model.h"
#include "../../../compat/safety/fc_motor_gate.h"
#include "../../../compat/safety/fc_supervisor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <string.h>

#define TAG "motor_exp"

// Адреса половин. Знание «118 — это A» принадлежит настройке, а не логике:
// координатор оперирует номерами половин, транспорт — адресами.
#define ID_A 118u
#define ID_B 100u

// Гистограмма разбега. Шаг 100 мкс до 3 мс — этого достаточно, чтобы увидеть
// и типичное значение, и хвост; точнее мерить нечем, разрешение таймера 1 мкс,
// но интерес представляет форма, а не отдельные микросекунды.
#define SKEW_BINS 32u
#define SKEW_BIN_US 100u

static struct {
    volatile bool run;
    volatile bool stop_req;
    TaskHandle_t task;
    float amps;
    uint32_t ms;
    uint32_t inject;
    FcMotorExpStats st;
    uint32_t skew_hist[SKEW_BINS];
    uint32_t skew_n;
} E;

// --------------------------------------------------------------- транспорт

static bool tx_half(uint8_t half, float amps, void *ctx) {
    (void) ctx;
    // Впрыск отказа передачи. Стоит ДО реальной передачи: смысл впрыска в
    // том, чтобы половина не получила команду, а не в том, чтобы получила и
    // мы сделали вид, что нет.
    if (half == FC_DUAL_A && (E.inject & FC_MOTOR_INJECT_FAIL_TX_A)) {
        return false;
    }
    if (half == FC_DUAL_B && (E.inject & FC_MOTOR_INJECT_FAIL_TX_B)) {
        return false;
    }
    uint8_t id = (half == FC_DUAL_A) ? ID_A : ID_B;
    bool ok = fc_can_bus_motor_send_current(id, amps);
    if (ok) {
        E.st.last_tx_us = fc_uptime_us();
    }
    return ok;
}

static uint64_t tx_now(void *ctx) {
    (void) ctx;
    return fc_uptime_us();
}

static const FcDualTransport TRANSPORT = {
    .name = "vesc-can",
    .send_current = tx_half,
    .now_us = tx_now,
    .ctx = NULL,
};

// ------------------------------------------------------- сбор входов

static uint32_t node_age_us(uint8_t id, uint64_t now) {
    FcCanHealth h = fc_can_bus_health();
    const FcCanNode *n = fc_can_health_node(&h, id);
    if (n == NULL || !n->ever_seen) {
        return 0xFFFFFFFFu;
    }
    uint64_t last = n->last_status_us > n->last_seen_us ? n->last_status_us : n->last_seen_us;
    return now > last ? (uint32_t) (now - last) : 0;
}

static FcDualMotorInputs gather(float amps, uint64_t now) {
    FcDualMotorInputs in;
    memset(&in, 0, sizeof in);
    in.now_us = now;
    in.requested_current_a = amps;

    FcSupervisorStatus s = fc_supervisor_status();
    // Экспериментальный режим не требует состояния ARMED: оно означает
    // человека на доске, а колесо вывешено. Требуется отсутствие отказа и
    // живой контур — то есть что система исправна, а не что она везёт.
    in.supervisor_allows = (s.state != FC_SUP_FAULT) && s.inputs.loop_alive &&
                           s.inputs.platform_initialized && s.inputs.config_valid;

    in.last_imu_us = s.last_imu_sample_us;
    if (E.inject & FC_MOTOR_INJECT_IMU_STALE) {
        in.last_imu_us = 0;
    }

    in.last_command_us = E.st.last_command_us;

    uint32_t age_a = node_age_us(ID_A, now);
    uint32_t age_b = node_age_us(ID_B, now);
    in.last_feedback_us[FC_DUAL_A] = (age_a == 0xFFFFFFFFu || age_a > now) ? 0 : now - age_a;
    in.last_feedback_us[FC_DUAL_B] = (age_b == 0xFFFFFFFFu || age_b > now) ? 0 : now - age_b;
    if (E.inject & FC_MOTOR_INJECT_NODE_A_STALE) {
        in.last_feedback_us[FC_DUAL_A] = 0;
    }
    if (E.inject & FC_MOTOR_INJECT_NODE_B_STALE) {
        in.last_feedback_us[FC_DUAL_B] = 0;
    }
    if (E.inject & FC_MOTOR_INJECT_NODE_A_FAULT) {
        in.esc_fault_code[FC_DUAL_A] = 0xFF;
    }
    if (E.inject & FC_MOTOR_INJECT_NODE_B_FAULT) {
        in.esc_fault_code[FC_DUAL_B] = 0xFF;
    }

    FcCanStats cs = fc_can_bus_stats();
    in.can_bus_off = (cs.bus_off_count > 0) || (E.inject & FC_MOTOR_INJECT_BUS_OFF) != 0;
    in.can_degraded = (cs.bus_error_count > 0) || (E.inject & FC_MOTOR_INJECT_CAN_DEGRADED) != 0;
    return in;
}

// --------------------------------------------------- backend для Motor Gate

static bool gate_backend_send(FcMotorRequestKind kind, float value, void *ctx) {
    (void) ctx;
    // Транспорт умеет ровно один тип. Остальные не «игнорируются», а не
    // доходят: координатор строит пару только для тока.
    if (kind != FC_MOTOR_REQ_CURRENT) {
        return false;
    }
    uint64_t now = fc_uptime_us();
    FcDualMotorInputs in = gather(value, now);
    FcDualMotorPlan p = fc_dual_motor_plan(&in);
    E.st.last_deny_mask = p.deny_mask;

    if (!p.send) {
        ++E.st.pairs_denied;
        return false;
    }

    FcDualSendResult r = fc_dual_motor_execute(&p, &TRANSPORT);
    if (r.pair_whole && r.sent[FC_DUAL_A]) {
        ++E.st.pairs_sent;
    } else if (!r.pair_whole) {
        ++E.st.pairs_partial;
    }

    E.st.skew_last_us = r.skew_us;
    if (r.skew_us > E.st.skew_max_us) {
        E.st.skew_max_us = r.skew_us;
    }
    uint32_t bin = r.skew_us / SKEW_BIN_US;
    if (bin >= SKEW_BINS) {
        bin = SKEW_BINS - 1;
    }
    ++E.skew_hist[bin];
    ++E.skew_n;
    return r.pair_whole && r.sent[FC_DUAL_A];
}

static const FcMotorBackend BACKEND = {
    .name = "vesc-can dual (experimental)",
    .send = gate_backend_send,
    .ctx = NULL,
};

// ------------------------------------------------------------- источник

static void producer(void *arg) {
    (void) arg;
    TickType_t next = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(FC_MOTOR_EXP_PERIOD_US / 1000u);
    uint64_t deadline = fc_uptime_us() + (uint64_t) E.ms * 1000ull;

    while (E.run && !E.stop_req) {
        uint64_t now = fc_uptime_us();
        if (now >= deadline) {
            break;
        }
        E.st.remaining_ms = (uint32_t) ((deadline - now) / 1000ull);

        // Впрыск остановки источника: перестаём просить, НЕ снимая вооружения.
        // Именно так выглядит зависший контур, и проверяется здесь то, что
        // тяга снимется сама, а не то, что оператор её снял.
        if (!(E.inject & FC_MOTOR_INJECT_PRODUCER_STALL)) {
            E.st.last_command_us = now;
            FcGateVerdict v =
                fc_motor_gate_request_from(FC_MOTOR_ORIGIN_EXPERIMENT, FC_MOTOR_REQ_CURRENT,
                                           E.amps, now);
            E.st.last_gate_verdict = (uint32_t) v;
            if (v == FC_GATE_ALLOWED) {
                ++E.st.gate_allowed;
            } else {
                ++E.st.gate_rejected;
            }
        }
        ++E.st.cycles;
        xTaskDelayUntil(&next, period ? period : 1);
    }

    E.run = false;
    E.st.running = false;
    E.st.remaining_ms = 0;
    E.task = NULL;
    FC_LOGI(TAG, "источник остановлен: циклов %llu, пар целиком %llu, частичных %llu",
            (unsigned long long) E.st.cycles, (unsigned long long) E.st.pairs_sent,
            (unsigned long long) E.st.pairs_partial);
    vTaskDelete(NULL);
}

// ----------------------------------------------------------------- наружу

void fc_motor_experiment_init(void) {
    memset(&E, 0, sizeof E);
    FcDualMotorConfig c = fc_dual_motor_default_config();
    // Программный предел много ниже предела ESC. Координатор ОТВЕРГАЕТ то,
    // что выше, а не подрезает: подрезание превратило бы ошибку расчёта в
    // тихо принятую команду предельной величины.
    c.current_limit_a = FC_MOTOR_EXP_MAX_CURRENT_A;
    fc_dual_motor_init(&c);
    fc_motor_gate_set_backend(&BACKEND);
    // Motor Gate тоже держит свой предел, и он должен быть согласован.
    fc_motor_gate_set_value_limit(FC_MOTOR_EXP_MAX_CURRENT_A);
}

bool fc_motor_experiment_arm(uint32_t *deny_mask) {
    FcSupervisorStatus s = fc_supervisor_status();
    FcCanStats cs = fc_can_bus_stats();
    // Модель батареи берётся из применённой конфигурации, а не из констант:
    // числа в v0.7D.1 записывались по одному и проверялись побайтово, и
    // расхождение между тем, что в ESC, и тем, что здесь, обязано быть видно.
    FcBatteryModel bm = fc_battery_model_applied();

    FcDualArmInputs a;
    memset(&a, 0, sizeof a);
    // ЗДЕСЬ НЕТ проверки s.faults_latched, и это существенно. У супервизора
    // faults_latched — история («всё, что когда-либо срабатывало»), а не
    // текущее залипание. Потребовав её нулевой, мы получили бы машину,
    // которая после любого единичного отказа не вооружается уже никогда —
    // даже после штатного fault-clear, то есть предусмотренная процедура
    // восстановления перестала бы работать.
    //
    // Залипающий отказ, который ДОЛЖЕН запрещать вооружение, — это защёлка
    // самого координатора (частичная передача), и она проверяется отдельно
    // внутри try_arm.
    a.supervisor_healthy = (s.state != FC_SUP_FAULT) && (s.faults == 0) &&
                           s.inputs.platform_initialized && s.inputs.config_valid &&
                           s.inputs.loop_alive;
    a.imu_healthy = s.inputs.imu_healthy;
    a.node_healthy[FC_DUAL_A] = node_age_us(ID_A, fc_uptime_us()) < FC_DUAL_NODE_FRESH_US;
    a.node_healthy[FC_DUAL_B] = node_age_us(ID_B, fc_uptime_us()) < FC_DUAL_NODE_FRESH_US;
    a.can_healthy = (cs.bus_off_count == 0) && (cs.bus_error_count == 0) && fc_can_bus_running();
    a.battery_model_valid = fc_battery_model_valid(&bm, NULL);
    a.motor_model_valid = fc_motor_model_ready_for_torque_test(NULL);
    a.boot_complete = fc_boot_phase_done();
    a.realtime_qualified = s.inputs.watchdog_healthy && s.inputs.loop_alive;

    return fc_dual_motor_try_arm(&a, deny_mask);
}

void fc_motor_experiment_disarm(void) {
    fc_motor_experiment_stop();
    fc_dual_motor_disarm();
}

void fc_motor_experiment_clear_latch(void) {
    fc_dual_motor_clear_latch();
}

bool fc_motor_experiment_armed(void) {
    return fc_dual_motor_stats().armed;
}

bool fc_motor_experiment_run(float amps, uint32_t ms) {
    if (E.run) {
        return false;
    }
    // Отвергаем, а не зажимаем.
    if (!isfinite(amps) || fabsf(amps) > FC_MOTOR_EXP_MAX_CURRENT_A) {
        return false;
    }
    if (ms == 0 || ms > 600000u) {
        return false;
    }
    E.amps = amps;
    E.ms = ms;
    E.stop_req = false;
    E.run = true;
    E.st.running = true;
    E.st.requested_a = amps;
    // Ядро 0: источник не относится к жёсткому реальному времени, а ядро 1
    // занято контуром. Приоритет выше фоновых задач, но ниже контура.
    BaseType_t ok = xTaskCreatePinnedToCore(producer, "motor_exp", 4096, NULL, 6, &E.task, 0);
    if (ok != pdPASS) {
        E.run = false;
        E.st.running = false;
        return false;
    }
    return true;
}

void fc_motor_experiment_stop(void) {
    E.stop_req = true;
}

void fc_motor_experiment_inject(uint32_t mask) {
    E.inject = mask;
    E.st.inject_mask = mask;
}

uint32_t fc_motor_experiment_injected(void) {
    return E.inject;
}

static uint32_t percentile(uint32_t pct) {
    if (E.skew_n == 0) {
        return 0;
    }
    uint32_t target = (E.skew_n * pct) / 100u;
    uint32_t acc = 0;
    for (uint32_t i = 0; i < SKEW_BINS; ++i) {
        acc += E.skew_hist[i];
        if (acc >= target) {
            return i * SKEW_BIN_US;
        }
    }
    return (SKEW_BINS - 1) * SKEW_BIN_US;
}

FcMotorExpStats fc_motor_experiment_stats(void) {
    FcMotorExpStats s = E.st;
    s.skew_p50_us = percentile(50);
    s.skew_p99_us = percentile(99);
    return s;
}

void fc_motor_experiment_reset_stats(void) {
    memset(&E.st, 0, sizeof E.st);
    memset(E.skew_hist, 0, sizeof E.skew_hist);
    E.skew_n = 0;
    fc_can_bus_motor_reset_stats();
}

#endif // FC_MOTOR_BACKEND_AVAILABLE
