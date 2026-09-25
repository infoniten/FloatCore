// Прогон замкнутого контура: срок, зажатие, переходы (ТЗ v0.9K §4, §6, §7, §15).
//
// Собирается в ЭКСПЕРИМЕНТАЛЬНОМ профиле с FLOATCORE_REFLOAT_REAL_MOTOR_LOOP:
// в лабораторной сборке этого кода нет, и проверить его там нельзя. Вместо
// транспорта — поддельный backend, который считает каждую отправку и
// запоминает величину. Главное, что доказывается: после срока в мотор не
// уходит НИЧЕГО, и повторного старта без нового вооружения нет.

#include "../../compat/safety/fc_build_profile.h"
#include "../../compat/safety/fc_motor_gate.h"
#include "../../compat/safety/fc_supervisor.h"

#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>

#if !FC_CLOSED_LOOP_AVAILABLE
#error "этот набор собирается только со сборкой замкнутого контура"
#endif

static int g_fail, g_checks;

static void check(bool ok, const char *what) {
    ++g_checks;
    if (!ok) {
        ++g_fail;
    }
    printf("      %s %s\n", ok ? "\033[32mPASS\033[0m" : "\033[31mFAIL\033[0m", what);
}

static void note(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    printf("  \033[90m·\033[0m ");
    vprintf(fmt, ap);
    printf("\n");
    va_end(ap);
}

// Поддельный backend: считает отправки и помнит последнюю величину.
static int g_sends;
static float g_last;
static bool fake_send(FcMotorRequestKind kind, float value, void *ctx) {
    (void) kind;
    (void) ctx;
    ++g_sends;
    g_last = value;
    return true;
}
static const FcMotorBackend FAKE = {"fake", fake_send, NULL};

#define MS 1000ull

static uint64_t bring_up(uint64_t t) {
    fc_supervisor_init(t);
    fc_supervisor_begin_self_test(t += 1000);
    fc_supervisor_self_test_result(true, t += 1000);
    fc_supervisor_report_platform_ready(true, t);
    fc_supervisor_report_config_valid(true, t);
    fc_supervisor_report_watchdog(true, t);
    fc_supervisor_report_calibration_valid(true, t);
    fc_supervisor_report_footpad(false, t);
    fc_supervisor_report_imu_healthy(true, t);
    fc_supervisor_report_loop_tick(t);
    fc_supervisor_report_imu_sample(t);
    return t;
}

// Держать контур и датчик живыми, как это делает прошивка.
static void alive(uint64_t t) {
    fc_supervisor_report_loop_tick(t);
    fc_supervisor_report_imu_sample(t);
}

static FcGateVerdict req(float a, uint64_t t) {
    return fc_motor_gate_request_from(FC_MOTOR_ORIGIN_REFLOAT, FC_MOTOR_REQ_CURRENT, a, t);
}

static uint64_t setup(uint64_t t) {
    t = bring_up(t);
    fc_motor_gate_init();
    fc_motor_gate_set_backend(&FAKE);
    fc_motor_gate_set_value_limit(5.0f);  // санитарная проверка = предел ESC
    g_sends = 0;
    return t;
}

static void test_states(void) {
    note("цепочка состояний: READY -> ARMED -> RUNNING -> DISARMED по сроку");
    uint64_t t = setup(10000000);
    check(fc_supervisor_request_ready(t += MS), "READY");
    check(!fc_supervisor_begin_burst(100 * MS, t += MS), "из READY прогон не начинается");
    check(fc_supervisor_request_closed_loop_ready(t += MS), "ARMED — готов к контуру");
    check(!fc_supervisor_motor_output_permitted(), "в ARMED выход на мотор ЗАПРЕЩЁН");
    alive(t);
    check(req(0.8f, t += MS) == FC_GATE_REJECTED_DISARMED && g_sends == 0,
          "запрос Refloat в ARMED отвергнут, отправок 0");
    check(!fc_supervisor_begin_burst(600 * MS, t += MS), "прогон длиннее 500 мс отвергнут");
    check(!fc_supervisor_begin_burst(0, t), "нулевой прогон отвергнут");
}

static void test_burst_deadline_and_clamp(void) {
    note("прогон 100 мс: зажатие ±1.5 А, после срока — ничего");
    uint64_t t = setup(20000000);
    fc_supervisor_request_ready(t += MS);
    fc_supervisor_request_closed_loop_ready(t += MS);
    uint64_t start = t += MS;
    check(fc_motor_gate_arm_burst(start + 100 * MS, 1.5f), "гейт вооружён на 100 мс");
    check(fc_supervisor_begin_burst(100 * MS, start), "RUNNING на 100 мс");
    check(fc_supervisor_motor_output_permitted(), "в RUNNING выход разрешён");

    alive(start);
    check(req(0.8f, start + 2 * MS) == FC_GATE_ALLOWED && fabsf(g_last - 0.8f) < 1e-6f,
          "0.8 А проходит без изменений");
    check(req(3.2f, start + 4 * MS) == FC_GATE_ALLOWED && fabsf(g_last - 1.5f) < 1e-6f,
          "3.2 А зажато до +1.5");
    check(req(-4.0f, start + 6 * MS) == FC_GATE_ALLOWED && fabsf(g_last + 1.5f) < 1e-6f,
          "-4.0 А зажато до -1.5");
    FcGateBurstStats b = fc_motor_gate_burst_stats();
    check(b.clamped == 2 && b.allowed == 3, "зажатия посчитаны: 2 из 3");

    // Каждые 2 мс, как контур, до срока и после.
    int before = g_sends;
    int sent_in = 0, sent_after = 0;
    for (uint64_t k = 8; k < 160; k += 2) {
        uint64_t now = start + k * MS;
        alive(now);
        int s0 = g_sends;
        (void) req(0.5f, now);
        if (g_sends != s0) {
            if (now < start + 100 * MS) {
                ++sent_in;
            } else {
                ++sent_after;
            }
        }
    }
    (void) before;
    check(sent_in > 0, "до срока команды уходят");
    check(sent_after == 0, "ПОСЛЕ срока не ушло НИ ОДНОЙ команды");
    b = fc_motor_gate_burst_stats();
    check(!b.active, "гейт отозвал прогон сам");
    check((fc_motor_gate_allowed_origins() & (1u << FC_MOTOR_ORIGIN_REFLOAT)) == 0,
          "бит REFLOAT снят");
}

static void test_supervisor_ends_independently(void) {
    note("супервизор сам выходит из RUNNING по сроку, без запросов в гейт");
    uint64_t t = setup(30000000);
    fc_supervisor_request_ready(t += MS);
    fc_supervisor_request_closed_loop_ready(t += MS);
    uint64_t start = t += MS;
    fc_motor_gate_arm_burst(start + 100 * MS, 1.5f);
    fc_supervisor_begin_burst(100 * MS, start);
    alive(start + 50 * MS);
    fc_supervisor_poll(start + 50 * MS);
    check(fc_supervisor_state() == FC_SUP_RUNNING, "до срока — RUNNING");
    alive(start + 100 * MS);
    fc_supervisor_poll(start + 100 * MS);
    check(fc_supervisor_state() == FC_SUP_DISARMED, "по сроку — DISARMED, сам");
    check(!fc_supervisor_motor_output_permitted(), "выход запрещён");
    check(!fc_supervisor_begin_burst(100 * MS, start + 101 * MS),
          "повторного старта без нового вооружения нет");
}

static void test_no_restart_on_stale_arm(void) {
    note("старый срок не даёт перезапуска даже при новом RUNNING");
    uint64_t t = setup(40000000);
    fc_supervisor_request_ready(t += MS);
    fc_supervisor_request_closed_loop_ready(t += MS);
    uint64_t start = t += MS;
    fc_motor_gate_arm_burst(start + 100 * MS, 1.5f);
    fc_supervisor_begin_burst(100 * MS, start);
    fc_supervisor_disarm(start + 20 * MS);  // оператор снял раньше срока
    // Новый RUNNING без нового вооружения гейта: срок в гейте уже прошёл.
    uint64_t t2 = start + 300 * MS;
    alive(t2);
    fc_supervisor_request_ready(t2);
    fc_supervisor_request_closed_loop_ready(t2 + MS);
    fc_supervisor_begin_burst(100 * MS, t2 + 2 * MS);
    alive(t2 + 3 * MS);
    int s0 = g_sends;
    FcGateVerdict v = req(0.5f, t2 + 3 * MS);
    check(v != FC_GATE_ALLOWED && g_sends == s0,
          "без нового вооружения гейта запрос Refloat отвергнут");
}

static void test_fault_during_burst(void) {
    note("отказ во время прогона закрывает выход сразу");
    uint64_t t = setup(50000000);
    fc_supervisor_request_ready(t += MS);
    fc_supervisor_request_closed_loop_ready(t += MS);
    uint64_t start = t += MS;
    fc_motor_gate_arm_burst(start + 100 * MS, 1.5f);
    fc_supervisor_begin_burst(100 * MS, start);
    alive(start + 10 * MS);
    check(req(0.5f, start + 10 * MS) == FC_GATE_ALLOWED, "до отказа — проходит");
    fc_supervisor_raise_fault(FC_FAULT_IMU_UNHEALTHY, start + 12 * MS);
    int s0 = g_sends;
    check(req(0.5f, start + 14 * MS) == FC_GATE_REJECTED_FAULT && g_sends == s0,
          "после отказа — отвергнуто, отправок нет");
}

static void test_invalid_values(void) {
    note("нечисловые и абсурдные значения отвергаются, а не зажимаются");
    uint64_t t = setup(60000000);
    fc_supervisor_request_ready(t += MS);
    fc_supervisor_request_closed_loop_ready(t += MS);
    uint64_t start = t += MS;
    fc_motor_gate_arm_burst(start + 100 * MS, 1.5f);
    fc_supervisor_begin_burst(100 * MS, start);
    alive(start + 2 * MS);
    int s0 = g_sends;
    check(req(NAN, start + 2 * MS) == FC_GATE_REJECTED_INVALID, "NaN отвергнут");
    check(req(9.0f, start + 4 * MS) == FC_GATE_REJECTED_INVALID,
          "9 А — выше предела ESC — отвергнуто, а не зажато до 1.5");
    check(g_sends == s0, "отправок нет");
    check(!fc_motor_gate_arm_burst(start + 200 * MS, 3.5f), "оболочка выше 3 А не принимается");
    check(!fc_motor_gate_arm_burst(start + 200 * MS, 0.0f), "нулевая оболочка не принимается");
}

static void test_footpads(void) {
    note("датчики ног: READY — только свободная доска; ARMED/RUNNING — только имитация");
    uint64_t t = setup(70000000);
    fc_supervisor_report_footpad(true, t);
    check(!fc_supervisor_request_ready(t += MS), "нажатые датчики: READY не даётся");
    fc_supervisor_report_footpad_simulated(true, t);
    check(!fc_supervisor_request_ready(t += MS), "даже при имитации READY не даётся");
    fc_supervisor_report_footpad(false, t);
    fc_supervisor_report_footpad_simulated(false, t);
    check(fc_supervisor_request_ready(t += MS), "свободная доска: READY");
    fc_supervisor_report_footpad(true, t);
    fc_supervisor_report_footpad_simulated(true, t);
    check(!fc_supervisor_request_closed_loop_ready(t += MS),
          "READY -> ARMED при нажатых датчиках не проходит (READY снят)");

    // Правильный порядок: ARMED на свободной доске, затем имитация, затем прогон.
    t = setup(71000000);
    fc_supervisor_request_ready(t += MS);
    check(fc_supervisor_request_closed_loop_ready(t += MS), "ARMED на свободной доске");
    fc_supervisor_report_footpad_simulated(true, t);
    fc_supervisor_report_footpad(true, t);
    alive(t += MS);
    fc_supervisor_poll(t);
    check(fc_supervisor_state() == FC_SUP_ARMED, "имитация нажатия ARMED не снимает");
    uint64_t start = t += MS;
    fc_motor_gate_arm_burst(start + 100 * MS, 1.5f);
    check(fc_supervisor_begin_burst(100 * MS, start), "прогон при имитации начинается");
    alive(start + 10 * MS);
    fc_supervisor_poll(start + 10 * MS);
    check(fc_supervisor_state() == FC_SUP_RUNNING, "имитация RUNNING не снимает");
    fc_supervisor_report_footpad_simulated(false, start + 12 * MS);
    alive(start + 14 * MS);
    fc_supervisor_poll(start + 14 * MS);
    check(fc_supervisor_state() == FC_SUP_DISARMED,
          "нажатие без признака имитации снимает RUNNING (при опросе)");

    // Настоящее нажатие во время прогона — немедленно, без опроса.
    t = setup(72000000);
    fc_supervisor_request_ready(t += MS);
    fc_supervisor_request_closed_loop_ready(t += MS);
    start = t += MS;
    fc_motor_gate_arm_burst(start + 100 * MS, 1.5f);
    fc_supervisor_begin_burst(100 * MS, start);
    alive(start + 2 * MS);
    fc_supervisor_report_footpad(true, start + 2 * MS);
    check(fc_supervisor_state() == FC_SUP_DISARMED,
          "настоящее нажатие снимает RUNNING сразу, без опроса");
    int s0 = g_sends;
    check(req(0.5f, start + 4 * MS) == FC_GATE_REJECTED_DISARMED && g_sends == s0,
          "и запрос Refloat после этого отвергнут");
    fc_supervisor_report_footpad(false, start + 5 * MS);
}

int main(void) {
    printf("\n\033[1mПрогон замкнутого контура: срок, зажатие, переходы\033[0m\n");
    test_states();
    test_burst_deadline_and_clamp();
    test_supervisor_ends_independently();
    test_no_restart_on_stale_arm();
    test_fault_during_burst();
    test_invalid_values();
    test_footpads();
    printf("\n================================================================\n");
    if (g_fail == 0) {
        printf("\033[32mВсе проверки пройдены\033[0m (%d)\n\n", g_checks);
        return 0;
    }
    printf("\033[31mПровалено %d из %d\033[0m\n\n", g_fail, g_checks);
    return 1;
}
