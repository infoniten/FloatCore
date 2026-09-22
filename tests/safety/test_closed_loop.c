// Вход в замкнутый контур: условия и окна (ТЗ v0.9G §13, §14, §22).
//
// Набор собирается в профиле LAB_SAFE, и это здесь не ограничение, а часть
// доказательства: в лабораторной сборке разрешение не выдаётся НИ ПРИ КАКИХ
// входных данных, и это проверяется прямо.

#include "../../compat/safety/fc_build_profile.h"
#include "../../compat/safety/fc_closed_loop.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

void fc_test_check(bool ok, const char *what);
void fc_test_note(const char *fmt, ...);
#define check fc_test_check
#define note fc_test_note

// Кандидат A из v0.9F плюс оболочка §3. Числа не круглые намеренно: это те же
// величины, что подтверждены железом, а не подогнанные под тест.
static FcClosedLoopConfig good_cfg(void) {
    FcClosedLoopConfig c = {
        .envelope_a = 1.5f,
        .amps_per_deg = 2.73f,
        .amps_per_deg_per_s = 0.0819f,
        .refloat_pitch_tolerance_deg = 4.0f,
    };
    return c;
}

static FcClosedLoopInputs good_inputs(void) {
    FcClosedLoopInputs in;
    memset(&in, 0, sizeof(in));
    in.gate_closed_loop_enabled = true;
    in.coordinator_armed = true;
    in.supervisor_entry_allowed = true;
    in.imu_permit = 0;
    in.calibration_valid = true;
    in.node_a_fresh = true;
    in.node_b_fresh = true;
    in.can_healthy = true;
    in.motor_model_valid = true;
    in.battery_model_valid = true;
    in.limits_synchronized = true;
    in.realtime_qualified = true;
    in.test_condition_explicit = true;
    in.balance_pitch_deg = 0.05f;
    in.pitch_rate_dps = 0.5f;
    return in;
}

#define ONLY_PROFILE (1u << FC_CL_DENY_PROFILE)

static void test_lab_safe_never_enters(void) {
    note("в LAB_SAFE вход не разрешается ни при каких входных данных");

    FcClosedLoopConfig c = good_cfg();
    FcClosedLoopInputs in = good_inputs();
    uint32_t mask = 0;
    bool ok = fc_closed_loop_may_enter(&c, &in, &mask);

#if FC_CLOSED_LOOP_AVAILABLE
    check(ok, "при полном наборе условий вход разрешён");
    check(mask == 0, "маска отказов пуста");
#else
    check(!ok, "вход ОТКЛОНЁН: сборка без замкнутого контура");
    check((mask & ONLY_PROFILE) != 0, "причина названа: профиль");
    check(mask == ONLY_PROFILE, "и это ЕДИНСТВЕННАЯ причина — остальные условия выполнены");
#endif
}

// Каждое условие обязано ставить СВОЙ бит: иначе оператор устраняет не то.
static void test_each_condition_named(void) {
    note("каждое невыполненное условие называет себя отдельным битом");

    FcClosedLoopConfig c = good_cfg();
    uint32_t mask;

#define SPOIL(field, value, bit, what)                                                             \
    do {                                                                                           \
        FcClosedLoopInputs in = good_inputs();                                                     \
        in.field = value;                                                                          \
        mask = 0;                                                                                  \
        (void) fc_closed_loop_may_enter(&c, &in, &mask);                                           \
        check((mask & (1u << (bit))) != 0, what);                                                  \
        check((mask & ~(uint32_t) ((1u << (bit)) | ONLY_PROFILE)) == 0,                            \
              "  и ничего лишнего не приписано");                                                  \
    } while (0)

    SPOIL(gate_closed_loop_enabled, false, FC_CL_DENY_GATE_DISABLED, "гейт не впустил Refloat");
    SPOIL(coordinator_armed, false, FC_CL_DENY_NOT_ARMED, "координатор не вооружён");
    SPOIL(supervisor_entry_allowed, false, FC_CL_DENY_SUPERVISOR, "супервизор не в состоянии входа");
    SPOIL(imu_permit, 1u, FC_CL_DENY_IMU_PERMIT, "политика IMU не даёт права");
    SPOIL(calibration_valid, false, FC_CL_DENY_CALIBRATION, "калибровка недействительна");
    SPOIL(node_a_fresh, false, FC_CL_DENY_NODE_STALE, "половина A молчит");
    SPOIL(node_b_fresh, false, FC_CL_DENY_NODE_STALE, "половина B молчит");
    SPOIL(can_healthy, false, FC_CL_DENY_CAN_UNHEALTHY, "шина в отказе");
    SPOIL(motor_model_valid, false, FC_CL_DENY_MOTOR_MODEL, "модель мотора недостоверна");
    SPOIL(battery_model_valid, false, FC_CL_DENY_BATTERY_MODEL, "модель батареи недостоверна");
    SPOIL(limits_synchronized, false, FC_CL_DENY_LIMITS, "пределы не синхронизированы");
    SPOIL(realtime_qualified, false, FC_CL_DENY_REALTIME, "реальное время не квалифицировано");
    SPOIL(test_condition_explicit, false, FC_CL_DENY_TEST_CONDITION, "footpad не задан явно");
#undef SPOIL
}

static void test_entry_windows(void) {
    note("окна входа выводятся из оболочки, а не задаются числом");

    FcClosedLoopConfig c = good_cfg();
    FcClosedLoopWindows w = fc_closed_loop_windows(&c);

    // 1.5 А / 2.73 А на градус = 0.549°
    check(fabsf(w.pitch_window_deg - 0.5495f) < 0.001f, "окно угла = оболочка / (А на градус)");
    // 1.5 А / 0.0819 А на °/с = 18.3 °/с
    check(fabsf(w.rate_window_dps - 18.315f) < 0.01f, "окно скорости = оболочка / (А на °/с)");

    // Порог самого Refloat — потолок. Если наш расчёт дал больше, побеждает он.
    FcClosedLoopConfig wide = good_cfg();
    wide.amps_per_deg = 0.1f;  // очень мягкий контур: расчётное окно 15°
    FcClosedLoopWindows ww = fc_closed_loop_windows(&wide);
    check(ww.pitch_window_deg <= 4.0f + 1e-6f, "окно угла не превышает startup_pitch_tolerance");
    check(fabsf(ww.pitch_window_deg - 4.0f) < 1e-6f, "и упирается ровно в него");
}

static void test_window_boundaries(void) {
    note("границы окна и нечисловые значения");

    FcClosedLoopConfig c = good_cfg();
    uint32_t mask;

    // Ровно на границе входить нельзя: там контур стартует уже в упоре.
    FcClosedLoopInputs at = good_inputs();
    at.balance_pitch_deg = 0.5495f;
    mask = 0;
    (void) fc_closed_loop_may_enter(&c, &at, &mask);
    check((mask & (1u << FC_CL_DENY_PITCH_WINDOW)) != 0, "угол РОВНО на границе отвергается");

    FcClosedLoopInputs nan_pitch = good_inputs();
    nan_pitch.balance_pitch_deg = NAN;
    mask = 0;
    (void) fc_closed_loop_may_enter(&c, &nan_pitch, &mask);
    check((mask & (1u << FC_CL_DENY_PITCH_WINDOW)) != 0, "NaN в угле отвергается, а не проходит");

    FcClosedLoopInputs nan_rate = good_inputs();
    nan_rate.pitch_rate_dps = NAN;
    mask = 0;
    (void) fc_closed_loop_may_enter(&c, &nan_rate, &mask);
    check((mask & (1u << FC_CL_DENY_RATE_WINDOW)) != 0, "NaN в скорости отвергается");

    // Знак роли не играет: окно симметрично.
    FcClosedLoopInputs neg = good_inputs();
    neg.balance_pitch_deg = -0.6f;
    mask = 0;
    (void) fc_closed_loop_may_enter(&c, &neg, &mask);
    check((mask & (1u << FC_CL_DENY_PITCH_WINDOW)) != 0, "отрицательный угол вне окна отвергается");
}

static void test_envelope_bounds(void) {
    note("оболочка не может молча превысить предел ESC на торможении");

    uint32_t mask;
    FcClosedLoopInputs in = good_inputs();

    FcClosedLoopConfig big = good_cfg();
    big.envelope_a = 3.5f;  // выше предела -3 А
    mask = 0;
    (void) fc_closed_loop_may_enter(&big, &in, &mask);
    check((mask & (1u << FC_CL_DENY_ENVELOPE)) != 0, "оболочка выше 3 А отвергается");

    FcClosedLoopConfig zero = good_cfg();
    zero.envelope_a = 0.0f;
    mask = 0;
    (void) fc_closed_loop_may_enter(&zero, &in, &mask);
    check((mask & (1u << FC_CL_DENY_ENVELOPE)) != 0, "нулевая оболочка отвергается");

    FcClosedLoopConfig edge = good_cfg();
    edge.envelope_a = 3.0f;  // ровно предел: допустимо, ESC его и так обрежет
    mask = 0;
    (void) fc_closed_loop_may_enter(&edge, &in, &mask);
    check((mask & (1u << FC_CL_DENY_ENVELOPE)) == 0, "оболочка ровно 3 А допустима");

    mask = 0;
    check(!fc_closed_loop_may_enter(NULL, &in, &mask), "без конфигурации разрешения нет");
    check(!fc_closed_loop_may_enter(&edge, NULL, &mask), "без входов разрешения нет");
}

static void test_deny_names(void) {
    note("у каждой причины есть имя");
    for (int i = 0; i < FC_CL_DENY_COUNT; ++i) {
        const char *n = fc_closed_loop_deny_name((FcClosedLoopDeny) i);
        if (!n || n[0] == '?') {
            check(false, "причина без имени");
            return;
        }
    }
    check(true, "все причины названы");
    check(FC_CL_DENY_COUNT <= 32, "маска причин помещается в 32 бита");
}

void test_closed_loop_all(void) {
    test_lab_safe_never_enters();
    test_each_condition_named();
    test_entry_windows();
    test_window_boundaries();
    test_envelope_bounds();
    test_deny_names();
}
