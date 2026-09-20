// Host-тесты архитектуры моторного backend (ТЗ v0.8C §3, §8, §9).
//
// Ни один тест здесь не способен ничего передать. Транспорт — тестовый,
// он записывает вызовы в массив; настоящего сериализатора моторных команд в
// репозитории нет, а его имена отравлены.

#include "../../compat/motor/fc_dual_motor.h"
#include "../../compat/motor/fc_motor_transport.h"
#include "../../compat/safety/fc_build_profile.h"
#include "../../compat/safety/fc_motor_gate.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

void fc_test_check(bool ok, const char *what);
void fc_test_note(const char *fmt, ...);
#define check fc_test_check
#define note fc_test_note

// ------------------------------------------------- тестовый транспорт (§8)

typedef struct {
    int calls;
    uint8_t half[8];
    float amps[8];
    bool fail_half[FC_DUAL_HALVES]; // впрыск отказа передачи
    uint64_t clock_us;
    uint32_t step_us; // на сколько продвигается часы между передачами
} FakeTransport;

static bool fake_send(uint8_t half, float amps, void *ctx) {
    FakeTransport *f = (FakeTransport *) ctx;
    if (f->calls < 8) {
        f->half[f->calls] = half;
        f->amps[f->calls] = amps;
    }
    ++f->calls;
    f->clock_us += f->step_us;
    return !f->fail_half[half];
}

static uint64_t fake_now(void *ctx) {
    return ((FakeTransport *) ctx)->clock_us;
}

static FcDualTransport wrap(FakeTransport *f) {
    FcDualTransport t;
    memset(&t, 0, sizeof t);
    t.name = "fake";
    t.send_current = fake_send;
    t.now_us = fake_now;
    t.ctx = f;
    return t;
}

// --------------------------------------------------------- исправные входы

#define T0 10000000ull

static FcDualMotorInputs healthy(void) {
    FcDualMotorInputs in;
    memset(&in, 0, sizeof in);
    in.now_us = T0;
    in.supervisor_allows = true;
    in.last_imu_us = T0 - 1000;
    in.last_command_us = T0 - 1000;
    in.last_feedback_us[FC_DUAL_A] = T0 - 20000;
    in.last_feedback_us[FC_DUAL_B] = T0 - 20000;
    in.requested_current_a = 2.0f;
    return in;
}

static FcDualMotorPlan plan_of(const FcDualMotorInputs *in) {
    return fc_dual_motor_plan(in);
}

static void fresh_start(void) {
    FcDualMotorConfig c = fc_dual_motor_default_config();
    fc_dual_motor_init(&c);
    fc_dual_motor_arm();
}

// Сколько отказных сценариев реально прогнано. Считаем здесь, а не по
// счётчику модуля: каждый сценарий начинается с fresh_start(), который
// обнуляет статистику, и накопленного числа там взяться неоткуда.
static int DENY_SCENARIOS;

// Проверить, что отказ назван именно этой причиной и что команды нет.
static void expect_deny(const char *name, FcDualMotorInputs *in, FcDualDenyReason r) {
    FcDualMotorPlan p = plan_of(in);
    ++DENY_SCENARIOS;
    char buf[160];
    snprintf(buf, sizeof buf, "%s: тяга запрещена", name);
    check(!p.send, buf);
    snprintf(buf, sizeof buf, "%s: причина названа (%s)", name, fc_dual_motor_reason_name(r));
    check((p.deny_mask & (uint32_t) r) != 0, buf);
    snprintf(buf, sizeof buf, "%s: команда не сформирована", name);
    check(p.current_a[FC_DUAL_A] == 0.0f && p.current_a[FC_DUAL_B] == 0.0f, buf);
}

// ------------------------------------------------------------- инварианты

static void test_semantics(void) {
    note("СЕМАНТИКА КОМАНДЫ И ЗНАК (ТЗ v0.8C §6, §7)");
    fresh_start();
    FcDualMotorInputs in = healthy();
    in.requested_current_a = 2.0f;
    FcDualMotorPlan p = plan_of(&in);

    check(p.send, "исправная система: пара команд разрешена");
    // §6: величина — ток ОДНОГО мотора. Каждая половина получает её целиком,
    // а не половину: делить пополам значило бы вдвое урезать момент доски.
    check(p.current_a[FC_DUAL_A] == 2.0f && p.current_a[FC_DUAL_B] == 2.0f,
          "ток одного мотора уходит каждой половине целиком, а не делится");
    // §7: положительный момент доски = положительная команда обеим.
    check(p.current_a[FC_DUAL_A] > 0 && p.current_a[FC_DUAL_B] > 0,
          "положительный момент доски даёт положительную команду обеим половинам");
    check(p.order[0] == FC_DUAL_A && p.order[1] == FC_DUAL_B,
          "порядок передачи фиксирован: сначала A, затем B");

    FcDualMotorConfig def = fc_dual_motor_default_config();
    check(!def.invert[FC_DUAL_A] && !def.invert[FC_DUAL_B],
          "инверсий по умолчанию нет: на v0.8B обе половины физически совпали");

    // Инверсия существует как свойство конфигурации, а не как «-1» внутри
    // транспорта — это требование §7, и оно проверяется, а не декларируется.
    FcDualMotorConfig inv = fc_dual_motor_default_config();
    inv.invert[FC_DUAL_B] = true;
    fc_dual_motor_init(&inv);
    fc_dual_motor_arm();
    p = plan_of(&in);
    check(p.current_a[FC_DUAL_A] == 2.0f && p.current_a[FC_DUAL_B] == -2.0f,
          "инверсия применяется явно и только к объявленной половине");

    in.requested_current_a = -1.5f;
    fresh_start();
    p = plan_of(&in);
    check(p.current_a[FC_DUAL_A] == -1.5f && p.current_a[FC_DUAL_B] == -1.5f,
          "отрицательный момент доски симметричен: знак у обеих половин один");
}

static void test_invariants(void) {
    note("ИНВАРИАНТЫ I3, I4, I5, I10 (ТЗ v0.8C §3)");

    // I3: состояние supervisor решает.
    fresh_start();
    FcDualMotorInputs in = healthy();
    in.supervisor_allows = false;
    expect_deny("I3 supervisor", &in, FC_DUAL_DENY_SUPERVISOR);

    // I5: обе половины обязаны быть здоровы. Проверяется поодиночке — иначе
    // отказ одной мог бы маскироваться здоровьем другой.
    fresh_start();
    in = healthy();
    in.last_feedback_us[FC_DUAL_A] = T0 - FC_DUAL_NODE_FRESH_US - 1;
    expect_deny("I5 половина A молчит", &in, FC_DUAL_DENY_NODE_A_STALE);
    fresh_start();
    in = healthy();
    in.last_feedback_us[FC_DUAL_B] = T0 - FC_DUAL_NODE_FRESH_US - 1;
    expect_deny("I5 половина B молчит", &in, FC_DUAL_DENY_NODE_B_STALE);

    // I4: защёлка. Восстановления «само собой» быть не должно.
    fresh_start();
    fc_dual_motor_report_send(true, false, T0);
    in = healthy();
    FcDualMotorPlan p = plan_of(&in);
    check(!p.send, "I4 после частичной передачи тяга запрещена");
    check((p.deny_mask & (uint32_t) FC_DUAL_DENY_LATCHED) != 0, "I4 отказ защёлкнут");
    // Даже когда всё снова исправно — защёлка держит.
    p = plan_of(&in);
    check(!p.send, "I4 исправные входы защёлку НЕ снимают");
    fc_dual_motor_clear_latch();
    p = plan_of(&in);
    check(!p.send, "I4 одного снятия защёлки мало: разрешение нужно запросить заново");
    check((p.deny_mask & (uint32_t) FC_DUAL_DENY_NOT_ARMED) != 0,
          "I4 после снятия защёлки система не вооружена");
    fc_dual_motor_arm();
    p = plan_of(&in);
    check(p.send, "I4 предусмотренная процедура возвращает тягу");

    // I10: при пропавшем источнике прошлое значение не повторяется.
    fresh_start();
    in = healthy();
    p = plan_of(&in);
    check(p.send && p.current_a[FC_DUAL_A] == 2.0f, "I10 свежая команда принята");
    in.last_command_us = T0 - FC_DUAL_COMMAND_FRESH_US - 1;
    p = plan_of(&in);
    check(!p.send, "I10 устаревшая команда не передаётся");
    check(p.current_a[FC_DUAL_A] == 0.0f && p.current_a[FC_DUAL_B] == 0.0f,
          "I10 прошлое значение НЕ повторяется, план пуст");
}

static void test_timing(void) {
    note("ВРЕМЕННЫЕ ГРАНИЦЫ I11 (ТЗ v0.8C §3)");
    // Каждая наша граница обязана быть заметно короче чужого таймаута ESC,
    // иначе барьер не успевает ни на что повлиять.
    check(FC_DUAL_COMMAND_FRESH_US < 50000u, "I11 свежесть команды короче таймаута ESC");
    check(FC_DUAL_IMU_FRESH_US < 50000u, "I11 свежесть IMU короче таймаута ESC");
    check(FC_DUAL_COMMAND_FRESH_US >= 2000u * 5,
          "допуск команды переживает 4 пропуска подряд при темпе 2 мс");
    check(FC_DUAL_NODE_FRESH_US > FC_DUAL_COMMAND_FRESH_US,
          "молчание половины подтверждается не поспешнее собственного сбоя");
    check(FC_DUAL_NODE_FRESH_US >= 3u * 20000u,
          "порог молчания половины не меньше трёх периодов STATUS 50 Гц");
    check(FC_DUAL_MAX_SKEW_US * 50u <= 50000u,
          "допустимый разбег пары не больше 2 % таймаута ESC");
    note("худшая асимметрия тяги между половинами: %u мкс",
         (unsigned) fc_dual_motor_worst_asymmetry_us());
    check(fc_dual_motor_worst_asymmetry_us() == FC_DUAL_NODE_FRESH_US + 60000u,
          "граница асимметрии посчитана, а не назначена");
}

// ------------------------------------------------- матрица отказов (§9)

static void test_fault_matrix(void) {
    note("МАТРИЦА ОТКАЗОВ, 22 сценария (ТЗ v0.8C §9)");
    FcDualMotorInputs in;
    FcDualMotorPlan p;

    // 1
    fresh_start();
    in = healthy();
    p = plan_of(&in);
    check(p.send && p.deny_mask == 0, "1 обе половины здоровы: тяга разрешена");

    // 2
    fresh_start(); in = healthy();
    in.last_imu_us = T0 - FC_DUAL_IMU_FRESH_US - 1;
    expect_deny("2 IMU устарел", &in, FC_DUAL_DENY_IMU_STALE);

    // 3
    fresh_start(); in = healthy();
    in.last_command_us = T0 - FC_DUAL_COMMAND_FRESH_US - 1;
    expect_deny("3 команда устарела", &in, FC_DUAL_DENY_COMMAND_STALE);

    // 4, 5
    fresh_start(); in = healthy();
    in.last_feedback_us[FC_DUAL_A] = 0;
    expect_deny("4 узел 118 молчит", &in, FC_DUAL_DENY_NODE_A_STALE);
    fresh_start(); in = healthy();
    in.last_feedback_us[FC_DUAL_B] = 0;
    expect_deny("5 узел 100 молчит", &in, FC_DUAL_DENY_NODE_B_STALE);

    // 6, 7
    fresh_start(); in = healthy();
    in.esc_fault_code[FC_DUAL_A] = 3;
    expect_deny("6 fault половины 118", &in, FC_DUAL_DENY_NODE_A_FAULT);
    fresh_start(); in = healthy();
    in.esc_fault_code[FC_DUAL_B] = 7;
    expect_deny("7 fault половины 100", &in, FC_DUAL_DENY_NODE_B_FAULT);

    // 8
    fresh_start(); in = healthy();
    in.last_feedback_us[FC_DUAL_A] = 0;
    in.last_feedback_us[FC_DUAL_B] = 0;
    p = plan_of(&in);
    check(!p.send && (p.deny_mask & (uint32_t) FC_DUAL_DENY_NODE_A_STALE) &&
              (p.deny_mask & (uint32_t) FC_DUAL_DENY_NODE_B_STALE),
          "8 молчат обе: названы обе причины, а не первая");

    // 9, 10
    fresh_start(); in = healthy(); in.can_degraded = true;
    expect_deny("9 шина деградировала", &in, FC_DUAL_DENY_CAN_DEGRADED);
    fresh_start(); in = healthy(); in.can_bus_off = true;
    expect_deny("10 BUS_OFF", &in, FC_DUAL_DENY_BUS_OFF);

    // 11
    fresh_start(); in = healthy(); in.supervisor_allows = false;
    expect_deny("11 supervisor в FAULT", &in, FC_DUAL_DENY_SUPERVISOR);

    // 12: перезапуск посреди пары. После init состояние чистое, но НЕ
    // разрешающее: сброс не должен оказаться способом снять защёлку.
    fresh_start();
    fc_dual_motor_report_send(true, false, T0);
    FcDualMotorConfig c = fc_dual_motor_default_config();
    fc_dual_motor_init(&c);
    in = healthy();
    p = plan_of(&in);
    check(!p.send && (p.deny_mask & (uint32_t) FC_DUAL_DENY_NOT_ARMED),
          "12 после перезапуска тяга запрещена до явного запроса разрешения");

    // 13, 14, 15: частичная передача.
    FakeTransport f;
    for (int variant = 0; variant < 2; ++variant) {
        memset(&f, 0, sizeof f);
        f.step_us = 300;
        f.fail_half[variant == 0 ? FC_DUAL_B : FC_DUAL_A] = true;
        fresh_start();
        in = healthy();
        p = plan_of(&in);
        FcDualTransport t = wrap(&f);
        FcDualSendResult r = fc_dual_motor_execute(&p, &t);
        check(!r.pair_whole, variant == 0 ? "13 первая ушла, вторая нет: пара неполная"
                                          : "15 вторая ушла, первая нет: пара неполная");
        FcDualMotorPlan next = plan_of(&in);
        check(!next.send && (next.deny_mask & (uint32_t) FC_DUAL_DENY_PARTIAL_SEND),
              variant == 0 ? "13 следующая пара запрещена" : "15 следующая пара запрещена");
    }

    // 14: транспорт отверг обе — это не расхождение половин, а просто отказ
    // передачи. Защёлкивать его нельзя, иначе единичная потеря шины навсегда
    // запрещала бы тягу.
    memset(&f, 0, sizeof f);
    f.step_us = 300;
    f.fail_half[FC_DUAL_A] = true;
    f.fail_half[FC_DUAL_B] = true;
    fresh_start();
    in = healthy();
    p = plan_of(&in);
    FcDualTransport t = wrap(&f);
    FcDualSendResult r = fc_dual_motor_execute(&p, &t);
    check(r.pair_whole, "14 отвергнуты обе: половины не разошлись");
    check(!fc_dual_motor_stats().latched, "14 симметричный отказ передачи не защёлкивается");

    // 16: источник команд встал.
    fresh_start(); in = healthy();
    in.last_command_us = T0 - 1000000;
    expect_deny("16 источник команд встал", &in, FC_DUAL_DENY_COMMAND_STALE);

    // 17, 18
    fresh_start(); in = healthy(); in.requested_current_a = NAN;
    expect_deny("17 команда NaN", &in, FC_DUAL_DENY_VALUE_INVALID);
    fresh_start(); in = healthy(); in.requested_current_a = INFINITY;
    expect_deny("18 команда Inf", &in, FC_DUAL_DENY_VALUE_INVALID);

    // 19, 20: выход за предел ЗАПРЕЩАЕТ, а не насыщает.
    fresh_start(); in = healthy(); in.requested_current_a = 5.1f;
    expect_deny("19 выше предела", &in, FC_DUAL_DENY_VALUE_RANGE);
    fresh_start(); in = healthy(); in.requested_current_a = -5.1f;
    expect_deny("20 ниже предела", &in, FC_DUAL_DENY_VALUE_RANGE);

    // 21: отметка из будущего. Так выглядит рассогласование часов или
    // переполнение счётчика; считать это свежестью нельзя.
    fresh_start(); in = healthy(); in.last_command_us = T0 + 1000;
    expect_deny("21 отметка из будущего", &in, FC_DUAL_DENY_COMMAND_STALE);

    // 22: разрыв последовательности — отметка прыгнула назад.
    fresh_start(); in = healthy();
    p = plan_of(&in);
    check(p.send, "22 первая команда принята");
    in.last_command_us = T0 - 900000;
    in.now_us = T0 + 1000;
    expect_deny("22 разрыв последовательности", &in, FC_DUAL_DENY_COMMAND_STALE);

    // Общее требование §9: ни один отказной путь не превращает старую
    // команду в новую. Проверяется на всех накопленных отказах сразу.
    note("отказных сценариев прогнано: %d", DENY_SCENARIOS);
    check(DENY_SCENARIOS >= 16, "матрица отказов действительно прогнана, а не объявлена");
}

static void test_execute_and_skew(void) {
    note("ИСПОЛНЕНИЕ ПАРЫ И РАЗБЕГ (ТЗ v0.8C §5)");
    FakeTransport f;
    memset(&f, 0, sizeof f);
    f.step_us = 256; // кадр 8 байт на 500 кбит/с
    fresh_start();
    FcDualMotorInputs in = healthy();
    FcDualMotorPlan p = plan_of(&in);
    FcDualTransport t = wrap(&f);
    FcDualSendResult r = fc_dual_motor_execute(&p, &t);

    check(f.calls == 2, "передано ровно две команды");
    check(f.half[0] == FC_DUAL_A && f.half[1] == FC_DUAL_B, "порядок A затем B соблюдён");
    check(r.pair_whole, "пара ушла целиком");
    check(r.skew_us == 256, "разбег измерен, а не предположен");
    check(r.skew_us <= FC_DUAL_MAX_SKEW_US, "разбег укладывается в объявленную границу");

    // Запрет означает тишину, а не нулевую команду: нулевая команда продлила
    // бы сторожевой таймер ESC и оставила бы половину в состоянии
    // «контроллер жив» ровно тогда, когда он не жив.
    memset(&f, 0, sizeof f);
    fresh_start();
    in.supervisor_allows = false;
    p = plan_of(&in);
    r = fc_dual_motor_execute(&p, &t);
    check(f.calls == 0, "при запрете не передаётся НИЧЕГО, даже ноль");
}

static void test_no_physical_output(void) {
    note("ФИЗИЧЕСКОГО ВЫХОДА НЕТ (ТЗ v0.8C §16, инварианты I1, I2)");
    check(FC_MOTOR_BACKEND_AVAILABLE == 0, "I2 в этом профиле backend недоступен как код");

    // I1: даже при полностью исправных входах путь наружу закрыт Motor
    // Gate-ом, у которого в этом профиле backend-а нет и быть не может.
    fc_motor_gate_init();
    FcGateVerdict v = fc_motor_gate_request(FC_MOTOR_REQ_CURRENT, 1.0f, T0);
    check(v != FC_GATE_ALLOWED, "I1 запрос тяги через Motor Gate не проходит");
    FcMotorGateStats gs = fc_motor_gate_stats();
    check(gs.physically_sent == 0, "I1 физически не отправлено ничего");
    check(gs.delivered_to_backend == 0, "I1 до backend-а ничего не дошло");
    note("вердикт Motor Gate: %s, backend: %s", fc_motor_gate_verdict_name(v),
         fc_motor_gate_backend_name());
}

void test_motor_backend_all(void) {
    test_semantics();
    test_invariants();
    test_timing();
    test_fault_matrix();
    test_execute_and_skew();
    test_no_physical_output();
}
