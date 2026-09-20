#include "fc_dual_motor.h"

#include <math.h>
#include <string.h>

static FcDualMotorConfig CFG;
static FcDualMotorStats ST;
static uint32_t LAST_SKEW_US;

// Обе половины получили команду в прошлый раз. Нужно, чтобы отличить «пара
// не уходила вовсе» от «ушла половина»: первое безобидно, второе означает,
// что половины разошлись по тяге.
static bool PREV_PAIR_WHOLE = true;

FcDualMotorConfig fc_dual_motor_default_config(void) {
    FcDualMotorConfig c;
    memset(&c, 0, sizeof c);
    // Инверсий нет. Это не умолчание «на всякий случай», а физически
    // доказанный на v0.8B факт: положительная команда обеим половинам даёт
    // движение вперёд (docs/second_motor_bringup.md §7).
    c.invert[FC_DUAL_A] = false;
    c.invert[FC_DUAL_B] = false;
    // Временный предел первого прокрута, стоящий сейчас в обеих половинах.
    // Рабочее значение будет другим и появится отдельным решением.
    c.current_limit_a = 5.0f;
    return c;
}

void fc_dual_motor_init(const FcDualMotorConfig *cfg) {
    CFG = cfg ? *cfg : fc_dual_motor_default_config();
    memset(&ST, 0, sizeof ST);
    LAST_SKEW_US = 0;
    PREV_PAIR_WHOLE = true;
}

void fc_dual_motor_arm(void) {
    ST.armed = true;
}

bool fc_dual_motor_try_arm(const FcDualArmInputs *in, uint32_t *deny_mask) {
    uint32_t m = 0;
    if (in == NULL) {
        m = 0xFFFFFFFFu;
    } else {
        if (!in->supervisor_healthy) m |= FC_DUAL_ARM_DENY_SUPERVISOR;
        if (!in->imu_healthy) m |= FC_DUAL_ARM_DENY_IMU;
        if (!in->node_healthy[FC_DUAL_A]) m |= FC_DUAL_ARM_DENY_NODE_A;
        if (!in->node_healthy[FC_DUAL_B]) m |= FC_DUAL_ARM_DENY_NODE_B;
        if (!in->can_healthy) m |= FC_DUAL_ARM_DENY_CAN;
        if (!in->battery_model_valid) m |= FC_DUAL_ARM_DENY_BATTERY_MODEL;
        if (!in->motor_model_valid) m |= FC_DUAL_ARM_DENY_MOTOR_MODEL;
        if (!in->boot_complete) m |= FC_DUAL_ARM_DENY_BOOT;
        if (!in->realtime_qualified) m |= FC_DUAL_ARM_DENY_REALTIME;
    }
    // Защёлка проверяется отдельно от входов: она живёт в самом координаторе
    // и снимается только предусмотренной процедурой.
    if (ST.latched || !PREV_PAIR_WHOLE) {
        m |= FC_DUAL_ARM_DENY_LATCHED;
    }
    if (deny_mask) {
        *deny_mask = m;
    }
    if (m != 0) {
        return false;
    }
    ST.armed = true;
    return true;
}

const char *fc_dual_motor_arm_deny_name(FcDualArmDeny r) {
    switch (r) {
    case FC_DUAL_ARM_DENY_SUPERVISOR: return "supervisor нездоров";
    case FC_DUAL_ARM_DENY_IMU: return "IMU нездоров";
    case FC_DUAL_ARM_DENY_NODE_A: return "половина A нездорова";
    case FC_DUAL_ARM_DENY_NODE_B: return "половина B нездорова";
    case FC_DUAL_ARM_DENY_LATCHED: return "залипший отказ не снят";
    case FC_DUAL_ARM_DENY_CAN: return "шина CAN нездорова";
    case FC_DUAL_ARM_DENY_BATTERY_MODEL: return "модель батареи не проверена";
    case FC_DUAL_ARM_DENY_MOTOR_MODEL: return "параметры мотора не подтверждены";
    case FC_DUAL_ARM_DENY_BOOT: return "загрузка не завершена";
    case FC_DUAL_ARM_DENY_REALTIME: return "realtime-квалификация не пройдена";
    default: return "?";
    }
}

void fc_dual_motor_disarm(void) {
    ST.armed = false;
}

void fc_dual_motor_clear_latch(void) {
    ST.latched = false;
    PREV_PAIR_WHOLE = true;
    // Снятие защёлки НЕ разрешает тягу: разрешение запрашивается отдельно.
    // Иначе одна команда оператора снимала бы сразу два барьера.
    ST.armed = false;
}

static void deny(uint32_t *mask, FcDualDenyReason r) {
    *mask |= (uint32_t) r;
}

static bool stale(uint64_t now, uint64_t last, uint32_t limit) {
    // Отметка из будущего означает рассогласование часов, а не свежесть.
    if (last > now) {
        return true;
    }
    return (now - last) > (uint64_t) limit;
}

FcDualMotorPlan fc_dual_motor_plan(const FcDualMotorInputs *in) {
    FcDualMotorPlan p;
    memset(&p, 0, sizeof p);
    p.order[0] = (uint8_t) FC_DUAL_A;
    p.order[1] = (uint8_t) FC_DUAL_B;

    ++ST.evaluations;

    uint32_t mask = 0;

    if (!ST.armed) {
        deny(&mask, FC_DUAL_DENY_NOT_ARMED);
    }
    if (ST.latched) {
        deny(&mask, FC_DUAL_DENY_LATCHED);
    }
    if (!PREV_PAIR_WHOLE) {
        deny(&mask, FC_DUAL_DENY_PARTIAL_SEND);
    }
    if (!in->supervisor_allows) {
        deny(&mask, FC_DUAL_DENY_SUPERVISOR);
    }
    if (stale(in->now_us, in->last_imu_us, FC_DUAL_IMU_FRESH_US)) {
        deny(&mask, FC_DUAL_DENY_IMU_STALE);
    }
    if (stale(in->now_us, in->last_command_us, FC_DUAL_COMMAND_FRESH_US)) {
        deny(&mask, FC_DUAL_DENY_COMMAND_STALE);
    }
    if (stale(in->now_us, in->last_feedback_us[FC_DUAL_A], FC_DUAL_NODE_FRESH_US)) {
        deny(&mask, FC_DUAL_DENY_NODE_A_STALE);
    }
    if (stale(in->now_us, in->last_feedback_us[FC_DUAL_B], FC_DUAL_NODE_FRESH_US)) {
        deny(&mask, FC_DUAL_DENY_NODE_B_STALE);
    }
    if (in->esc_fault_code[FC_DUAL_A] != 0) {
        deny(&mask, FC_DUAL_DENY_NODE_A_FAULT);
    }
    if (in->esc_fault_code[FC_DUAL_B] != 0) {
        deny(&mask, FC_DUAL_DENY_NODE_B_FAULT);
    }
    if (in->can_bus_off) {
        deny(&mask, FC_DUAL_DENY_BUS_OFF);
    }
    if (in->can_degraded) {
        deny(&mask, FC_DUAL_DENY_CAN_DEGRADED);
    }

    float v = in->requested_current_a;
    if (isnan(v) || isinf(v)) {
        deny(&mask, FC_DUAL_DENY_VALUE_INVALID);
    } else if (fabsf(v) > CFG.current_limit_a) {
        // Именно запрет, а не насыщение. Насыщение превратило бы ошибку
        // расчёта в тихо принятую команду предельной величины — худший из
        // возможных исходов для доски под человеком.
        deny(&mask, FC_DUAL_DENY_VALUE_RANGE);
    }

    p.deny_mask = mask;
    ST.last_deny_mask = mask;

    if (mask != 0) {
        ++ST.permits_denied;
        for (int b = 0; b < FC_DUAL_DENY_REASON_COUNT; ++b) {
            if (mask & (1u << b)) {
                ++ST.deny_by_reason[b];
            }
        }
        // Отказ никогда не превращается в команду. Значения остаются нулями,
        // а send — false: прошлое значение не повторяется (ТЗ v0.8C §3 I10).
        p.send = false;
        return p;
    }

    ++ST.permits_granted;
    p.send = true;
    p.sequence = ++ST.sequence;
    for (unsigned i = 0; i < FC_DUAL_HALVES; ++i) {
        p.current_a[i] = CFG.invert[i] ? -v : v;
    }
    return p;
}

void fc_dual_motor_report_send(bool sent_a, bool sent_b, uint64_t now_us) {
    (void) now_us;
    if (sent_a == sent_b) {
        PREV_PAIR_WHOLE = true;
        return;
    }
    // Половины разошлись: одна получила новую тягу, вторая осталась на
    // прошлой. Продолжать нормальную работу нельзя, и восстановление
    // «само собой» запрещено — только через clear_latch (ТЗ §3 I4, §4 E).
    ++ST.partial_sends;
    ++ST.latched_faults;
    ST.latched = true;
    PREV_PAIR_WHOLE = false;
}

void fc_dual_motor_note_skew(uint32_t skew_us) {
    LAST_SKEW_US = skew_us;
}

uint32_t fc_dual_motor_last_skew_us(void) {
    return LAST_SKEW_US;
}

uint32_t fc_dual_motor_worst_asymmetry_us(void) {
    // Худший случай — «тихий» отказ: половина перестала исполнять команды, но
    // кадры уходят, и заметить это можно только по устареванию обратной
    // связи. Тогда одна половина выбегает по своему таймауту, а вторая ещё
    // получает команды до момента обнаружения.
    //
    //   обнаружение          FC_DUAL_NODE_FRESH_US
    // + таймаут ESC          50 000 мкс
    // + округление до тика   10 000 мкс
    return FC_DUAL_NODE_FRESH_US + 50000u + 10000u;
}

FcDualMotorStats fc_dual_motor_stats(void) {
    return ST;
}

const char *fc_dual_motor_reason_name(FcDualDenyReason r) {
    switch (r) {
    case FC_DUAL_DENY_NOT_ARMED: return "разрешение не запрашивалось";
    case FC_DUAL_DENY_SUPERVISOR: return "supervisor не разрешает";
    case FC_DUAL_DENY_LATCHED: return "залипший отказ";
    case FC_DUAL_DENY_IMU_STALE: return "семпл IMU устарел";
    case FC_DUAL_DENY_COMMAND_STALE: return "команда контура устарела";
    case FC_DUAL_DENY_NODE_A_STALE: return "половина A молчит";
    case FC_DUAL_DENY_NODE_B_STALE: return "половина B молчит";
    case FC_DUAL_DENY_NODE_A_FAULT: return "fault половины A";
    case FC_DUAL_DENY_NODE_B_FAULT: return "fault половины B";
    case FC_DUAL_DENY_CAN_DEGRADED: return "шина CAN деградировала";
    case FC_DUAL_DENY_BUS_OFF: return "шина CAN в BUS_OFF";
    case FC_DUAL_DENY_VALUE_INVALID: return "значение NaN или Inf";
    case FC_DUAL_DENY_VALUE_RANGE: return "значение вне предела тока";
    case FC_DUAL_DENY_PARTIAL_SEND: return "прошлая пара ушла не целиком";
    default: return "?";
    }
}

// --------------------------------------------------------------- исполнение
//
// Вынесено сюда, а не в отдельный файл: функция коротка и её единственная
// содержательная часть — порядок и учёт разбега, то есть та же логика
// координатора.

#include "fc_motor_transport.h"

FcDualSendResult fc_dual_motor_execute(const FcDualMotorPlan *plan, const FcDualTransport *t) {
    FcDualSendResult r;
    memset(&r, 0, sizeof r);

    if (plan == NULL || t == NULL || t->send_current == NULL || !plan->send) {
        // Запрет означает тишину, а не команду нуля. Нулевая команда — это
        // тоже команда: она продлевает сторожевой таймер ESC и оставляет
        // половину в состоянии «контроллер жив». При отказе нужно обратное.
        r.pair_whole = true;
        return r;
    }

    uint64_t t0 = t->now_us ? t->now_us(t->ctx) : 0;
    r.sent[FC_DUAL_A] = t->send_current((uint8_t) FC_DUAL_A, plan->current_a[FC_DUAL_A], t->ctx);
    uint64_t t1 = t->now_us ? t->now_us(t->ctx) : 0;
    r.sent[FC_DUAL_B] = t->send_current((uint8_t) FC_DUAL_B, plan->current_a[FC_DUAL_B], t->ctx);

    r.skew_us = (uint32_t) (t1 - t0);
    r.pair_whole = r.sent[FC_DUAL_A] == r.sent[FC_DUAL_B];

    fc_dual_motor_note_skew(r.skew_us);
    fc_dual_motor_report_send(r.sent[FC_DUAL_A], r.sent[FC_DUAL_B], t1);
    return r;
}
