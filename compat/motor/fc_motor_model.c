#include "fc_motor_model.h"

#include <string.h>

// Половина A — это ID 118, к ней на v0.8A подключён физический мотор, и её
// параметры измерены штатной детекцией FOC под наблюдением. Половина B — ID
// 100, мотор к ней не подключён, и её значения по-прежнему только прочитаны.
//
// Что выяснилось на v0.8A про происхождение конфигурации. Сопротивление и
// таблица датчиков Hall оказались настоящими выводами детекции ЭТОГО мотора:
// R совпал с записанным на 0.26 %, таблица Hall — в пределах одной единицы из
// двухсот. А число полюсов и потокосцепление были введены руками и неверны —
// в 2.14 и 5.1 раза. Подозрение, записанное на v0.7D («ровно 0.1 при
// некруглых R и L — похоже на ручной ввод»), подтвердилось.
static const FcMotorParam PARAMS[] = {
    {
        .name = "foc_motor_r",
        .units = "Ом",
        .value_a = 0.3188f,
        .value_b = 0.3162f,
        .kind = FC_PARAM_MEASURED_BY_DETECTION,
        .trust_a = FC_PARAM_VERIFIED,
        .trust_b = FC_PARAM_READ_FROM_ESC,
        .needed_by_refloat = false,
        .safe_before_detection = false,
        .must_refresh_after_detection = true,
        .note = "измерено детекцией на v0.8A: 318.8 мОм против записанных 319.6 — "
                "расхождение 0.26 %, то есть записанное значение тоже мерили "
                "на этом моторе",
        .verify_criterion = "новая детекция FOC на физически подключённом ИМЕННО ЭТОМ моторе",
    },
    {
        .name = "foc_motor_l",
        .units = "мкГн",
        .value_a = 874.0f,
        .value_b = 866.0f,
        .kind = FC_PARAM_MEASURED_BY_DETECTION,
        .trust_a = FC_PARAM_VERIFIED,
        .trust_b = FC_PARAM_READ_FROM_ESC,
        .needed_by_refloat = false,
        .safe_before_detection = false,
        .must_refresh_after_detection = true,
        .note = "измерено 873.7 мкГн против записанных 730.4. Расхождение 20 % "
                "объясняется положением ротора: Lq-Ld составляет 333 мкГн, то есть "
                "38 % от самой индуктивности",
        .verify_criterion = "новая детекция FOC на физически подключённом ИМЕННО ЭТОМ моторе",
    },
    {
        .name = "foc_motor_flux_linkage",
        .units = "Вб",
        .value_a = 0.0196f,
        .value_b = 0.1f,
        .kind = FC_PARAM_MEASURED_BY_DETECTION,
        .trust_a = FC_PARAM_VERIFIED,
        .trust_b = FC_PARAM_UNVERIFIED,
        .needed_by_refloat = true,
        .safe_before_detection = false,
        .must_refresh_after_detection = true,
        .note = "измерено 19.6 мВб против записанных 100. Записанное давало "
                "максимальную скорость 151 об-мин = 8 км-ч, что для моноколеса "
                "невозможно; измеренное даёт 770 об-мин = 40 км-ч. У половины B "
                "по-прежнему стоит 0.1 и почти наверняка тоже неверно",
        .verify_criterion = "новая детекция FOC на физически подключённом ИМЕННО ЭТОМ моторе",
    },
    {
        .name = "si_motor_poles",
        .units = "полюсов",
        .value_a = 30.0f,
        .value_b = 14.0f,
        .kind = FC_PARAM_CONFIGURED,
        .trust_a = FC_PARAM_VERIFIED,
        .trust_b = FC_PARAM_UNVERIFIED,
        .needed_by_refloat = true,
        .safe_before_detection = false,
        .must_refresh_after_detection = false,
        .note = "ИЗМЕРЕНО на v0.8A вращением от руки: 899 шагов тахометра на 10 "
                "оборотов = 89.9 на оборот, при шести шагах на электрический "
                "оборот это 15 пар, то есть 30 полюсов. Записанные 14 неверны. "
                "Детекция FOC полюса не определяет вовсе",
        .verify_criterion = "ПОДТВЕРЖДЕНО: тахометр считает 6 шагов на электрический оборот "
                "(mcpwm_foc.c:3765), поэтому полюса = dtach / (3 x оборотов)",
    },
    {
        .name = "l_current_max",
        .units = "А",
        .value_a = 5.0f,
        .value_b = 25.0f,
        .kind = FC_PARAM_SAFETY_POLICY,
        .trust_a = FC_PARAM_READ_FROM_ESC,
        .trust_b = FC_PARAM_READ_FROM_ESC,
        .needed_by_refloat = true,
        .safe_before_detection = true,
        .must_refresh_after_detection = false,
        .note = "на половине A выставлен ВРЕМЕННЫЙ предел первого прокрута 5 А; "
                "рабочее значение 25 А осталось на половине B. Предел проверен в "
                "работе: на старте при заторможенном роторе ток упирался в него",
        .verify_criterion = "предел выбирается человеком, детекцией не подтверждается; считается подтверждённым после проверки на вывешенном колесе",
    },
    {
        .name = "l_current_min",
        .units = "А",
        .value_a = -3.0f,
        .value_b = -5.0f,
        .kind = FC_PARAM_SAFETY_POLICY,
        .trust_a = FC_PARAM_READ_FROM_ESC,
        .trust_b = FC_PARAM_READ_FROM_ESC,
        .needed_by_refloat = true,
        .safe_before_detection = true,
        .must_refresh_after_detection = false,
        .note = "временный предел первого прокрута на половине A. Асимметрия "
                "разгон-торможение требует решения до первой езды",
        .verify_criterion = "то же, плюс проверка, что торможение не даёт недопустимого regen",
    },
    {
        .name = "l_in_current_max",
        .units = "А",
        .value_a = 5.0f,
        .value_b = 15.0f,
        .kind = FC_PARAM_SAFETY_POLICY,
        .trust_a = FC_PARAM_READ_FROM_ESC,
        .trust_b = FC_PARAM_READ_FROM_ESC,
        .needed_by_refloat = false,
        .safe_before_detection = true,
        .must_refresh_after_detection = false,
        .note = "временный предел первого прокрута на половине A",
        .verify_criterion = "то же, плюс измерение тока батареи под нагрузкой",
    },
    {
        .name = "si_battery_cells",
        .units = "ячеек",
        .value_a = 10.0f,
        .value_b = 10.0f,
        .kind = FC_PARAM_CONFIGURED,
        .trust_a = FC_PARAM_VERIFIED,
        .trust_b = FC_PARAM_VERIFIED,
        .needed_by_refloat = true,
        .safe_before_detection = false,
        .must_refresh_after_detection = false,
        .note = "10S подтверждено владельцем, измерением мультиметром (40.8 В = "
                "4.08 В на ячейку) и согласием со всеми отсечками ESC",
        .verify_criterion = "измерение напряжения батареи мультиметром и сверка с числом ячеек",
    },
    {
        .name = "si_wheel_diameter",
        .units = "м",
        .value_a = 0.083f,
        .value_b = 0.083f,
        .kind = FC_PARAM_CONFIGURED,
        .trust_a = FC_PARAM_UNVERIFIED,
        .trust_b = FC_PARAM_UNVERIFIED,
        .needed_by_refloat = false,
        .safe_before_detection = false,
        .must_refresh_after_detection = false,
        .note = "83 мм для моноколеса неправдоподобно — вместе с gear_ratio 3.0 "
                "похоже на остатки конфигурации другого аппарата",
        .verify_criterion = "физическое измерение диаметра колеса",
    },
    {
        .name = "si_gear_ratio",
        .units = "-",
        .value_a = 3.0f,
        .value_b = 3.0f,
        .kind = FC_PARAM_CONFIGURED,
        .trust_a = FC_PARAM_UNVERIFIED,
        .trust_b = FC_PARAM_UNVERIFIED,
        .needed_by_refloat = false,
        .safe_before_detection = false,
        .must_refresh_after_detection = false,
        .note = "у мотор-колеса прямой привод, то есть 1.0",
        .verify_criterion = "подтверждение прямого привода мотор-колеса, то есть 1.0",
    },
};

#define PARAM_COUNT (sizeof(PARAMS) / sizeof(PARAMS[0]))

const FcMotorParam *fc_motor_model_all(size_t *n) {
    if (n) {
        *n = PARAM_COUNT;
    }
    return PARAMS;
}

const FcMotorParam *fc_motor_model_find(const char *name) {
    for (size_t i = 0; i < PARAM_COUNT; ++i) {
        if (strcmp(PARAMS[i].name, name) == 0) {
            return &PARAMS[i];
        }
    }
    return NULL;
}

bool fc_motor_model_ready_for_output(const char **why) {
    const char *dummy = NULL;
    if (!why) {
        why = &dummy;
    }
    for (size_t i = 0; i < PARAM_COUNT; ++i) {
        const FcMotorParam *p = &PARAMS[i];
        if (!p->needed_by_refloat && p->kind != FC_PARAM_SAFETY_POLICY) {
            continue;
        }
        if (p->trust_a != FC_PARAM_VERIFIED || p->trust_b != FC_PARAM_VERIFIED) {
            *why = p->name;
            return false;
        }
    }
    *why = NULL;
    return true;
}

size_t fc_motor_model_unverified_count(void) {
    size_t n = 0;
    for (size_t i = 0; i < PARAM_COUNT; ++i) {
        if (PARAMS[i].trust_a != FC_PARAM_VERIFIED ||
            PARAMS[i].trust_b != FC_PARAM_VERIFIED) {
            ++n;
        }
    }
    return n;
}

const char *fc_param_kind_name(FcParamKind k) {
    switch (k) {
    case FC_PARAM_MEASURED_BY_DETECTION:
        return "измеряется детекцией";
    case FC_PARAM_CONFIGURED:
        return "введено человеком";
    case FC_PARAM_SAFETY_POLICY:
        return "предел безопасности";
    default:
        return "?";
    }
}

const char *fc_param_trust_name(FcParamTrust t) {
    switch (t) {
    case FC_PARAM_UNVERIFIED:
        return "НЕ ПОДТВЕРЖДЕНО";
    case FC_PARAM_READ_FROM_ESC:
        return "прочитано из ESC";
    case FC_PARAM_VERIFIED:
        return "подтверждено на моторе";
    default:
        return "?";
    }
}
