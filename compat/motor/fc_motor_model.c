#include "fc_motor_model.h"

#include <string.h>

// Значения прочитаны через CAN на v0.7B и побайтово сверены с независимым
// чтением по USB: FW_VERSION и APPCONF совпали целиком, в MCCONF разошлись
// только смещения датчиков, которые ESC меряет при каждом включении.
//
// Совпадение двух путей доказывает, что канал не врёт. Оно НЕ доказывает,
// что сами числа относятся к тому мотору, который будет подключён.
static const FcMotorParam PARAMS[] = {
    {
        .name = "foc_motor_r",
        .units = "Ом",
        .value_a = 0.3196f,
        .value_b = 0.3162f,
        .kind = FC_PARAM_MEASURED_BY_DETECTION,
        .trust = FC_PARAM_READ_FROM_ESC,
        .needed_by_refloat = false,
        .safe_before_detection = false,
        .must_refresh_after_detection = true,
        .note = "половины различаются — признак того, что детекция когда-то "
                "делалась раздельно и на реальном железе",
    },
    {
        .name = "foc_motor_l",
        .units = "мкГн",
        .value_a = 730.0f,
        .value_b = 866.0f,
        .kind = FC_PARAM_MEASURED_BY_DETECTION,
        .trust = FC_PARAM_READ_FROM_ESC,
        .needed_by_refloat = false,
        .safe_before_detection = false,
        .must_refresh_after_detection = true,
        .note = "разброс между половинами 19 % — много для одинаковых моторов",
    },
    {
        .name = "foc_motor_flux_linkage",
        .units = "Вб",
        .value_a = 0.1f,
        .value_b = 0.1f,
        .kind = FC_PARAM_MEASURED_BY_DETECTION,
        .trust = FC_PARAM_READ_FROM_ESC,
        .needed_by_refloat = true,
        .safe_before_detection = false,
        .must_refresh_after_detection = true,
        .note = "ровно 0.1 на обеих половинах при разных R и L — round number, "
                "какие детекция не выдаёт; похоже на ручной ввод",
    },
    {
        .name = "si_motor_poles",
        .units = "полюсов",
        .value_a = 14.0f,
        .value_b = 14.0f,
        .kind = FC_PARAM_CONFIGURED,
        .trust = FC_PARAM_UNVERIFIED,
        .needed_by_refloat = true,
        .safe_before_detection = false,
        .must_refresh_after_detection = false,
        .note = "детекция FOC число полюсов НЕ определяет; проверяется только "
                "вращением с известной механической скоростью",
    },
    {
        .name = "l_current_max",
        .units = "А",
        .value_a = 25.0f,
        .value_b = 25.0f,
        .kind = FC_PARAM_SAFETY_POLICY,
        .trust = FC_PARAM_READ_FROM_ESC,
        .needed_by_refloat = true,
        .safe_before_detection = true,
        .must_refresh_after_detection = false,
        .note = "предел, а не свойство мотора: детекция его менять не должна",
    },
    {
        .name = "l_current_min",
        .units = "А",
        .value_a = -5.0f,
        .value_b = -5.0f,
        .kind = FC_PARAM_SAFETY_POLICY,
        .trust = FC_PARAM_READ_FROM_ESC,
        .needed_by_refloat = true,
        .safe_before_detection = true,
        .must_refresh_after_detection = false,
        .note = "асимметрия 25/-5 означает слабое торможение: для баланса это "
                "решение, которое надо принять осознанно",
    },
    {
        .name = "l_in_current_max",
        .units = "А",
        .value_a = 15.0f,
        .value_b = 15.0f,
        .kind = FC_PARAM_SAFETY_POLICY,
        .trust = FC_PARAM_READ_FROM_ESC,
        .needed_by_refloat = false,
        .safe_before_detection = true,
        .must_refresh_after_detection = false,
        .note = "входной ток батареи, действует поверх моторного",
    },
    {
        .name = "si_battery_cells",
        .units = "ячеек",
        .value_a = 3.0f,
        .value_b = 3.0f,
        .kind = FC_PARAM_CONFIGURED,
        .trust = FC_PARAM_UNVERIFIED,
        .needed_by_refloat = true,
        .safe_before_detection = false,
        .must_refresh_after_detection = false,
        .note = "ПРОТИВОРЕЧИТ отсечкам: 34.0/31.0 В и 41.5/42.0 В описывают 10S, "
                "а не 3S. Refloat умножает пороги HV/LV на это число",
    },
    {
        .name = "si_wheel_diameter",
        .units = "м",
        .value_a = 0.083f,
        .value_b = 0.083f,
        .kind = FC_PARAM_CONFIGURED,
        .trust = FC_PARAM_UNVERIFIED,
        .needed_by_refloat = false,
        .safe_before_detection = false,
        .must_refresh_after_detection = false,
        .note = "83 мм для моноколеса неправдоподобно — вместе с gear_ratio 3.0 "
                "похоже на остатки конфигурации другого аппарата",
    },
    {
        .name = "si_gear_ratio",
        .units = "-",
        .value_a = 3.0f,
        .value_b = 3.0f,
        .kind = FC_PARAM_CONFIGURED,
        .trust = FC_PARAM_UNVERIFIED,
        .needed_by_refloat = false,
        .safe_before_detection = false,
        .must_refresh_after_detection = false,
        .note = "у мотор-колеса прямой привод, то есть 1.0",
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
        if (p->trust != FC_PARAM_VERIFIED) {
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
        if (PARAMS[i].trust != FC_PARAM_VERIFIED) {
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
