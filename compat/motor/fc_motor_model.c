#include "fc_motor_model.h"

#include <string.h>

// Половина A — ID 118, мотор A, квалифицирован на v0.8A. Половина B — ID 100,
// мотор B, квалифицирован на v0.8B. Оба мотора сидят на одной оси жёстко.
//
// Про происхождение конфигурации. У обеих половин она оказалась СМЕСЬЮ:
// таблица датчиков Hall — настоящий вывод детекции этого мотора (совпала с
// измеренной в пределах единиц из двухсот), а число полюсов и
// потокосцепление введены руками и неверны — у A в 2.14 и 5.1 раза, у B в
// 2.14 и 5.0 раза. Отсюда правило: непроверенный параметр не становится
// верным оттого, что второй мотор такой же.
//
// Про R и L. На v0.8B двенадцать прогонов детекции показали, что при
// l_current_max = 5 А ток детекции равен 1.67 А, падение на обмотке 0.5 В
// против шины 41 В, и искажение мёртвого времени ключей сравнимо с самим
// измеряемым падением. Разброс ±20 % у ОБОИХ моторов. Поэтому измеренные R и
// L в половину B не записывались: наше измерение менее точно, чем то, что
// уже стоит. Подробности — docs/second_motor_bringup.md §4.
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
        .note = "половина A: пять прогонов на v0.8B дали среднее 318.4 мОм при sd 34, "
                "что сходится с записанными 319.6 и с измерением v0.8A. Половина B: "
                "восемь прогонов дали среднее 265 при sd 65 — записанные 316.2 этим "
                "не подтверждены и не опровергнуты, метод на 5 А слишком груб",
        .verify_criterion = "детекция FOC при токе, дающем падение много больше искажения мёртвого времени; на l_current_max = 5 А метод даёт +-20 %",
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
        .note = "зависит от положения ротора: Lq-Ld составляет 38-42 % от самой "
                "индуктивности, поэтому разброс между прогонами и между половинами "
                "(713 против 874 мкГн) не превышает разброса у ОДНОГО мотора",
        .verify_criterion = "детекция FOC при достаточном токе, с фиксацией положения ротора",
    },
    {
        .name = "foc_motor_flux_linkage",
        .units = "Вб",
        .value_a = 0.0196f,
        .value_b = 0.019961f,
        .kind = FC_PARAM_MEASURED_BY_DETECTION,
        .trust_a = FC_PARAM_VERIFIED,
        .trust_b = FC_PARAM_VERIFIED,
        .needed_by_refloat = true,
        .safe_before_detection = false,
        .must_refresh_after_detection = true,
        .note = "A: 19.600 мВб, B: 19.961 мВб — разница 1.84 %. У обеих было записано "
                "0.1 Вб, то есть впятеро больше: это давало максимум 151 об-мин = "
                "8 км-ч, для моноколеса невозможно. Единственный параметр мотора, "
                "который на наших пределах меряется надёжно",
        .verify_criterion = "детекция FOC с подтверждением вращения по тахометру второй половины и проверкой правдоподобия максимальных оборотов",
    },
    {
        .name = "si_motor_poles",
        .units = "полюсов",
        .value_a = 30.0f,
        .value_b = 30.0f,
        .kind = FC_PARAM_CONFIGURED,
        .trust_a = FC_PARAM_VERIFIED,
        .trust_b = FC_PARAM_VERIFIED,
        .needed_by_refloat = true,
        .safe_before_detection = false,
        .must_refresh_after_detection = false,
        .note = "ИЗМЕРЕНО вращением от руки на ОБЕИХ половинах: 900 шагов тахометра "
                "на 10 оборотов, при шести шагах на электрический оборот это 15 пар, "
                "то есть 30 полюсов. Записанные 14 у половины B дали бы 420 шагов. "
                "Детекция FOC полюса не определяет вовсе. На v0.8B измерение "
                "проверило само себя: у половин стояли РАЗНЫЕ si_motor_poles, а "
                "счётчики совпали шаг в шаг, то есть тахометр от этого поля не зависит",
        .verify_criterion = "ПОДТВЕРЖДЕНО: тахометр считает 6 шагов на электрический оборот "
                "(mcpwm_foc.c:3765), поэтому полюса = dtach / (3 x оборотов)",
    },
    {
        .name = "l_current_max",
        .units = "А",
        .value_a = 5.0f,
        .value_b = 5.0f,
        .kind = FC_PARAM_SAFETY_POLICY,
        .trust_a = FC_PARAM_READ_FROM_ESC,
        .trust_b = FC_PARAM_READ_FROM_ESC,
        .needed_by_refloat = true,
        .safe_before_detection = true,
        .must_refresh_after_detection = false,
        .note = "ВРЕМЕННЫЙ предел первого прокрута, на v0.8B выставлен на ОБЕИХ "
                "половинах (было 25 А на B). Предел проверен в работе: на старте "
                "при заторможенном роторе ток упирался в него. Для езды он другой",
        .verify_criterion = "предел выбирается человеком, детекцией не подтверждается; считается подтверждённым после проверки на вывешенном колесе",
    },
    {
        .name = "l_current_min",
        .units = "А",
        .value_a = -3.0f,
        .value_b = -3.0f,
        .kind = FC_PARAM_SAFETY_POLICY,
        .trust_a = FC_PARAM_READ_FROM_ESC,
        .trust_b = FC_PARAM_READ_FROM_ESC,
        .needed_by_refloat = true,
        .safe_before_detection = true,
        .must_refresh_after_detection = false,
        .note = "временный предел первого прокрута, на v0.8B выставлен на обеих "
                "половинах. Асимметрия разгон-торможение требует решения до "
                "первой езды",
        .verify_criterion = "то же, плюс проверка, что торможение не даёт недопустимого regen",
    },
    {
        .name = "l_in_current_max",
        .units = "А",
        .value_a = 5.0f,
        .value_b = 5.0f,
        .kind = FC_PARAM_SAFETY_POLICY,
        .trust_a = FC_PARAM_READ_FROM_ESC,
        .trust_b = FC_PARAM_READ_FROM_ESC,
        .needed_by_refloat = false,
        .safe_before_detection = true,
        .must_refresh_after_detection = false,
        .note = "временный предел первого прокрута, на v0.8B выставлен на обеих половинах",
        .verify_criterion = "то же, плюс измерение тока батареи под нагрузкой",
    },
    {
        .name = "l_in_current_min",
        .units = "А",
        .value_a = -1.0f,
        .value_b = -1.0f,
        .kind = FC_PARAM_SAFETY_POLICY,
        .trust_a = FC_PARAM_READ_FROM_ESC,
        .trust_b = FC_PARAM_READ_FROM_ESC,
        .needed_by_refloat = false,
        .safe_before_detection = true,
        .must_refresh_after_detection = false,
        .note = "ноль означает НЕ мягкое торможение, а отсутствие тормозного "
                "момента вовсе: ограничение накладывается на iq и делится на mod_q "
                "(mcpwm_foc.c:3564-3567). -1 А это C/6 для сборки 6 А*ч, и отсечка "
                "regen 41.5-42.0 В обрезает его до нуля на полной батарее. Для езды "
                "значение будет другим",
        .verify_criterion = "измерение тока в батарею при торможении на вывешенном колесе",
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
                "4.08 В на ячейку, повторно на v0.8B) и согласием со всеми "
                "отсечками ESC",
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
