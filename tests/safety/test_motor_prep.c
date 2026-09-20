// Host-тесты подготовки к моторному этапу (ТЗ v0.7C §17).
//
// Проверяется то, что можно проверить без мотора и что стоит проверить
// именно сейчас: политика сторожевого таймера, модель батареи, степень
// доверия к параметрам мотора и контракт разрешения выхода.
//
// Ни один тест не трогает выход на мотор — его не существует.

#include "../../compat/log/fc_log.h"
#include "../../compat/motor/fc_motor_model.h"
#include "../../compat/safety/fc_battery_model.h"
#include "../../compat/safety/fc_build_profile.h"
#include "../../compat/safety/fc_command_watchdog.h"
#include "../../compat/safety/fc_motor_gate.h"
#include "../../compat/safety/fc_motor_permit.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void fc_test_check(bool ok, const char *what);
void fc_test_note(const char *fmt, ...);
#define check fc_test_check
#define note fc_test_note

// ------------------------------------------------------- сторожевой таймер

// Политика, предлагаемая на v0.7C. Числа обоснованы в
// docs/vesc_watchdog_policy.md и здесь именно проверяются.
static FcCommandWatchdogPolicy proposed(void) {
    FcCommandWatchdogPolicy p = {
        .command_period_us = 2000,      // 500 Гц, темп контура
        .tolerated_misses = 4,          // 4 подряд = 10 мс
        .esc_timeout_us = 50000,        // timeout_msec = 50
        .supervisor_timeout_us = 20000, // FloatCore прекращает слать раньше
        .esc_tick_us = FC_ESC_TIMEOUT_TICK_US,
    };
    return p;
}

static void test_watchdog_policy(void) {
    printf("\n\033[1mПолитика сторожевого таймера команд\033[0m\n");
    const char *why = NULL;

    FcCommandWatchdogPolicy p = proposed();
    check(fc_command_watchdog_policy_valid(&p, &why), "предлагаемая политика непротиворечива");
    note("500 Гц команд, допуск 4 пропуска, свой таймаут 20 мс, ESC 50 мс");
    note("худшее удержание последней команды при отказе FloatCore: %u мкс",
         fc_command_watchdog_worst_case_us(&p));

    // Ноль читается как «строже некуда», а означает «проверки нет».
    p = proposed();
    p.esc_timeout_us = 0;
    check(!fc_command_watchdog_policy_valid(&p, &why), "timeout_msec = 0 отвергается");
    note("причина: %s", why);

    // Текущее значение на стенде.
    p = proposed();
    p.esc_timeout_us = 1000000;
    check(!fc_command_watchdog_policy_valid(&p, &why),
          "нынешние 1000 мс отвергаются как слишком долгие");
    note("причина: %s", why);

    // Порядок барьеров.
    p = proposed();
    p.supervisor_timeout_us = 60000;
    check(!fc_command_watchdog_policy_valid(&p, &why),
          "свой таймаут позже ESC отвергается: барьеры бесполезны");
    note("причина: %s", why);

    // Дискретность потока Timeout на ESC.
    p = proposed();
    p.esc_timeout_us = 15000;
    check(!fc_command_watchdog_policy_valid(&p, &why),
          "таймаут меньше двух тиков потока ESC отвергается");

    // Допуск, который не успевает реализоваться.
    p = proposed();
    p.tolerated_misses = 50;
    check(!fc_command_watchdog_policy_valid(&p, &why),
          "допуск больше собственного таймаута отвергается");
}

static void test_command_stream(void) {
    printf("\n\033[1mСостояние потока команд\033[0m\n");
    FcCommandWatchdogPolicy p = proposed();
    FcCommandStream s;
    fc_command_stream_init(&s, &p);

    check(fc_command_stream_poll(&s, 0) == FC_CMD_STREAM_IDLE,
          "до первой команды состояние IDLE, а не OK");

    uint64_t t = 1000000;
    fc_command_stream_sent(&s, t);
    check(fc_command_stream_poll(&s, t + 2000) == FC_CMD_STREAM_OK, "команды в срок — OK");
    check(fc_command_stream_poll(&s, t + 10000) == FC_CMD_STREAM_OK,
          "четыре пропуска — ещё в допуске");
    check(fc_command_stream_poll(&s, t + 12000) == FC_CMD_STREAM_LATE,
          "за пределом допуска — LATE");
    check(fc_command_stream_poll(&s, t + 20000) == FC_CMD_STREAM_LOST,
          "по своему таймауту — LOST, раньше чем сработает ESC");
    check(fc_command_stream_poll(&s, t + 49000) == FC_CMD_STREAM_LOST,
          "LOST держится, пока команд нет");
    note("ESC снял бы тягу только на %u мкс — на %u мкс позже",
         fc_command_watchdog_worst_case_us(&p),
         fc_command_watchdog_worst_case_us(&p) - p.supervisor_timeout_us);

    fc_command_stream_sent(&s, t + 60000);
    check(fc_command_stream_poll(&s, t + 60000) == FC_CMD_STREAM_OK,
          "поток восстановился — состояние OK");
    check(s.transitions_to_lost == 1, "переход в LOST посчитан");
    check(s.max_gap_us == 60000 - 0, "худший разрыв запомнен");
}

// -------------------------------------------------------- модель батареи

static FcBatteryModel stand_model(int cells) {
    FcBatteryModel m = {
        .cells = cells,
        .cut_start = 34.0f,
        .cut_end = 31.0f,
        .regen_cut_start = 41.5f,
        .regen_cut_end = 42.0f,
        .max_vin = 57.0f,
        .min_vin = 8.0f,
        .refloat_tiltback_lv = 3.0f, // значения Refloat по умолчанию, на ячейку
        .refloat_tiltback_hv = 4.3f,
    };
    return m;
}

static void test_battery_model(void) {
    printf("\n\033[1mМодель батареи\033[0m\n");
    const char *why = NULL;

    // То, что реально стоит в конфигурации ESC на стенде.
    FcBatteryModel bad = stand_model(3);
    check(!fc_battery_model_valid(&bad, &why),
          "si_battery_cells = 3 при отсечках 34/31 В отвергается");
    note("причина: %s", why);
    note("34.0 В / 3 ячейки = %.2f В на ячейку — столько не бывает",
         (double) (bad.cut_start / 3.0f));

    // Правильное число ячеек согласует вольты — но вскрывает ВТОРУЮ
    // нестыковку, которая при cells = 3 была не видна за первой.
    FcBatteryModel good = stand_model(10);
    check(fc_battery_cells_from_regen_cutoff(&good) == 10,
          "число ячеек восстанавливается по regen-отсечке");
    note("пороги Refloat при 10S: LV %.1f В, HV %.1f В при отсечке ESC %.1f В",
         (double) fc_battery_refloat_lv_volts(&good),
         (double) fc_battery_refloat_hv_volts(&good), (double) good.cut_start);
    check(!fc_battery_model_valid(&good, &why),
          "10S с порогами Refloat по умолчанию всё ещё отвергается");
    note("причина: %s", why);
    note("отсечка ESC 3.4 В/яч выше предупреждения Refloat 3.0 В/яч — "
         "тяга пропадёт раньше, чем водителя предупредят");

    // Что именно надо изменить, чтобы модель сошлась: предупреждать раньше,
    // чем прошивка начнёт срезать ток. 3.5 В/ячейку даёт 35 В против 34.
    FcBatteryModel fixed = stand_model(10);
    fixed.refloat_tiltback_lv = 3.5f;
    check(fc_battery_model_valid(&fixed, &why),
          "порог отката 3.5 В/яч делает модель 10S непротиворечивой");
    note("LV %.1f В > отсечка %.1f В — предупреждение приходит первым",
         (double) fc_battery_refloat_lv_volts(&fixed), (double) fixed.cut_start);

    // Альтернатива: опустить отсечки ESC ниже предупреждения. Тоже сходится,
    // но трогает конфигурацию ESC, а не свою.
    FcBatteryModel alt = stand_model(10);
    alt.cut_start = 29.0f;
    alt.cut_end = 27.0f;
    check(fc_battery_model_valid(&alt, &why),
          "опускание отсечек ESC ниже 3.0 В/яч тоже устраняет противоречие");

    // Заглушка FloatCore: 15 ячеек для той же батареи.
    FcBatteryModel stub = stand_model(15);
    check(!fc_battery_model_valid(&stub, &why),
          "заглушка FloatCore на 15 ячеек для этой батареи отвергается");
    note("причина: %s", why);
    note("при 15 порог LV Refloat = %.1f В при отсечке ESC %.1f В — "
         "предупреждение позже, чем пропадёт тяга",
         (double) fc_battery_refloat_lv_volts(&stub), (double) stub.cut_start);

    // Порядок порогов.
    FcBatteryModel inv = stand_model(10);
    inv.cut_end = 36.0f;
    check(!fc_battery_model_valid(&inv, &why), "перевёрнутые отсечки отвергаются");

    inv = stand_model(10);
    inv.max_vin = 40.0f;
    check(!fc_battery_model_valid(&inv, &why),
          "аппаратный максимум ниже regen-отсечки отвергается");

    // Требование самого Refloat: отсечки должны быть НИЖЕ порога отката.
    FcBatteryModel late = stand_model(10);
    late.refloat_tiltback_lv = 3.2f;  // 32 В при отсечке 34 В
    check(!fc_battery_model_valid(&late, &why),
          "порог отката ниже отсечки ESC отвергается");
    note("причина: %s", why);
}

// --------------------------------------------------------- модель мотора

static void test_motor_model(void) {
    printf("\n\033[1mМодель параметров мотора\033[0m\n");
    size_t n = 0;
    const FcMotorParam *all = fc_motor_model_all(&n);
    check(n > 0 && all != NULL, "таблица параметров не пуста");

    // После v0.8A половины различаются: к 118 подключён мотор и её параметры
    // измерены, у 100 мотора нет. Проверяем именно это различие, а не
    // усреднённое «доверие» — усреднение скрыло бы, что готова только одна.
    const FcMotorParam *poles = fc_motor_model_find("si_motor_poles");
    check(poles != NULL, "число полюсов есть в модели");
    check(poles->value_a == 30.0f, "у половины A измерено 30 полюсов");
    check(poles->trust_a == FC_PARAM_VERIFIED, "полюса подтверждены на моторе A");
    check(poles->value_b == 14.0f && poles->trust_b == FC_PARAM_UNVERIFIED,
          "у половины B по-прежнему 14 и НЕ подтверждено");
    check(poles->kind == FC_PARAM_CONFIGURED,
          "число полюсов — введённое значение: детекция FOC его не определяет");
    note("критерий: %s", poles->verify_criterion);

    const FcMotorParam *flux = fc_motor_model_find("foc_motor_flux_linkage");
    check(flux != NULL, "потокосцепление есть в модели");
    check(flux->value_a > 0.015f && flux->value_a < 0.025f,
          "у половины A измерено около 19.6 мВб");
    check(flux->trust_a == FC_PARAM_VERIFIED, "потокосцепление подтверждено на моторе A");
    check(flux->value_b == 0.1f && flux->trust_b == FC_PARAM_UNVERIFIED,
          "у половины B осталось 0.1 и НЕ подтверждено");

    const FcMotorParam *r = fc_motor_model_find("foc_motor_r");
    check(r != NULL && r->trust_a == FC_PARAM_VERIFIED,
          "сопротивление подтверждено детекцией на моторе A");
    check(r->trust_b == FC_PARAM_READ_FROM_ESC,
          "у половины B сопротивление только прочитано");

    const FcMotorParam *cells = fc_motor_model_find("si_battery_cells");
    check(cells != NULL && cells->value_a == 10.0f && cells->value_b == 10.0f,
          "число ячеек 10 на обеих половинах");
    check(cells->trust_a == FC_PARAM_VERIFIED && cells->trust_b == FC_PARAM_VERIFIED,
          "число ячеек подтверждено измерением напряжения");

    const FcMotorParam *imax = fc_motor_model_find("l_current_max");
    check(imax != NULL && imax->kind == FC_PARAM_SAFETY_POLICY,
          "предел тока — решение о безопасности, а не свойство мотора");
    check(imax->value_a == 5.0f, "на половине A стоит временный предел прокрута 5 А");
    check(imax->value_b == 25.0f, "на половине B рабочие 25 А не менялись");
    check(!imax->must_refresh_after_detection, "детекция не должна менять предел тока");

    const FcMotorParam *wheel = fc_motor_model_find("si_wheel_diameter");
    check(wheel != NULL && wheel->trust_a == FC_PARAM_UNVERIFIED &&
              wheel->trust_b == FC_PARAM_UNVERIFIED,
          "диаметр колеса не подтверждён ни на одной половине");

    const char *why = NULL;
    check(!fc_motor_model_ready_for_output(&why),
          "модель НЕ готова к выходу на мотор");
    note("первое непройденное: %s", why ? why : "-");
    note("неподтверждённых хотя бы на одной половине: %zu из %zu",
         fc_motor_model_unverified_count(), n);

    // Готовность одной половины не делает машину готовой: требование
    // «подтверждено на обеих» намеренно строгое, машина двухмоторная.
    int verified_a = 0;
    for (size_t i = 0; i < n; ++i) {
        if (all[i].trust_a == FC_PARAM_VERIFIED) {
            ++verified_a;
        }
    }
    check(verified_a > 0, "на половине A уже есть подтверждённые параметры");
    check(!fc_motor_model_ready_for_output(NULL),
          "но модель всё равно не готова: половина B не подтверждена");
    note("подтверждено на половине A: %d параметров из %zu", verified_a, n);

    // Каждый параметр, измеряемый детекцией, обязан требовать обновления
    // после неё: иначе смысл детекции теряется.
    int detected = 0, must_refresh = 0;
    for (size_t i = 0; i < n; ++i) {
        if (all[i].kind == FC_PARAM_MEASURED_BY_DETECTION) {
            ++detected;
            if (all[i].must_refresh_after_detection) {
                ++must_refresh;
            }
        }
    }
    check(detected > 0 && detected == must_refresh,
          "все измеряемые детекцией параметры помечены к обновлению после неё");

    int with_criterion = 0;
    for (size_t i = 0; i < n; ++i) {
        if (all[i].verify_criterion && all[i].verify_criterion[0]) {
            ++with_criterion;
        }
    }
    check(with_criterion == (int) n, "у каждого параметра записан критерий подтверждения");
}

// ---------------------------------------- пороги отката против отсечек ESC

static void test_tiltback_vs_derating(void) {
    printf("\n\033[1mПороги отката против ограничений ESC (10S)\033[0m\n");
    const char *why = NULL;

    // Низкое напряжение. Предупреждение обязано прийти РАНЬШЕ, чем ESC
    // начнёт срезать ток, иначе водитель узнает о разряде по пропавшей тяге.
    FcBatteryModel m = stand_model(10);
    m.refloat_tiltback_lv = 3.5f;  // 35 В против отсечки 34 В
    m.refloat_tiltback_hv = 4.1f;  // 41 В против regen-отсечки 41.5 В
    check(fc_battery_model_valid(&m, &why), "предлагаемая политика 3.5/4.1 В/яч сходится");
    note("LV %.1f В > отсечка %.1f В: запас %.1f В на просадку под нагрузкой",
         (double) fc_battery_refloat_lv_volts(&m), (double) m.cut_start,
         (double) (fc_battery_refloat_lv_volts(&m) - m.cut_start));

    // Высокое напряжение — зеркальная задача. Значение Refloat по умолчанию
    // 4.3 В/яч даёт 43 В, а это ВЫШЕ полностью заряженной 10S (42.0 В):
    // порог недостижим, и предупреждение не придёт никогда, хотя regen ESC
    // начинает срезаться уже на 41.5 В.
    FcBatteryModel hv_default = stand_model(10);
    hv_default.refloat_tiltback_hv = 4.3f;
    check(fc_battery_refloat_hv_volts(&hv_default) > hv_default.regen_cut_start,
          "порог 4.3 В/яч выше начала regen-отсечки — предупреждение опаздывает");
    check(fc_battery_refloat_hv_volts(&hv_default) > hv_default.regen_cut_end,
          "порог 4.3 В/яч вообще недостижим для 10S");
    note("HV по умолчанию %.1f В при максимуме батареи %.1f В — никогда не сработает",
         (double) fc_battery_refloat_hv_volts(&hv_default), (double) hv_default.regen_cut_end);

    check(fc_battery_refloat_hv_volts(&m) < m.regen_cut_start,
          "предлагаемые 4.1 В/яч срабатывают раньше начала regen-отсечки");
    note("HV %.1f В < начало regen-отсечки %.1f В: запас %.1f В",
         (double) fc_battery_refloat_hv_volts(&m), (double) m.regen_cut_start,
         (double) (m.regen_cut_start - fc_battery_refloat_hv_volts(&m)));
}

// -------------------------------------------------- контракт разрешения

static FcMotorPermitInputs all_true(void) {
    FcMotorPermitInputs in;
    memset(&in, 1, sizeof(in));  // все bool в true
    return in;
}

static void test_motor_permit(void) {
    printf("\n\033[1mКонтракт разрешения выхода на мотор\033[0m\n");

    FcMotorPermitInputs in;
    memset(&in, 0, sizeof(in));
    FcMotorPermitResult r;
    fc_motor_permit_evaluate(&in, &r);
    check(!r.permitted, "при всех входах false разрешения нет");

    uint32_t n_names = 0;
    fc_motor_permit_condition_names(&n_names);
    check(r.reason_count == n_names, "перечислены ВСЕ невыполненные условия, а не первое");
    note("условий в контракте: %u", n_names);
    check(n_names >= 20, "контракт расширен проверками качества платформы (ТЗ v0.7D §19)");

    // Новые входы обязаны запрещать так же, как и старые.
    const char *added[] = {"realtime-квалификация не пройдена", "модель батареи не проверена",
                           "политика сторожевого таймера не проверена",
                           "параметры мотора не подтверждены на моторе"};
    const char *const *names = fc_motor_permit_condition_names(NULL);
    for (size_t a = 0; a < sizeof(added) / sizeof(added[0]); ++a) {
        bool found = false;
        for (uint32_t i = 0; i < n_names; ++i) {
            if (strcmp(names[i], added[a]) == 0) {
                found = true;
            }
        }
        check(found, added[a]);
    }

    in = all_true();
    fc_motor_permit_evaluate(&in, &r);
    check(r.permitted && r.reason_count == 0, "при всех выполненных условиях контракт сходится");

    // Каждое условие поодиночке обязано запрещать.
    int blocking = 0;
    bool *fields = (bool *) &in;
    size_t field_count = sizeof(in) / sizeof(bool);
    for (size_t i = 0; i < field_count; ++i) {
        in = all_true();
        fields = (bool *) &in;
        fields[i] = false;
        fc_motor_permit_evaluate(&in, &r);
        if (!r.permitted && r.reason_count == 1) {
            ++blocking;
        }
    }
    check(blocking == (int) field_count,
          "каждое условие поодиночке запрещает выход и называет себя");
    note("проверено по одному: %d условий", blocking);

    // Главное: в этой сборке выхода нет вовсе.
    check(!fc_motor_permit_available_in_this_build(),
          "в этой сборке выход на мотор недоступен как код");
    check(FC_MOTOR_BACKEND_AVAILABLE == 0, "backend вывода отсутствует");
    check(strcmp(fc_motor_gate_backend_name(), "none (blocked)") == 0,
          "Motor Gate по-прежнему без backend-а");
}

// ------------------------------------------------------------- журнал

static void push(FcLog *l, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    fc_log_push(l, FC_LOG_INFO, "test", 42, fmt, ap);
    va_end(ap);
}

static void test_logger(void) {
    printf("\n\033[1mЖурнал: ограниченность и отсутствие блокировки\033[0m\n");
    FcLog l;
    fc_log_init(&l);

    check(fc_log_pending(&l) == 0, "кольцо начинается пустым");

    push(&l, "строка %d", 1);
    check(fc_log_pending(&l) == 1, "запись принята");

    FcLogEntry e;
    check(fc_log_pop(&l, &e), "запись читается обратно");
    check(strcmp(e.msg, "строка 1") == 0, "содержимое не искажено");
    check(strcmp(e.tag, "test") == 0, "тег сохранён");
    check(e.t_us == 42, "отметка времени — момент события, а не печати");
    check(!fc_log_pop(&l, &e), "после выборки кольцо снова пусто");

    // Переполнение не блокирует и не портит уже принятое.
    fc_log_init(&l);
    for (int i = 0; i < FC_LOG_SLOTS + 20; ++i) {
        push(&l, "%d", i);
    }
    check(fc_log_pending(&l) == FC_LOG_SLOTS, "кольцо не растёт сверх размера");
    check(l.dropped == 20, "лишние записи отброшены и посчитаны, а не потеряны молча");
    check(l.high_water == FC_LOG_SLOTS, "пик заполнения зафиксирован");
    check(fc_log_pop(&l, &e) && strcmp(e.msg, "0") == 0,
          "при переполнении сохраняются ПЕРВЫЕ записи: они объясняют причину");

    // Длинное сообщение обрезается, но не портит соседей и считается.
    fc_log_init(&l);
    char big[FC_LOG_MSG_MAX * 2];
    memset(big, 'x', sizeof(big) - 1);
    big[sizeof(big) - 1] = '\0';
    push(&l, "%s", big);
    check(l.truncated == 1, "обрезание сообщения посчитано");
    check(fc_log_pop(&l, &e), "обрезанная запись всё равно доступна");
    check(strlen(e.msg) == FC_LOG_MSG_MAX - 1, "длина ограничена размером слота");

    // Порядок сохраняется.
    fc_log_init(&l);
    for (int i = 0; i < 10; ++i) {
        push(&l, "%d", i);
    }
    bool ordered = true;
    for (int i = 0; i < 10; ++i) {
        if (!fc_log_pop(&l, &e) || atoi(e.msg) != i) {
            ordered = false;
        }
    }
    check(ordered, "порядок записей сохраняется");
}

void test_motor_prep_all(void) {
    test_watchdog_policy();
    test_command_stream();
    test_battery_model();
    test_motor_model();
    test_tiltback_vs_derating();
    test_motor_permit();
    test_logger();
}
