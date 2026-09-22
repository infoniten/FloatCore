// Теневой разбор: учёт упора в предел и привязка корзин к ошибке регулятора
// (ТЗ v0.9F §14, §22).
//
// Эти тесты закрепляют два дефекта, которые проявлялись НЕ отказом, а тихо
// неверным числом в отчёте — то есть худшим из возможных способов:
//
//   1. Упор в предел считался точным `>=`. Зажатие происходит внутри Refloat
//      над float-величиной и даёт результат на единицы ULP ниже предела,
//      поэтому счётчик не срабатывал никогда. Отчёт показывал «упор ESC
//      0.0 %» там, где команда стояла в потолке 100 % времени, и по этому
//      числу делались выводы о пригодности коэффициентов.
//
//   2. Корзина выбирается по ошибке РЕГУЛЯТОРА, а не по углу. Refloat считает
//      её от balance_pitch и setpoint (pid.c:58), а не от сырого pitch:
//      setpoint на стенде уходит от нуля, и привязка к pitch смещала бы всю
//      таблицу.

#include "../../compat/diag/fc_shadow.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

void fc_test_check(bool ok, const char *what);
void fc_test_note(const char *fmt, ...);
#define check fc_test_check
#define note fc_test_note

static void record_one(float current, float setpoint, float balance_pitch) {
    FcShadowSample s;
    memset(&s, 0, sizeof(s));
    s.current = current;
    s.setpoint = setpoint;
    s.balance_pitch = balance_pitch;
    s.pitch = balance_pitch;
    fc_shadow_record(&s);
}

// Сумма упоров по всем корзинам: конкретная корзина для этой проверки
// неважна, важен сам факт учёта.
static uint64_t total_sat_esc(const FcShadowStats *st) {
    uint64_t n = 0;
    for (unsigned i = 0; i < FC_SHADOW_ANGLE_BINS; ++i) {
        n += st->bin_saturated_esc[i];
    }
    return n;
}

static uint64_t total_sat_env(const FcShadowStats *st) {
    uint64_t n = 0;
    for (unsigned i = 0; i < FC_SHADOW_ANGLE_BINS; ++i) {
        n += st->bin_saturated_env[i];
    }
    return n;
}

static void test_saturation_at_exact_limit(void) {
    note("упор в предел ESC считается и на самой границе");

    fc_shadow_reset();
    fc_shadow_set_esc_limit(5.0f);
    fc_shadow_set_envelope(0.5f);

    // Ровно предел. Именно этот случай молчал до v0.9F.
    record_one(5.0f, 0.0f, -2.0f);
    FcShadowStats st = fc_shadow_stats();
    check(total_sat_esc(&st) == 1, "ток РОВНО в предел ESC засчитан как упор");

    // На единицы ULP ниже предела — тоже упор: именно такие значения и
    // приходят из зажатия внутри Refloat.
    fc_shadow_reset();
    record_one(5.0f - 1e-6f, 0.0f, -2.0f);
    st = fc_shadow_stats();
    check(total_sat_esc(&st) == 1, "ток на единицы ULP ниже предела засчитан как упор");

    // Заведомо ниже предела упором быть не должен: допуск не смеет
    // превратиться в широкую полосу.
    fc_shadow_reset();
    record_one(4.9f, 0.0f, -2.0f);
    st = fc_shadow_stats();
    check(total_sat_esc(&st) == 0, "ток на 0.1 А ниже предела упором НЕ считается");

    // Допуск должен быть строго меньше различимого тока, иначе он начнёт
    // засчитывать то, что упором не является.
    check(FC_SHADOW_LIMIT_EPS_A < 0.01f, "допуск меньше любого различимого тока");
}

static void test_envelope_saturation_at_exact_limit(void) {
    note("упор в предел опыта считается по тому же правилу");

    fc_shadow_reset();
    fc_shadow_set_esc_limit(5.0f);
    fc_shadow_set_envelope(0.5f);

    record_one(0.5f, 0.0f, -2.0f);
    FcShadowStats st = fc_shadow_stats();
    check(total_sat_env(&st) == 1, "ток РОВНО в предел опыта засчитан как упор");
    check(total_sat_esc(&st) == 0, "но упором в предел ESC он не считается");

    fc_shadow_reset();
    record_one(0.49f, 0.0f, -2.0f);
    st = fc_shadow_stats();
    check(total_sat_env(&st) == 0, "ток ниже предела опыта упором НЕ считается");
}

static void test_bin_follows_controller_error(void) {
    note("корзина выбирается по ошибке регулятора, а не по сырому углу");

    fc_shadow_reset();
    fc_shadow_set_esc_limit(5.0f);
    fc_shadow_set_envelope(0.5f);

    // Угол большой, но setpoint ушёл за ним: ошибка регулятора мала, и
    // запись обязана попасть в НИЖНЮЮ корзину. Привязка к pitch отправила бы
    // её в верхнюю.
    record_one(0.05f, -3.0f, -3.05f);
    FcShadowStats st = fc_shadow_stats();
    check(st.bin_n[0] == 1, "при pitch -3.05° и setpoint -3.0° запись в корзине 0.0-0.1°");

    uint64_t upper = 0;
    for (unsigned i = 1; i < FC_SHADOW_ANGLE_BINS; ++i) {
        upper += st.bin_n[i];
    }
    check(upper == 0, "ни одна верхняя корзина не задета");

    // Знак ошибки на выбор корзины не влияет: берётся модуль.
    fc_shadow_reset();
    record_one(-0.05f, -3.0f, -2.95f);
    st = fc_shadow_stats();
    check(st.bin_n[0] == 1, "ошибка противоположного знака попадает в ту же корзину");
}

static void test_deadzone_classification(void) {
    note("границы мёртвой зоны разделяют три области");

    fc_shadow_reset();
    fc_shadow_set_esc_limit(5.0f);
    fc_shadow_set_envelope(5.0f);

    record_one(0.29f, 0.0f, 0.0f);
    record_one(0.35f, 0.0f, 0.0f);
    record_one(0.45f, 0.0f, 0.0f);
    // Знак не должен менять классификацию: считается модуль.
    record_one(-0.29f, 0.0f, 0.0f);

    FcShadowStats st = fc_shadow_stats();
    check(st.below_low == 2, "ниже 0.30 А — физически неэффективная область, знак не важен");
    check(st.in_band == 1, "0.30-0.40 А — полоса частичного отклика");
    check(st.above_high == 1, "выше 0.40 А — устойчивый отклик");
    check(st.samples == 4, "учтены все записи");
}

void test_shadow_all(void) {
    fc_shadow_init();
    test_saturation_at_exact_limit();
    test_envelope_saturation_at_exact_limit();
    test_bin_follows_controller_error();
    test_deadzone_classification();
}
