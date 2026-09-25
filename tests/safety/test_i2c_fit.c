// Разделение времени I²C на провод и накладные расходы (ТЗ v0.9I §2, §9, §25).
//
// Данные синтетические, с заранее известным ответом: так проверяется сама
// арифметика, а не удачный замер. Именно в этой арифметике была ошибка v0.9H —
// всё время вызова тогда было поделено на такты.

#include "../../compat/diag/fc_i2c_fit.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

void fc_test_check(bool ok, const char *what);
void fc_test_note(const char *fmt, ...);
#define check fc_test_check
#define note fc_test_note

static const uint8_t LEN[7] = {1, 2, 4, 8, 14, 20, 26};

static void synth(double hz, double overhead_us, double *t) {
    double t_clk = 1e6 / hz;
    for (int k = 0; k < 7; ++k) {
        t[k] = overhead_us + FC_I2C_FIXED_CLOCKS * t_clk + 9.0 * t_clk * LEN[k];
    }
}

static void test_recovers_400khz(void) {
    note("честные 400 кГц восстанавливаются по наклону");
    double t[7];
    synth(400000.0, 190.0, t);
    FcI2cFit f = fc_i2c_fit(LEN, t, 7);
    check(f.valid && f.linear, "подгонка годна");
    check(fabs(f.slope_us_per_byte - 22.5) < 0.01, "наклон 22.5 мкс на байт");
    check(fabs(f.wire_hz - 400000.0) < 1.0, "частота на проводе 400 кГц");
    check(fabs(f.overhead_us - 190.0) < 0.5, "накладные расходы отделены от провода");
}

static void test_overhead_does_not_look_like_slow_bus(void) {
    note("дорогой драйвер НЕ выглядит медленной шиной");
    // Ровно ошибка v0.9H: суммарное время для 14 байт около 900 мкс. Поделив
    // его на такты, я получил «187 кГц». Здесь шина честная, дорог драйвер.
    double t[7];
    synth(400000.0, 500.0, t);
    FcI2cFit f = fc_i2c_fit(LEN, t, 7);
    double total14 = f.intercept_us + f.slope_us_per_byte * 14;
    check(total14 > 800.0, "суммарное время на 14 байт как в живом контуре");
    check(fabs(f.wire_hz - 400000.0) < 1.0, "а шина всё равно 400 кГц");
    check(f.overhead_us > 490.0, "вся разница — в накладных");
}

static void test_detects_slow_bus(void) {
    note("действительно медленная шина видна по наклону");
    double t[7];
    synth(187000.0, 190.0, t);
    FcI2cFit f = fc_i2c_fit(LEN, t, 7);
    check(fabs(f.wire_hz - 187000.0) < 1.0, "187 кГц восстанавливаются");
    check(f.slope_us_per_byte > 48.0, "наклон около 48 мкс на байт");
}

static void test_nonlinear_rejected(void) {
    note("нелинейные данные помечаются как негодные");
    // Как на ядре 1 при вытеснении: минимумы разбросаны, а не лежат на прямой.
    double t[7] = {873, 900, 1400, 900, 873, 2100, 1300};
    FcI2cFit f = fc_i2c_fit(LEN, t, 7);
    check(!f.linear, "модель признана негодной");
    check(f.max_residual_us > FC_I2C_FIT_MAX_RESIDUAL_US, "отклонение названо");
}

static void test_degenerate(void) {
    note("вырожденные данные не дают выдуманной частоты");
    uint8_t same[4] = {14, 14, 14, 14};
    double t[4] = {600, 610, 605, 600};
    FcI2cFit f = fc_i2c_fit(same, t, 4);
    check(!f.valid, "одинаковые N: подгонка отвергнута");
    check(f.wire_hz == 0.0, "частота не выдумана");
    FcI2cFit g = fc_i2c_fit(LEN, t, 2);
    check(!g.valid, "двух точек мало");
}

void test_i2c_fit_all(void) {
    test_recovers_400khz();
    test_overhead_does_not_look_like_slow_bus();
    test_detects_slow_bus();
    test_nonlinear_rejected();
    test_degenerate();
}
