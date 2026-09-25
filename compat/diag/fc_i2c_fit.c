#include "fc_i2c_fit.h"

#include <math.h>

FcI2cFit fc_i2c_fit(const uint8_t *len, const double *t_us, int n) {
    FcI2cFit f = {0};
    if (!len || !t_us || n < 3) {
        return f;
    }
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (int k = 0; k < n; ++k) {
        sx += len[k];
        sy += t_us[k];
        sxx += (double) len[k] * len[k];
        sxy += (double) len[k] * t_us[k];
    }
    double den = (double) n * sxx - sx * sx;
    // Все N одинаковы — наклон не определён. Молча выдать ноль значило бы
    // сообщить «бесконечно быструю шину».
    if (fabs(den) < 1e-9) {
        return f;
    }
    f.slope_us_per_byte = ((double) n * sxy - sx * sy) / den;
    f.intercept_us = (sy - f.slope_us_per_byte * sx) / (double) n;
    for (int k = 0; k < n; ++k) {
        double e = fabs(t_us[k] - (f.intercept_us + f.slope_us_per_byte * len[k]));
        if (e > f.max_residual_us) {
            f.max_residual_us = e;
        }
    }
    f.linear = f.max_residual_us < FC_I2C_FIT_MAX_RESIDUAL_US;
    if (f.slope_us_per_byte > 0.0) {
        double t_clk = f.slope_us_per_byte / 9.0;
        f.wire_hz = 1e6 / t_clk;
        f.fixed_wire_us = FC_I2C_FIXED_CLOCKS * t_clk;
        f.overhead_us = f.intercept_us - f.fixed_wire_us;
    }
    f.valid = f.slope_us_per_byte > 0.0;
    return f;
}
