#include "fc_imu_detect.h"

#include "fc_ahrs.h"

#include <math.h>
#include <string.h>

#define RAD2DEG 57.29577951308232f

static struct {
    FcDetectConfig cfg;
    FcDetectStatus st;
    float yaw_deg;

    double sum_a[3], sum_a2[3];
    double sum_g[3], sum_g2[3];
    uint64_t last_us;
    bool have_last;
} D;

FcDetectConfig fc_imu_detect_default_config(void) {
    FcDetectConfig c;
    // 1000 семплов при 500 Гц — две секунды. Столько же усредняет штатная
    // процедура VESC (там 1000 при 1000 Гц). Больше двух секунд просить у
    // человека, который держит доску неподвижно, уже неразумно.
    c.samples = 1000;
    // Пороги СКО взяты по измеренному шуму покоя, а не назначены: на этой
    // плате СКО ускорения около 0.003 g, угловой скорости около 0.15 °/с.
    // Пороги втрое-впятеро выше — отделяют шум от дрожания рук, но не
    // отвергают исправный датчик.
    c.max_gyro_std_dps = 0.8f;
    c.max_accel_std_g = 0.015f;
    // Мгновенный выброс: доска, которую взяли в руки, сразу даёт десятки °/с.
    c.max_gyro_instant_dps = 5.0f;
    c.accel_mag_tol_g = 0.05f;
    // Разрыв в потоке: при 500 Гц нормальный интервал 2 мс. Двадцать
    // миллисекунд означают, что семплы терялись, и усреднять такое нельзя.
    c.max_gap_us = 20000;
    return c;
}

static void reset_accumulation(void) {
    memset(D.sum_a, 0, sizeof(D.sum_a));
    memset(D.sum_a2, 0, sizeof(D.sum_a2));
    memset(D.sum_g, 0, sizeof(D.sum_g));
    memset(D.sum_g2, 0, sizeof(D.sum_g2));
    D.st.samples = 0;
    D.have_last = false;
}

void fc_imu_detect_start(const FcDetectConfig *cfg, float yaw_deg) {
    memset(&D, 0, sizeof(D));
    D.cfg = *cfg;
    D.yaw_deg = yaw_deg;
    D.st.state = FC_DETECT_COLLECTING;
    D.st.needed = cfg->samples;
    D.st.result = fc_imu_calibration_identity();
    reset_accumulation();
}

void fc_imu_detect_abort(void) {
    D.st.state = FC_DETECT_IDLE;
}

static void restart(FcDetectFailure why) {
    if (D.st.samples) {
        ++D.st.restarts;
    }
    D.st.failure = why;
    reset_accumulation();
}

FcDetectState fc_imu_detect_feed(
    const float accel_raw_g[3], const float gyro_raw_dps[3], uint64_t now_us
) {
    if (D.st.state != FC_DETECT_COLLECTING) {
        return D.st.state;
    }

    // Непрерывность потока. Пропуск семплов означает, что усреднение идёт по
    // рваному куску времени; на движущейся доске это ещё и скрывает движение.
    if (D.have_last && now_us > D.last_us && (now_us - D.last_us) > D.cfg.max_gap_us) {
        restart(FC_DETECT_FAIL_DISCONTINUITY);
        D.last_us = now_us;
        D.have_last = true;
        return D.st.state;
    }
    D.last_us = now_us;
    D.have_last = true;

    // Мгновенный выброс угловой скорости: доску взяли в руки.
    for (int i = 0; i < 3; ++i) {
        if (!isfinite(gyro_raw_dps[i]) || fabsf(gyro_raw_dps[i]) > D.cfg.max_gyro_instant_dps) {
            restart(FC_DETECT_FAIL_MOTION);
            return D.st.state;
        }
        if (!isfinite(accel_raw_g[i])) {
            restart(FC_DETECT_FAIL_ACCEL_MAG);
            return D.st.state;
        }
    }

    float amag = sqrtf(accel_raw_g[0] * accel_raw_g[0] + accel_raw_g[1] * accel_raw_g[1] +
                       accel_raw_g[2] * accel_raw_g[2]);
    if (fabsf(amag - 1.0f) > D.cfg.accel_mag_tol_g) {
        restart(FC_DETECT_FAIL_ACCEL_MAG);
        return D.st.state;
    }

    for (int i = 0; i < 3; ++i) {
        D.sum_a[i] += accel_raw_g[i];
        D.sum_a2[i] += (double) accel_raw_g[i] * accel_raw_g[i];
        D.sum_g[i] += gyro_raw_dps[i];
        D.sum_g2[i] += (double) gyro_raw_dps[i] * gyro_raw_dps[i];
    }
    ++D.st.samples;

    if (D.st.samples < D.cfg.samples) {
        return D.st.state;
    }

    // --- накопили: считаем средние и СКО ------------------------------------
    double n = (double) D.st.samples;
    for (int i = 0; i < 3; ++i) {
        double ma = D.sum_a[i] / n;
        double mg = D.sum_g[i] / n;
        double va = D.sum_a2[i] / n - ma * ma;
        double vg = D.sum_g2[i] / n - mg * mg;
        D.st.accel_mean_g[i] = (float) ma;
        D.st.gyro_mean_dps[i] = (float) mg;
        D.st.accel_std_g[i] = (float) sqrt(va > 0 ? va : 0);
        D.st.gyro_std_dps[i] = (float) sqrt(vg > 0 ? vg : 0);
    }
    D.st.accel_mag_g = sqrtf(D.st.accel_mean_g[0] * D.st.accel_mean_g[0] +
                             D.st.accel_mean_g[1] * D.st.accel_mean_g[1] +
                             D.st.accel_mean_g[2] * D.st.accel_mean_g[2]);

    // Дисперсия — вторая, независимая проверка неподвижности. Мгновенный порог
    // ловит рывок, СКО ловит медленное покачивание, которое каждый отдельный
    // семпл проходит.
    for (int i = 0; i < 3; ++i) {
        if (D.st.gyro_std_dps[i] > D.cfg.max_gyro_std_dps ||
            D.st.accel_std_g[i] > D.cfg.max_accel_std_g) {
            restart(FC_DETECT_FAIL_MOTION);
            return D.st.state;
        }
    }

    // --- результат ----------------------------------------------------------
    FcImuCalibration c = fc_imu_calibration_identity();

    // Углы текущего положения в том же соглашении, что и на выходе AHRS
    // (fc_ahrs.c, формулы из bldc/imu/ahrs.c).
    float ax = D.st.accel_mean_g[0] / D.st.accel_mag_g;
    float ay = D.st.accel_mean_g[1] / D.st.accel_mag_g;
    float az = D.st.accel_mean_g[2] / D.st.accel_mag_g;
    float roll_rad = -atan2f(ay, az);
    float sp = -ax;
    if (sp < -1.0f) {
        sp = -1.0f;
    } else if (sp > 1.0f) {
        sp = 1.0f;
    }
    float pitch_rad = asinf(sp);

    // Знаки взяты из штатной процедуры VESC (bldc/imu/imu.c):
    //     m_settings.rot_roll  = -RAD2DEG_f(roll_sample);
    //     m_settings.rot_pitch =  RAD2DEG_f(pitch_sample);
    // Проверяются host-тестом: после применения этой калибровки к тому же
    // вектору ускорения углы обязаны стать нулевыми.
    c.rot_roll_deg = -roll_rad * RAD2DEG;
    c.rot_pitch_deg = pitch_rad * RAD2DEG;
    c.rot_yaw_deg = D.yaw_deg;

    // Смещения гироскопа хранятся в ПОВЁРНУТОЙ системе координат, потому что
    // вычитаются после поворота. Измерены они в сырой — значит их надо
    // повернуть той же матрицей.
    float m[9];
    fc_imu_calibration_matrix(&c, m);
    const float *b = D.st.gyro_mean_dps;
    c.gyro_offset_dps[0] = b[0] * m[0] + b[1] * m[1] + b[2] * m[2];
    c.gyro_offset_dps[1] = b[0] * m[3] + b[1] * m[4] + b[2] * m[5];
    c.gyro_offset_dps[2] = b[0] * m[6] + b[1] * m[7] + b[2] * m[8];

    if (!fc_imu_calibration_plausible(&c)) {
        D.st.failure = FC_DETECT_FAIL_IMPLAUSIBLE;
        D.st.state = FC_DETECT_FAILED;
        return D.st.state;
    }

    D.st.result = c;
    D.st.failure = FC_DETECT_FAIL_NONE;
    D.st.state = FC_DETECT_DONE;
    return D.st.state;
}

FcDetectStatus fc_imu_detect_status(void) {
    return D.st;
}

const char *fc_imu_detect_state_name(FcDetectState s) {
    switch (s) {
    case FC_DETECT_IDLE:
        return "IDLE";
    case FC_DETECT_COLLECTING:
        return "COLLECTING";
    case FC_DETECT_DONE:
        return "DONE";
    case FC_DETECT_FAILED:
        return "FAILED";
    default:
        return "?";
    }
}

const char *fc_imu_detect_failure_name(FcDetectFailure f) {
    switch (f) {
    case FC_DETECT_FAIL_NONE:
        return "нет";
    case FC_DETECT_FAIL_MOTION:
        return "доска двигалась";
    case FC_DETECT_FAIL_ACCEL_MAG:
        return "модуль ускорения не соответствует покою";
    case FC_DETECT_FAIL_DISCONTINUITY:
        return "поток семплов прерывался";
    case FC_DETECT_FAIL_IMPLAUSIBLE:
        return "результат неправдоподобен";
    default:
        return "?";
    }
}
