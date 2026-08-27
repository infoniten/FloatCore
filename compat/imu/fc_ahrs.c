#include "fc_ahrs.h"

#include <math.h>

void fc_ahrs_init(FcAhrs *a) {
    a->q0 = 1.0f;
    a->q1 = 0.0f;
    a->q2 = 0.0f;
    a->q3 = 0.0f;
    a->acc_mag = 1.0f;
    a->kp = FC_AHRS_DEFAULT_KP;
    a->acc_confidence_decay = FC_AHRS_DEFAULT_ACC_CONFIDENCE_DECAY;
    a->initialised = false;
}

void fc_ahrs_configure(FcAhrs *a, float kp, float acc_confidence_decay) {
    // Отрицательные и нулевые значения не принимаются: при kp = 0 фильтр
    // перестаёт корректироваться акселерометром и уходит по смещению
    // гироскопа, а обнаружилось бы это уже под ногами.
    if (kp > 0.0f) {
        a->kp = kp;
    }
    if (acc_confidence_decay >= 0.0f) {
        a->acc_confidence_decay = acc_confidence_decay;
    }
}

void fc_ahrs_set_from_accel(FcAhrs *a, const float accel_g[3]) {
    float ax = accel_g[0], ay = accel_g[1], az = accel_g[2];
    float n = sqrtf(ax * ax + ay * ay + az * az);
    if (!(n > 0.01f)) {
        return;
    }
    ax /= n;
    ay /= n;
    az /= n;

    // Акселерометр в покое измеряет направление «вверх» в осях датчика
    // (balance_filter.c:99-101: при единичном кватернионе оценка гравитации
    // равна +Z). Углы берутся в том же соглашении, что и на выходе фильтра:
    // roll = -atan2(ay, az), pitch = asin(-ax). Знаки не подбираются — они
    // следуют из формул извлечения углов, к которым фильтр и должен сойтись.
    float roll = -atan2f(ay, az);
    float sin_pitch = -ax;
    if (sin_pitch < -1.0f) {
        sin_pitch = -1.0f;
    } else if (sin_pitch > 1.0f) {
        sin_pitch = 1.0f;
    }
    float pitch = asinf(sin_pitch);

    // Кватернион из (roll, pitch, yaw = 0). Порядок соответствует формулам
    // извлечения: сначала рыскание, затем тангаж, затем крен.
    float cr = cosf(-roll * 0.5f), sr = sinf(-roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    a->q0 = cr * cp;
    a->q1 = sr * cp;
    a->q2 = cr * sp;
    a->q3 = -sr * sp;
    a->acc_mag = n;
    a->initialised = true;
}

void fc_ahrs_update(FcAhrs *a, const float gyro_rad_s[3], const float accel_g[3], float dt) {
    float gx = gyro_rad_s[0];
    float gy = gyro_rad_s[1];
    float gz = gyro_rad_s[2];

    float ax = accel_g[0];
    float ay = accel_g[1];
    float az = accel_g[2];

    float accel_norm = sqrtf(ax * ax + ay * ay + az * az);

    // Коррекция считается только если вектор не вырожден: иначе деление на
    // почти ноль (balance_filter.c:84-86).
    if (accel_norm > 0.01f) {
        // Доверие к акселерометру падает, когда модуль вектора уходит от 1 g:
        // значит доска ускоряется, и «вниз» по акселерометру уже не вниз.
        // Коэффициент затухания — параметр, а не константа: его значение
        // платформа сообщает Refloat, и он же должен применяться здесь.
        a->acc_mag = a->acc_mag * 0.9f + accel_norm * 0.1f;
        float confidence = 1.0f - a->acc_confidence_decay * sqrtf(fabsf(a->acc_mag - 1.0f));
        if (confidence < 0.0f) {
            confidence = 0.0f;
        }
        float two_kp = 2.0f * a->kp * confidence;

        float r = 1.0f / accel_norm;
        ax *= r;
        ay *= r;
        az *= r;

        // Оценка направления гравитации из текущего кватерниона.
        float halfvx = a->q1 * a->q3 - a->q0 * a->q2;
        float halfvy = a->q0 * a->q1 + a->q2 * a->q3;
        float halfvz = a->q0 * a->q0 - 0.5f + a->q3 * a->q3;

        // Ошибка — векторное произведение оценки и измерения.
        float halfex = ay * halfvz - az * halfvy;
        float halfey = az * halfvx - ax * halfvz;
        float halfez = ax * halfvy - ay * halfvx;

        gx += two_kp * halfex;
        gy += two_kp * halfey;
        gz += two_kp * halfez;
    }

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    float qa = a->q0;
    float qb = a->q1;
    float qc = a->q2;
    a->q0 += (-qb * gx - qc * gy - a->q3 * gz);
    a->q1 += (qa * gx + qc * gz - a->q3 * gy);
    a->q2 += (qa * gy - qb * gz + a->q3 * gx);
    a->q3 += (qa * gz + qb * gy - qc * gx);

    float n = sqrtf(a->q0 * a->q0 + a->q1 * a->q1 + a->q2 * a->q2 + a->q3 * a->q3);
    if (n > 1e-9f) {
        float inv = 1.0f / n;
        a->q0 *= inv;
        a->q1 *= inv;
        a->q2 *= inv;
        a->q3 *= inv;
    }
}

float fc_ahrs_roll(const FcAhrs *a) {
    return -atan2f(a->q0 * a->q1 + a->q2 * a->q3, 0.5f - (a->q1 * a->q1 + a->q2 * a->q2));
}

float fc_ahrs_pitch(const FcAhrs *a) {
    float s = -2.0f * (a->q1 * a->q3 - a->q0 * a->q2);
    if (s < -1.0f) {
        return -(float) M_PI / 2.0f;
    }
    if (s > 1.0f) {
        return (float) M_PI / 2.0f;
    }
    return asinf(s);
}

float fc_ahrs_yaw(const FcAhrs *a) {
    return -atan2f(a->q0 * a->q3 + a->q1 * a->q2, 0.5f - (a->q2 * a->q2 + a->q3 * a->q3));
}

void fc_ahrs_quaternion(const FcAhrs *a, float q[4]) {
    q[0] = a->q0;
    q[1] = a->q1;
    q[2] = a->q2;
    q[3] = a->q3;
}
