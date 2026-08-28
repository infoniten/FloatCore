#include "fc_imu_calibration.h"

#include <math.h>
#include <string.h>

#define DEG2RAD 0.017453292519943295f

// Границы правдоподобия.
//
// Повороты: полный диапазон углов Эйлера. Больше по модулю — это не поворот,
// а испорченное число.
#define FC_IMU_CAL_MAX_ROT_DEG 180.0f
// Смещение гироскопа: у исправного ICM-20948 это единицы °/с (измерено 0.3…1.3).
// Двадцать — с большим запасом на худший экземпляр, но заведомо меньше, чем
// даст движущаяся доска.
#define FC_IMU_CAL_MAX_GYRO_OFFSET_DPS 20.0f
#define FC_IMU_CAL_MAX_ACCEL_OFFSET_G 1.0f

FcImuCalibration fc_imu_calibration_identity(void) {
    FcImuCalibration c;
    memset(&c, 0, sizeof(c));
    return c;
}

bool fc_imu_calibration_is_identity(const FcImuCalibration *c) {
    return c->rot_roll_deg == 0.0f && c->rot_pitch_deg == 0.0f && c->rot_yaw_deg == 0.0f &&
           c->gyro_offset_dps[0] == 0.0f && c->gyro_offset_dps[1] == 0.0f &&
           c->gyro_offset_dps[2] == 0.0f && c->accel_offset_g[0] == 0.0f &&
           c->accel_offset_g[1] == 0.0f && c->accel_offset_g[2] == 0.0f;
}

void fc_imu_calibration_matrix(const FcImuCalibration *c, float m[9]) {
    // Посимвольно как в bldc/imu/imu.c. Порядок углов: yaw -> pitch -> roll.
    float s1 = sinf(c->rot_yaw_deg * DEG2RAD);
    float c1 = cosf(c->rot_yaw_deg * DEG2RAD);
    float s2 = sinf(c->rot_pitch_deg * DEG2RAD);
    float c2 = cosf(c->rot_pitch_deg * DEG2RAD);
    float s3 = sinf(c->rot_roll_deg * DEG2RAD);
    float c3 = cosf(c->rot_roll_deg * DEG2RAD);

    m[0] = c1 * c2;
    m[1] = c1 * s2 * s3 - c3 * s1;
    m[2] = s1 * s3 + c1 * c3 * s2;
    m[3] = c2 * s1;
    m[4] = c1 * c3 + s1 * s2 * s3;
    m[5] = c3 * s1 * s2 - c1 * s3;
    m[6] = -s2;
    m[7] = c2 * s3;
    m[8] = c2 * c3;
}

void fc_imu_calibration_prepare(const FcImuCalibration *c, FcImuCalibrationPrepared *out) {
    fc_imu_calibration_matrix(c, out->m);
    for (int i = 0; i < 3; ++i) {
        out->gyro_offset_dps[i] = c->gyro_offset_dps[i];
        out->accel_offset_g[i] = c->accel_offset_g[i];
    }
}

void fc_imu_calibration_apply_prepared(
    const FcImuCalibrationPrepared *p,
    const float accel_in_g[3],
    const float gyro_in_dps[3],
    float accel_out_g[3],
    float gyro_out_dps[3]
) {
    const float *m = p->m;

    // Копии на случай, если вход и выход — один и тот же массив.
    float ax = accel_in_g[0], ay = accel_in_g[1], az = accel_in_g[2];
    float gx = gyro_in_dps[0], gy = gyro_in_dps[1], gz = gyro_in_dps[2];

    // 1. Поворот.
    accel_out_g[0] = ax * m[0] + ay * m[1] + az * m[2];
    accel_out_g[1] = ax * m[3] + ay * m[4] + az * m[5];
    accel_out_g[2] = ax * m[6] + ay * m[7] + az * m[8];

    gyro_out_dps[0] = gx * m[0] + gy * m[1] + gz * m[2];
    gyro_out_dps[1] = gx * m[3] + gy * m[4] + gz * m[5];
    gyro_out_dps[2] = gx * m[6] + gy * m[7] + gz * m[8];

    // 2. Вычитание смещений — ПОСЛЕ поворота, потому что они и хранятся в
    //    повёрнутой системе координат (bldc/imu/imu.c). Сделать наоборот —
    //    значит вычитать не из тех осей.
    for (int i = 0; i < 3; ++i) {
        accel_out_g[i] -= p->accel_offset_g[i];
        gyro_out_dps[i] -= p->gyro_offset_dps[i];
    }
}

void fc_imu_calibration_apply(
    const FcImuCalibration *c,
    const float accel_in_g[3],
    const float gyro_in_dps[3],
    float accel_out_g[3],
    float gyro_out_dps[3]
) {
    // Удобная обёртка: строит матрицу и применяет. В контуре НЕ используется —
    // там работает подготовленный вариант.
    FcImuCalibrationPrepared p;
    fc_imu_calibration_prepare(c, &p);
    fc_imu_calibration_apply_prepared(&p, accel_in_g, gyro_in_dps, accel_out_g, gyro_out_dps);
}

bool fc_imu_calibration_plausible(const FcImuCalibration *c) {
    const float rots[3] = {c->rot_roll_deg, c->rot_pitch_deg, c->rot_yaw_deg};
    for (int i = 0; i < 3; ++i) {
        if (!isfinite(rots[i]) || fabsf(rots[i]) > FC_IMU_CAL_MAX_ROT_DEG) {
            return false;
        }
        if (!isfinite(c->gyro_offset_dps[i]) ||
            fabsf(c->gyro_offset_dps[i]) > FC_IMU_CAL_MAX_GYRO_OFFSET_DPS) {
            return false;
        }
        if (!isfinite(c->accel_offset_g[i]) ||
            fabsf(c->accel_offset_g[i]) > FC_IMU_CAL_MAX_ACCEL_OFFSET_G) {
            return false;
        }
    }
    return true;
}

// --------------------------------------------------------------- хранение

// Раскладка слов:
//   0  магия
//   1  версия
//   2  rot_roll      (float, побитово)
//   3  rot_pitch
//   4  rot_yaw
//   5  gyro_offset_x
//   6  gyro_offset_y
//   7  gyro_offset_z
//   8  accel_offset_x
//   9  accel_offset_y
//  10  accel_offset_z
//  11  контрольная сумма слов 0..10

static uint32_t f2u(float f) {
    uint32_t u;
    memcpy(&u, &f, sizeof(u));
    return u;
}

static float u2f(uint32_t u) {
    float f;
    memcpy(&f, &u, sizeof(f));
    return f;
}

// Простая аддитивно-мультипликативная сумма. Криптографии здесь не нужно:
// задача — отличить осмысленную запись от стёртого носителя и от мусора,
// а не защититься от подделки.
static uint32_t checksum(const uint32_t *w, int n) {
    uint32_t s = 0x9E3779B9u;
    for (int i = 0; i < n; ++i) {
        s = s * 31u + w[i];
    }
    return s;
}

void fc_imu_calibration_serialize(const FcImuCalibration *c, uint32_t words[FC_IMU_CAL_WORDS]) {
    words[0] = FC_IMU_CAL_MAGIC;
    words[1] = FC_IMU_CAL_VERSION;
    words[2] = f2u(c->rot_roll_deg);
    words[3] = f2u(c->rot_pitch_deg);
    words[4] = f2u(c->rot_yaw_deg);
    words[5] = f2u(c->gyro_offset_dps[0]);
    words[6] = f2u(c->gyro_offset_dps[1]);
    words[7] = f2u(c->gyro_offset_dps[2]);
    words[8] = f2u(c->accel_offset_g[0]);
    words[9] = f2u(c->accel_offset_g[1]);
    words[10] = f2u(c->accel_offset_g[2]);
    words[11] = checksum(words, 11);
}

FcImuCalStatus fc_imu_calibration_deserialize(
    const uint32_t words[FC_IMU_CAL_WORDS], FcImuCalibration *out
) {
    *out = fc_imu_calibration_identity();

    // Стёртый носитель. Именно так выглядит NVS до первой записи и EEPROM на
    // VESC: это не ошибка, а «калибровки ещё нет».
    bool erased = true;
    for (int i = 0; i < FC_IMU_CAL_WORDS; ++i) {
        if (words[i] != 0xFFFFFFFFu) {
            erased = false;
            break;
        }
    }
    if (erased || words[0] == 0) {
        return FC_IMU_CAL_NOT_CALIBRATED;
    }

    if (words[0] != FC_IMU_CAL_MAGIC) {
        return FC_IMU_CAL_INVALID;
    }
    if (words[1] != FC_IMU_CAL_VERSION) {
        // Чужая версия формата. Интерпретировать её по своей раскладке —
        // худшее, что можно сделать: числа получатся правдоподобными и
        // неверными.
        return FC_IMU_CAL_INVALID;
    }
    if (words[11] != checksum(words, 11)) {
        return FC_IMU_CAL_INVALID;
    }

    FcImuCalibration c;
    memset(&c, 0, sizeof(c));
    c.rot_roll_deg = u2f(words[2]);
    c.rot_pitch_deg = u2f(words[3]);
    c.rot_yaw_deg = u2f(words[4]);
    c.gyro_offset_dps[0] = u2f(words[5]);
    c.gyro_offset_dps[1] = u2f(words[6]);
    c.gyro_offset_dps[2] = u2f(words[7]);
    c.accel_offset_g[0] = u2f(words[8]);
    c.accel_offset_g[1] = u2f(words[9]);
    c.accel_offset_g[2] = u2f(words[10]);

    if (!fc_imu_calibration_plausible(&c)) {
        return FC_IMU_CAL_INVALID;
    }

    *out = c;
    return FC_IMU_CAL_VALID;
}

const char *fc_imu_cal_status_name(FcImuCalStatus s) {
    switch (s) {
    case FC_IMU_CAL_NOT_CALIBRATED:
        return "NOT_CALIBRATED";
    case FC_IMU_CAL_VALID:
        return "VALID";
    case FC_IMU_CAL_INVALID:
        return "INVALID";
    default:
        return "?";
    }
}
