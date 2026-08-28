// Host-тесты слоя калибровки IMU (ТЗ v0.6E §27).
//
// Проверяется то, что нельзя проверить на плате глазами: точное совпадение
// матрицы поворота с upstream, порядок операций, сериализация и отказ от
// испорченных данных.

#include "../../compat/imu/fc_imu_calibration.h"
#include "../../compat/imu/fc_imu_detect.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

void fc_test_check(bool ok, const char *what);
void fc_test_note(const char *fmt, ...);
#define check fc_test_check
#define note fc_test_note

#define RAD2DEG 57.29577951308232f
#define DEG2RAD 0.017453292519943295f

// Углы в соглашении AHRS (fc_ahrs.c / bldc ahrs.c) из вектора ускорения.
static void angles_from_accel(const float a[3], float *roll_deg, float *pitch_deg) {
    float n = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    float ax = a[0] / n, ay = a[1] / n, az = a[2] / n;
    *roll_deg = -atan2f(ay, az) * RAD2DEG;
    float sp = -ax;
    if (sp < -1.0f) {
        sp = -1.0f;
    } else if (sp > 1.0f) {
        sp = 1.0f;
    }
    *pitch_deg = asinf(sp) * RAD2DEG;
}

static void test_identity(void) {
    printf("\n\033[1mЕдиничная калибровка ничего не меняет\033[0m\n");
    FcImuCalibration c = fc_imu_calibration_identity();
    check(fc_imu_calibration_is_identity(&c), "распознаётся как единичная");

    float m[9];
    fc_imu_calibration_matrix(&c, m);
    const float eye[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    bool ok = true;
    for (int i = 0; i < 9; ++i) {
        if (fabsf(m[i] - eye[i]) > 1e-6f) {
            ok = false;
        }
    }
    check(ok, "матрица поворота единичная");

    float a[3] = {0.1f, -0.2f, 0.97f}, g[3] = {1.0f, 2.0f, -3.0f};
    float ao[3], go[3];
    fc_imu_calibration_apply(&c, a, g, ao, go);
    check(fabsf(ao[0] - a[0]) < 1e-6f && fabsf(ao[2] - a[2]) < 1e-6f, "ускорение не изменилось");
    check(fabsf(go[1] - g[1]) < 1e-6f, "угловая скорость не изменилась");
}

static void test_pure_rotations(void) {
    printf("\n\033[1mЧистые повороты по каждой оси\033[0m\n");

    // Поворот только по крену на 90°: ось Y уходит в Z.
    FcImuCalibration c = fc_imu_calibration_identity();
    c.rot_roll_deg = 90.0f;
    float in[3] = {0.0f, 1.0f, 0.0f};
    float zero[3] = {0, 0, 0};
    float out[3], gout[3];
    fc_imu_calibration_apply(&c, in, zero, out, gout);
    check(fabsf(out[0]) < 1e-5f && fabsf(out[1]) < 1e-5f && fabsf(out[2] - 1.0f) < 1e-5f,
          "roll 90°: (0,1,0) -> (0,0,1)");
    note("получено (%+.3f %+.3f %+.3f)", (double) out[0], (double) out[1], (double) out[2]);

    // Поворот только по тангажу на 90°: ось Z уходит в X.
    c = fc_imu_calibration_identity();
    c.rot_pitch_deg = 90.0f;
    float in2[3] = {0.0f, 0.0f, 1.0f};
    fc_imu_calibration_apply(&c, in2, zero, out, gout);
    check(fabsf(out[0] - 1.0f) < 1e-5f && fabsf(out[2]) < 1e-5f, "pitch 90°: (0,0,1) -> (1,0,0)");
    note("получено (%+.3f %+.3f %+.3f)", (double) out[0], (double) out[1], (double) out[2]);

    // Поворот только по рысканью на 90°: ось X уходит в Y.
    c = fc_imu_calibration_identity();
    c.rot_yaw_deg = 90.0f;
    float in3[3] = {1.0f, 0.0f, 0.0f};
    fc_imu_calibration_apply(&c, in3, zero, out, gout);
    check(fabsf(out[1] - 1.0f) < 1e-5f && fabsf(out[0]) < 1e-5f, "yaw 90°: (1,0,0) -> (0,1,0)");
    note("получено (%+.3f %+.3f %+.3f)", (double) out[0], (double) out[1], (double) out[2]);
}

static void test_rotation_preserves_magnitude(void) {
    printf("\n\033[1mПоворот сохраняет модули: проверки правдоподобия не ломаются\033[0m\n");
    FcImuCalibration c = fc_imu_calibration_identity();
    c.rot_roll_deg = 17.0f;
    c.rot_pitch_deg = -23.0f;
    c.rot_yaw_deg = 61.0f;
    float a[3] = {0.13f, -0.29f, 0.95f};
    float g[3] = {3.0f, -7.0f, 11.0f};
    float ao[3], go[3];
    fc_imu_calibration_apply(&c, a, g, ao, go);
    float ma = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    float mo = sqrtf(ao[0] * ao[0] + ao[1] * ao[1] + ao[2] * ao[2]);
    check(fabsf(ma - mo) < 1e-5f, "модуль ускорения сохранён");
    float mg = sqrtf(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
    float mgo = sqrtf(go[0] * go[0] + go[1] * go[1] + go[2] * go[2]);
    check(fabsf(mg - mgo) < 1e-4f, "модуль угловой скорости сохранён");
}

static void test_order_of_operations(void) {
    printf("\n\033[1mПорядок: сначала поворот, потом вычитание смещений\033[0m\n");
    // Поворот на 90° по рысканью переводит X в Y. Смещение задано по Y.
    // Если порядок верен, вычитание попадёт в повёрнутую ось.
    FcImuCalibration c = fc_imu_calibration_identity();
    c.rot_yaw_deg = 90.0f;
    c.gyro_offset_dps[1] = 5.0f;
    float g[3] = {5.0f, 0.0f, 0.0f};  // после поворота станет (0, 5, 0)
    float a[3] = {0, 0, 1};
    float ao[3], go[3];
    fc_imu_calibration_apply(&c, a, g, ao, go);
    check(fabsf(go[1]) < 1e-4f, "смещение вычтено ПОСЛЕ поворота (ось Y обнулилась)");
    note("gyro после калибровки (%+.3f %+.3f %+.3f)", (double) go[0], (double) go[1],
         (double) go[2]);

    // Обратный порядок дал бы go[1] = 5: проверяем, что это не так.
    check(fabsf(go[1] - 5.0f) > 1.0f, "обратный порядок исключён");
}

static void test_in_place(void) {
    printf("\n\033[1mВход и выход могут быть одним массивом\033[0m\n");
    FcImuCalibration c = fc_imu_calibration_identity();
    c.rot_pitch_deg = 30.0f;
    float a[3] = {0.0f, 0.0f, 1.0f};
    float g[3] = {1.0f, 2.0f, 3.0f};
    float expect_a[3], expect_g[3];
    fc_imu_calibration_apply(&c, a, g, expect_a, expect_g);
    fc_imu_calibration_apply(&c, a, g, a, g);
    check(fabsf(a[0] - expect_a[0]) < 1e-6f && fabsf(a[2] - expect_a[2]) < 1e-6f,
          "результат совпадает с раздельными буферами");
}

static void test_plausibility(void) {
    printf("\n\033[1mПроверка правдоподобия\033[0m\n");
    FcImuCalibration c = fc_imu_calibration_identity();
    check(fc_imu_calibration_plausible(&c), "единичная правдоподобна");
    c.rot_pitch_deg = 200.0f;
    check(!fc_imu_calibration_plausible(&c), "поворот 200° отвергнут");
    c = fc_imu_calibration_identity();
    c.gyro_offset_dps[2] = 50.0f;
    check(!fc_imu_calibration_plausible(&c), "смещение гироскопа 50 °/с отвергнуто");
    c = fc_imu_calibration_identity();
    c.rot_roll_deg = NAN;
    check(!fc_imu_calibration_plausible(&c), "NaN отвергнут");
}

static void test_serialization(void) {
    printf("\n\033[1mСериализация и разбор\033[0m\n");
    FcImuCalibration c = fc_imu_calibration_identity();
    c.rot_roll_deg = -1.234f;
    c.rot_pitch_deg = 6.789f;
    c.rot_yaw_deg = 0.0f;
    c.gyro_offset_dps[0] = 1.06f;
    c.gyro_offset_dps[1] = 0.54f;
    c.gyro_offset_dps[2] = -0.28f;

    uint32_t w[FC_IMU_CAL_WORDS];
    fc_imu_calibration_serialize(&c, w);
    FcImuCalibration back;
    check(fc_imu_calibration_deserialize(w, &back) == FC_IMU_CAL_VALID, "разбор успешен");
    check(fabsf(back.rot_pitch_deg - c.rot_pitch_deg) < 1e-6f, "тангаж восстановлен точно");
    check(fabsf(back.gyro_offset_dps[0] - c.gyro_offset_dps[0]) < 1e-6f, "смещение восстановлено");

    printf("\n\033[1mОтказ от испорченного хранилища\033[0m\n");
    uint32_t erased[FC_IMU_CAL_WORDS];
    memset(erased, 0xFF, sizeof(erased));
    check(fc_imu_calibration_deserialize(erased, &back) == FC_IMU_CAL_NOT_CALIBRATED,
          "стёртый носитель — это NOT_CALIBRATED, а не ошибка");
    check(fc_imu_calibration_is_identity(&back), "и отдаётся единичная калибровка");

    fc_imu_calibration_serialize(&c, w);
    w[5] ^= 0x1u;  // порча одного бита в данных
    check(fc_imu_calibration_deserialize(w, &back) == FC_IMU_CAL_INVALID,
          "порча одного бита ловится контрольной суммой");
    check(fc_imu_calibration_is_identity(&back), "при INVALID наружу уходит единичная");

    fc_imu_calibration_serialize(&c, w);
    w[1] = FC_IMU_CAL_VERSION + 1u;
    w[11] = 0;  // сумма всё равно не сойдётся, но версия проверяется раньше
    check(fc_imu_calibration_deserialize(w, &back) == FC_IMU_CAL_INVALID,
          "чужая версия формата отвергается");

    fc_imu_calibration_serialize(&c, w);
    w[0] = 0xDEADBEEFu;
    check(fc_imu_calibration_deserialize(w, &back) == FC_IMU_CAL_INVALID, "чужая магия отвергается");

    // Правдоподобие проверяется и после успешного разбора.
    FcImuCalibration bad = fc_imu_calibration_identity();
    bad.gyro_offset_dps[0] = 100.0f;
    fc_imu_calibration_serialize(&bad, w);
    check(fc_imu_calibration_deserialize(w, &back) == FC_IMU_CAL_INVALID,
          "неправдоподобные значения отвергаются даже с верной суммой");
}

// --- Detect ------------------------------------------------------------------

static FcDetectState feed_still(int n, const float a[3], const float g[3], uint64_t *t) {
    FcDetectState st = FC_DETECT_COLLECTING;
    for (int i = 0; i < n; ++i) {
        *t += 2000;
        // Шум порядка реального: без него дисперсия нулевая, и тест проверял
        // бы вырожденный случай.
        float j = ((i % 7) - 3) * 0.0002f;
        float aa[3] = {a[0] + j, a[1], a[2]};
        float gg[3] = {g[0] + j * 10.0f, g[1], g[2]};
        st = fc_imu_detect_feed(aa, gg, *t);
    }
    return st;
}

static void test_detect_stationary(void) {
    printf("\n\033[1mDetect на неподвижной доске\033[0m\n");
    FcDetectConfig cfg = fc_imu_detect_default_config();
    cfg.samples = 500;
    fc_imu_detect_start(&cfg, 0.0f);

    // Доска наклонена: нос вверх примерно на 6.5°, лёгкий крен.
    // acc_x = -sin(6.5°) = -0.1132, acc_y = +0.03, остальное в Z.
    float a[3] = {-0.1132f, 0.0300f, 0.9930f};
    float g[3] = {1.06f, 0.54f, -0.28f};
    uint64_t t = 1000000;
    FcDetectState st = feed_still(600, a, g, &t);
    check(st == FC_DETECT_DONE, "измерение завершилось");

    FcDetectStatus ds = fc_imu_detect_status();
    FcImuCalibration c = ds.result;
    note("rot_roll %+.3f, rot_pitch %+.3f, rot_yaw %+.3f град", (double) c.rot_roll_deg,
         (double) c.rot_pitch_deg, (double) c.rot_yaw_deg);
    note("смещения %+.3f %+.3f %+.3f °/с", (double) c.gyro_offset_dps[0],
         (double) c.gyro_offset_dps[1], (double) c.gyro_offset_dps[2]);

    // ГЛАВНАЯ проверка: применив найденную калибровку к тому же вектору,
    // обязаны получить нулевые углы. Именно ради этого и нужны знаки из
    // upstream — здесь они проверяются, а не принимаются на веру.
    float ao[3], go[3];
    fc_imu_calibration_apply(&c, a, g, ao, go);
    float roll_deg, pitch_deg;
    angles_from_accel(ao, &roll_deg, &pitch_deg);
    check(fabsf(pitch_deg) < 0.05f, "после калибровки тангаж = 0");
    check(fabsf(roll_deg) < 0.05f, "после калибровки крен = 0");
    note("углы после калибровки: pitch %+.4f, roll %+.4f град", (double) pitch_deg,
         (double) roll_deg);

    // И смещение гироскопа обязано обнулиться.
    check(fabsf(go[0]) < 0.02f && fabsf(go[1]) < 0.02f && fabsf(go[2]) < 0.02f,
          "после калибровки угловая скорость в покое = 0");
    note("gyro после калибровки %+.4f %+.4f %+.4f °/с", (double) go[0], (double) go[1],
         (double) go[2]);
}

static void test_detect_rejects_motion(void) {
    printf("\n\033[1mDetect отвергает движущуюся доску\033[0m\n");
    FcDetectConfig cfg = fc_imu_detect_default_config();
    cfg.samples = 500;
    fc_imu_detect_start(&cfg, 0.0f);
    uint64_t t = 1000000;
    float a[3] = {0.0f, 0.0f, 1.0f};
    for (int i = 0; i < 2000; ++i) {
        t += 2000;
        float g[3] = {(i % 200 == 0) ? 40.0f : 0.5f, 0.0f, 0.0f};
        fc_imu_detect_feed(a, g, t);
    }
    FcDetectStatus ds = fc_imu_detect_status();
    check(ds.state == FC_DETECT_COLLECTING, "не завершилось");
    check(ds.restarts > 0, "накопление сбрасывалось");
    note("причина: %s, сбросов %u", fc_imu_detect_failure_name(ds.failure),
         (unsigned) ds.restarts);

    printf("\n\033[1mDetect отвергает медленное покачивание\033[0m\n");
    // Каждый отдельный семпл ниже мгновенного порога, но дисперсия велика:
    // именно этот случай мгновенный порог пропускает, а СКО ловит.
    fc_imu_detect_start(&cfg, 0.0f);
    t = 1000000;
    for (int i = 0; i < 2000; ++i) {
        t += 2000;
        float sway = (i % 2 == 0) ? 2.0f : -2.0f;
        float g[3] = {sway, 0.0f, 0.0f};
        fc_imu_detect_feed(a, g, t);
    }
    ds = fc_imu_detect_status();
    check(ds.state != FC_DETECT_DONE, "покачивание в пределах мгновенного порога отвергнуто");
    note("причина: %s", fc_imu_detect_failure_name(ds.failure));

    printf("\n\033[1mDetect отвергает неверный модуль ускорения\033[0m\n");
    fc_imu_detect_start(&cfg, 0.0f);
    t = 1000000;
    float a2[3] = {0.0f, 0.0f, 0.5f};  // 0.5 g — доску несут
    float g0[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 1000; ++i) {
        t += 2000;
        fc_imu_detect_feed(a2, g0, t);
    }
    ds = fc_imu_detect_status();
    check(ds.state == FC_DETECT_COLLECTING, "не завершилось");
    check(ds.failure == FC_DETECT_FAIL_ACCEL_MAG, "причина названа верно");

    printf("\n\033[1mDetect отвергает рваный поток семплов\033[0m\n");
    fc_imu_detect_start(&cfg, 0.0f);
    t = 1000000;
    for (int i = 0; i < 1000; ++i) {
        t += (i % 50 == 0) ? 50000 : 2000;  // разрыв 50 мс
        fc_imu_detect_feed(a, g0, t);
    }
    ds = fc_imu_detect_status();
    check(ds.state == FC_DETECT_COLLECTING, "не завершилось");
    check(ds.restarts > 0, "разрыв потока сбрасывает накопление");
}

static void test_detect_yaw(void) {
    printf("\n\033[1mРысканье задаётся пользователем, а не измеряется\033[0m\n");
    FcDetectConfig cfg = fc_imu_detect_default_config();
    cfg.samples = 300;
    fc_imu_detect_start(&cfg, 45.0f);
    float a[3] = {0.0f, 0.0f, 1.0f};
    float g[3] = {0.0f, 0.0f, 0.0f};
    uint64_t t = 1000000;
    feed_still(400, a, g, &t);
    FcDetectStatus ds = fc_imu_detect_status();
    check(ds.state == FC_DETECT_DONE, "измерение завершилось");
    check(fabsf(ds.result.rot_yaw_deg - 45.0f) < 1e-5f, "rot_yaw взят из аргумента");
}

void test_imu_calibration_all(void) {
    printf("\n\033[1mТесты слоя калибровки IMU: модель, поворот, хранение, Detect\033[0m\n");
    test_identity();
    test_pure_rotations();
    test_rotation_preserves_magnitude();
    test_order_of_operations();
    test_in_place();
    test_plausibility();
    test_serialization();
    test_detect_stationary();
    test_detect_rejects_motion();
    test_detect_yaw();
}
