// Платформенный AHRS FloatCore (ТЗ v0.6D §4, §11).
//
// Зачем он вообще нужен. Refloat берёт углы из ДВУХ независимых источников:
//
//   imu.c:36-40  pitch/roll/yaw  ← VESC_IF->imu_get_*()   — AHRS платформы
//   imu.c:41     balance_pitch   ← balance_filter_get_*() — собственный фильтр
//
// Первый идёт в отказы и quickstop (main.c:386, 422, 1016, 1040), второй — в
// PID (pid.c:59). Если эти два источника разойдутся по знаку или по нулю,
// Refloat получит противоречивые данные, и обнаружится это только на доске.
//
// Поэтому платформенный AHRS реализован как Mahony с ТОЙ ЖЕ математикой и тем
// же соглашением о знаках, что и balance_filter.c: то же уравнение оценки
// гравитации (:99-101), та же пропорциональная коррекция (:107-109), то же
// интегрирование кватерниона (:115-124) и те же формулы извлечения углов
// (:136-164), включая ведущий минус в roll и yaw. Согласованность двух
// источников тем самым обеспечена конструкцией, а не проверкой.
//
// Копией кода это не является: код написан заново по тем же формулам, чтобы
// платформенный слой не включал заголовки Refloat (запрет из fc_platform.h).
#pragma once

#include <stdbool.h>

typedef struct {
    float q0, q1, q2, q3;
    float acc_mag;
    float kp;
    float acc_confidence_decay;
    bool initialised;
} FcAhrs;

// Значения по умолчанию — те, которые Refloat САМ ЖЕ прописывает платформе.
//
// refloat-upstream/src/main.c:210-214:
//
//     if (VESC_IF->get_cfg_float(CFG_PARAM_IMU_mahony_kp) > 1) {
//         VESC_IF->set_cfg_float(CFG_PARAM_IMU_mahony_kp, 0.2);
//         VESC_IF->set_cfg_float(CFG_PARAM_IMU_mahony_ki, 0);
//         VESC_IF->set_cfg_float(CFG_PARAM_IMU_accel_confidence_decay, 0.1);
//     }
//
// Комментарий upstream объясняет смысл: если kp внутреннего фильтра прошивки
// больше единицы, это «старая схема», где он был ОСНОВНЫМ балансировочным
// фильтром; Refloat приводит его к значениям лёгкого фильтра истинного
// тангажа, потому что основным теперь является его собственный balance_filter.
//
// Отсюда два следствия, которые легко перепутать:
//
//  * коэффициенты Refloat (Pitch KP = 2.0, Roll KP = 1.4) относятся к ЕГО
//    фильтру и к платформенному AHRS отношения не имеют;
//  * платформенный AHRS обязан жить на 0.2 и затухании доверия 0.1, иначе
//    платформа сообщает Refloat одни параметры, а считает по другим.
//
// Единый kp, а не три: у внутреннего фильтра прошивки VESC он один
// (ATTITUDE_INFO в vesc_c_if.h содержит одно поле kp).
#define FC_AHRS_DEFAULT_KP 0.2f
#define FC_AHRS_DEFAULT_ACC_CONFIDENCE_DECAY 0.1f

void fc_ahrs_init(FcAhrs *a);

/** Применить параметры, о которых платформа сообщает через get_cfg_float. */
void fc_ahrs_configure(FcAhrs *a, float kp, float acc_confidence_decay);

/**
 * Начальная ориентация по одному вектору ускорения.
 *
 * Без неё фильтр стартует из единичного кватерниона и «доворачивает» доску
 * несколько секунд, а всё это время Refloat видит неверный pitch. Здесь же
 * ориентация ставится сразу в ту, которая соответствует измеренной гравитации.
 * Рыскание из акселерометра не определяется и берётся нулевым.
 */
void fc_ahrs_set_from_accel(FcAhrs *a, const float accel_g[3]);

/** Один шаг фильтра. gyro — рад/с, accel — g, dt — секунды. */
void fc_ahrs_update(FcAhrs *a, const float gyro_rad_s[3], const float accel_g[3], float dt);

// Углы в радианах, в соглашении balance_filter.c:136-164.
float fc_ahrs_roll(const FcAhrs *a);
float fc_ahrs_pitch(const FcAhrs *a);
float fc_ahrs_yaw(const FcAhrs *a);
void fc_ahrs_quaternion(const FcAhrs *a, float q[4]);
