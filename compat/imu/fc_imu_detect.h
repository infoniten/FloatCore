// Процедура Detect Calibration (ТЗ v0.6E §8, §9).
//
// Повторяет смысл штатной imu_get_calibration() прошивки VESC
// (bldc/imu/imu.c): на неподвижной доске измеряется нулевое смещение
// гироскопа, затем поворот, при котором текущее физическое положение читается
// как нулевые тангаж и крен. Рысканье не измеряется — оно задаётся
// пользователем, потому что акселерометр рысканья не наблюдает.
//
// Одно отличие от upstream, и его стоит назвать прямо. VESC измеряет крен и
// тангаж по своему AHRS, предварительно переключив его в Madgwick на 1000 Гц и
// подождав 1.5 с сходимости. Здесь углы берутся из УСРЕДНЁННОГО вектора
// ускорения по тем же формулам, что и в fc_ahrs_set_from_accel.
//
// Это не упрощение, а тождество: неподвижная точка фильтра Mahony при нулевой
// средней угловой скорости — это и есть направление, которое показывает
// акселерометр. Фильтр к нему сходится, а среднее даёт его сразу. Выигрыш —
// нет нужды подменять параметры фильтра на время процедуры и ждать десятки
// секунд (наш платформенный AHRS работает с kp = 0.2, его постоянная времени
// около пяти секунд).
//
// Модуль ничего не сохраняет и ничего не применяет: он только измеряет и
// отдаёт результат. Решение о применении и записи принимает вызывающий.
#pragma once

#include "fc_imu_calibration.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    FC_DETECT_IDLE = 0,
    FC_DETECT_COLLECTING,
    FC_DETECT_DONE,
    FC_DETECT_FAILED,
} FcDetectState;

typedef enum {
    FC_DETECT_FAIL_NONE = 0,
    FC_DETECT_FAIL_MOTION,        // доску двигали
    FC_DETECT_FAIL_ACCEL_MAG,     // модуль ускорения не похож на покой в 1 g
    FC_DETECT_FAIL_DISCONTINUITY, // поток семплов прерывался
    FC_DETECT_FAIL_IMPLAUSIBLE,   // результат не прошёл проверку правдоподобия
} FcDetectFailure;

typedef struct {
    FcDetectState state;
    FcDetectFailure failure;
    uint32_t samples;   // накоплено в текущей попытке
    uint32_t needed;    // сколько нужно
    uint32_t restarts;  // сколько раз накопление сбрасывалось

    // Метрики качества: по ним видно, была ли доска действительно неподвижна.
    float accel_mean_g[3];
    float gyro_mean_dps[3];
    float accel_std_g[3];
    float gyro_std_dps[3];
    float accel_mag_g;

    FcImuCalibration result;
} FcDetectStatus;

typedef struct {
    uint32_t samples;             // сколько семплов усреднять
    float max_gyro_std_dps;       // допустимое СКО угловой скорости
    float max_accel_std_g;        // допустимое СКО ускорения
    float max_gyro_instant_dps;   // мгновенный выброс, обрывающий накопление
    float accel_mag_tol_g;        // допуск модуля ускорения вокруг 1 g
    uint64_t max_gap_us;          // разрыв в потоке семплов
} FcDetectConfig;

FcDetectConfig fc_imu_detect_default_config(void);

/** Начать измерение. yaw_deg задаётся пользователем и не измеряется. */
void fc_imu_detect_start(const FcDetectConfig *cfg, float yaw_deg);

/** Отменить измерение. */
void fc_imu_detect_abort(void);

/**
 * Подать очередной СЫРОЙ семпл — до применения калибровки.
 *
 * Измерять надо именно сырые данные: процедура вычисляет калибровку, которая
 * их исправит, а не поправку к уже исправленным.
 */
FcDetectState fc_imu_detect_feed(
    const float accel_raw_g[3], const float gyro_raw_dps[3], uint64_t now_us
);

FcDetectStatus fc_imu_detect_status(void);
const char *fc_imu_detect_state_name(FcDetectState s);
const char *fc_imu_detect_failure_name(FcDetectFailure f);
