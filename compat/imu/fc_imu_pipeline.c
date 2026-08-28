#include "fc_imu_pipeline.h"

#include <math.h>
#include <string.h>

#define DEG2RAD 0.017453292519943295f

static struct {
    FcImuPipelineConfig cfg;
    FcImuPipelineStats st;
    FcAhrs ahrs;

    int16_t prev_words[7];
    bool have_prev_words;

    uint64_t prev_accept_us;
    bool have_prev_accept;

    FcImuCalibration cal;
    FcImuCalibrationPrepared cal_prepared;
    float residual_sum[3];

    FcImuSample sample;
    bool have_sample;
} P;

FcImuPipelineConfig fc_imu_pipeline_default_config(uint32_t nominal_period_us) {
    FcImuPipelineConfig c;
    c.nominal_period_us = nominal_period_us;
    // Пять подряд одинаковых семплов — тот же порог, что у fc_imu_health для
    // залипания. Значение согласовано намеренно: два разных порога для одного
    // физического явления неизбежно разошлись бы.
    c.stuck_duplicate_limit = 5;
    c.residual_samples = 500;
    c.residual_still_gyro_dps = 3.0f;
    c.residual_still_accel_tol_g = 0.1f;
    // Порог остатка. Постоянная калибровка должна была свести смещение к
    // сотым долям; остаток больше 2 °/с означает, что она не соответствует
    // текущему состоянию датчика. Это не отказ, но это обязано быть видно.
    c.residual_max_bias_dps = 2.0f;
    return c;
}

void fc_imu_pipeline_init(const FcImuPipelineConfig *cfg, const FcImuHealthConfig *health) {
    memset(&P, 0, sizeof(P));
    P.cfg = *cfg;
    P.st.min_gap_us = UINT64_MAX;
    P.cal = fc_imu_calibration_identity();
    fc_imu_calibration_prepare(&P.cal, &P.cal_prepared);
    fc_ahrs_init(&P.ahrs);
    fc_imu_health_init(health);
}

// Сравниваются только шесть слов движения, БЕЗ температуры.
//
// Это не оптимизация, а исправление. Первая версия сравнивала все семь слов,
// включая температуру, — казалось, что лишняя энтропия только помогает. На
// живой плате это дало ~1000 принятых семплов в секунду вместо 562.5:
// температурный канал ICM-20948 обновляется на собственной частоте 1.125 кГц
// и не подчиняется SMPLRT_DIV (DS-000189 §8.2), поэтому почти каждый опрос
// выглядел «новым», хотя оси не менялись. Контур при этом обрабатывал одни и
// те же данные движения дважды.
//
// Энтропии шести осей достаточно: на покоящейся доске шум акселерометра
// составляет десятки младших разрядов (измерено: sd 0.003 g при 8192 LSB/g,
// то есть около 25 LSB), и совпадение всех шести слов подряд у живого датчика
// означает не совпадение, а остановку выдачи.
#define FC_IMU_MOTION_WORDS 6

static bool same_motion(const int16_t a[7], const int16_t b[7]) {
    for (int i = 0; i < FC_IMU_MOTION_WORDS; ++i) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}

FcImuPipeVerdict fc_imu_pipeline_submit(
    bool read_ok,
    const int16_t raw_words[7],
    const float accel_g[3],
    const float gyro_dps[3],
    float temperature_c,
    uint64_t now_us
) {
    ++P.st.polls;

    if (!read_ok) {
        ++P.st.read_failures;
        // Диагностика обязана узнать о неудачной транзакции немедленно:
        // именно по ней супервизор поднимает отказ.
        FcImuRawSample empty;
        memset(&empty, 0, sizeof(empty));
        fc_imu_health_update(false, &empty, now_us);
        // Состояние банка/шины после ошибки неизвестно, поэтому и сравнение
        // с предыдущими словами больше не имеет смысла.
        P.have_prev_words = false;
        return FC_IMU_PIPE_READ_FAILED;
    }

    // --- дедупликация -------------------------------------------------------
    //
    // Роль этой проверки уточнена измерением. Регистры данных ICM-20948
    // обновляются на внутренней частоте 1.125 кГц независимо от SMPLRT_DIV
    // (измерено: 1126 Гц). Контур работает на 500 Гц, то есть вдвое реже,
    // поэтому каждое чтение и так возвращает новые данные, а дубликат в
    // норме не встречается вовсе.
    //
    // Поэтому здесь не «источник ритма», а ДЕТЕКТОР ЗАМИРАНИЯ: если слова
    // движения не изменились, значит датчик перестал обновлять регистры, и
    // такой ответ не является новым измерением. Одиночное совпадение
    // допускается (оно возможно при экстремально тихой обстановке), серия —
    // нет.
    if (P.have_prev_words && same_motion(raw_words, P.prev_words)) {
        ++P.st.duplicates;
        ++P.st.consecutive_duplicates;
        if (P.st.consecutive_duplicates > P.st.max_consecutive_duplicates) {
            P.st.max_consecutive_duplicates = P.st.consecutive_duplicates;
        }
        // Длинная серия дубликатов — это уже не «опрашиваем чаще выдачи», а
        // замерший датчик. Отдаём это диагностике как отказ чтения: сам по
        // себе повтор не является валидным семплом.
        if (P.st.consecutive_duplicates >= P.cfg.stuck_duplicate_limit) {
            FcImuRawSample empty;
            memset(&empty, 0, sizeof(empty));
            fc_imu_health_update(false, &empty, now_us);
        }
        return FC_IMU_PIPE_DUPLICATE;
    }

    memcpy(P.prev_words, raw_words, sizeof(P.prev_words));
    P.have_prev_words = true;
    P.st.consecutive_duplicates = 0;

    // --- калибровка ---------------------------------------------------------
    //
    // Порядок взят из bldc/imu/imu.c и обязателен: сначала поворот, затем
    // вычитание смещений (они и хранятся в повёрнутой системе координат).
    // Применяется ДО валидации и до всего остального, ровно как в прошивке
    // VESC: и AHRS, и callback пакета обязаны видеть один и тот же
    // исправленный кадр.
    //
    // Проверкам правдоподобия это не мешает: поворот не меняет ни модуль
    // вектора ускорения, ни модуль угловой скорости, а именно их и проверяет
    // fc_imu_health.
    float accel_cal[3], gyro_cal[3];
    fc_imu_calibration_apply_prepared(&P.cal_prepared, accel_g, gyro_dps, accel_cal, gyro_cal);

    // Процедура Detect работает по СЫРЫМ данным: она вычисляет калибровку,
    // которая их исправит, а не поправку к уже исправленным.
    fc_imu_detect_feed(accel_g, gyro_dps, now_us);

    // --- валидация ----------------------------------------------------------
    FcImuRawSample raw;
    memset(&raw, 0, sizeof(raw));
    memcpy(raw.accel_g, accel_cal, sizeof(raw.accel_g));
    memcpy(raw.gyro_dps, gyro_cal, sizeof(raw.gyro_dps));
    raw.temperature_c = temperature_c;
    raw.timestamp_us = now_us;
    raw.sample_counter = P.sample.sample_counter + 1;
    raw.valid = true;

    FcImuHealthState hs = fc_imu_health_update(true, &raw, now_us);
    if (hs != FC_IMU_OK) {
        ++P.st.rejected;
        return FC_IMU_PIPE_REJECTED;
    }

    // --- принят -------------------------------------------------------------
    float dt = 0.0f;
    if (P.have_prev_accept && now_us > P.prev_accept_us) {
        uint64_t gap = now_us - P.prev_accept_us;
        dt = (float) gap * 1e-6f;
        if (gap > P.st.max_gap_us) {
            P.st.max_gap_us = gap;
        }
        if (gap < P.st.min_gap_us) {
            P.st.min_gap_us = gap;
        }
        P.st.sum_gap_us += gap;
        ++P.st.gaps_counted;
        // Интервал в полтора номинала и больше означает, что между двумя
        // опросами датчик успел выдать два семпла, и один прошёл мимо.
        // Это не отказ, но это обязано быть видно в статистике.
        if (gap > (uint64_t) P.cfg.nominal_period_us * 3 / 2) {
            ++P.st.suspected_skips;
        }
    } else {
        dt = (float) P.cfg.nominal_period_us * 1e-6f;
    }
    P.prev_accept_us = now_us;
    P.have_prev_accept = true;

    // --- измерение остаточного смещения гироскопа ---------------------------
    //
    // Считается по КАЛИБРОВАННОМУ гироскопу и НЕ вычитается ни из чего.
    // Вычитание здесь означало бы вторую компенсацию поверх постоянных
    // смещений — ровно то, что ТЗ v0.6E §10 запрещает.
    if (P.st.residual_state == FC_IMU_RESIDUAL_COLLECTING) {
        float amag = sqrtf(accel_cal[0] * accel_cal[0] + accel_cal[1] * accel_cal[1] +
                           accel_cal[2] * accel_cal[2]);
        bool still = fabsf(amag - 1.0f) <= P.cfg.residual_still_accel_tol_g;
        for (int i = 0; i < 3 && still; ++i) {
            if (fabsf(gyro_cal[i]) > P.cfg.residual_still_gyro_dps) {
                still = false;
            }
        }
        if (!still) {
            if (P.st.residual_samples) {
                ++P.st.residual_restarts;
            }
            P.st.residual_samples = 0;
            P.residual_sum[0] = P.residual_sum[1] = P.residual_sum[2] = 0.0f;
        } else {
            for (int i = 0; i < 3; ++i) {
                P.residual_sum[i] += gyro_cal[i];
            }
            if (++P.st.residual_samples >= P.cfg.residual_samples) {
                float n = (float) P.st.residual_samples;
                bool ok = true;
                for (int i = 0; i < 3; ++i) {
                    P.st.residual_bias_dps[i] = P.residual_sum[i] / n;
                    if (fabsf(P.st.residual_bias_dps[i]) > P.cfg.residual_max_bias_dps) {
                        ok = false;
                    }
                }
                P.st.residual_state = ok ? FC_IMU_RESIDUAL_DONE : FC_IMU_RESIDUAL_REJECTED;
            }
        }
    }

    float gyro_rad[3];
    for (int i = 0; i < 3; ++i) {
        gyro_rad[i] = gyro_cal[i] * DEG2RAD;
    }

    // Первый принятый семпл задаёт начальную ориентацию: иначе фильтр
    // несколько секунд «доворачивал» бы доску из единичного кватерниона, и
    // всё это время Refloat видел бы неверный тангаж.
    if (!P.ahrs.initialised) {
        fc_ahrs_set_from_accel(&P.ahrs, accel_cal);
    } else {
        fc_ahrs_update(&P.ahrs, gyro_rad, accel_cal, dt);
    }

    FcImuSample s;
    memset(&s, 0, sizeof(s));
    s.timestamp_us = now_us;
    s.sample_counter = raw.sample_counter;
    s.dt_s = dt;
    memcpy(s.accel_g, accel_cal, sizeof(s.accel_g));
    memcpy(s.gyro_rad_s, gyro_rad, sizeof(s.gyro_rad_s));
    memcpy(s.gyro_dps, gyro_cal, sizeof(s.gyro_dps));
    memcpy(s.accel_raw_g, accel_g, sizeof(s.accel_raw_g));
    memcpy(s.gyro_raw_dps, gyro_dps, sizeof(s.gyro_raw_dps));
    s.temperature_c = temperature_c;
    fc_ahrs_quaternion(&P.ahrs, s.quat);
    s.roll_rad = fc_ahrs_roll(&P.ahrs);
    s.pitch_rad = fc_ahrs_pitch(&P.ahrs);
    s.yaw_rad = fc_ahrs_yaw(&P.ahrs);
    s.valid = true;
    s.health = hs;

    P.sample = s;
    P.have_sample = true;
    ++P.st.accepted;
    return FC_IMU_PIPE_ACCEPTED;
}

FcImuSample fc_imu_pipeline_sample(void) {
    return P.sample;
}

bool fc_imu_pipeline_has_sample(void) {
    return P.have_sample;
}

FcImuPipelineStats fc_imu_pipeline_stats(void) {
    return P.st;
}

const FcAhrs *fc_imu_pipeline_ahrs(void) {
    return &P.ahrs;
}

bool fc_imu_pipeline_residual_ready(void) {
    return P.st.residual_state != FC_IMU_RESIDUAL_COLLECTING;
}

void fc_imu_pipeline_set_calibration(const FcImuCalibration *c) {
    P.cal = *c;
    // Матрица считается здесь и только здесь: в контуре она уже готова.
    fc_imu_calibration_prepare(&P.cal, &P.cal_prepared);
}

FcImuCalibration fc_imu_pipeline_calibration(void) {
    return P.cal;
}

void fc_imu_pipeline_reset_ahrs(void) {
    // После смены калибровки прежняя оценка ориентации относится к другому
    // кадру. Сходиться из неё фильтр будет минуты (kp = 0.2), поэтому он
    // перезапускается и берёт начальную ориентацию из первого же семпла.
    fc_ahrs_init(&P.ahrs);
    P.st.residual_state = FC_IMU_RESIDUAL_COLLECTING;
    P.st.residual_samples = 0;
    P.residual_sum[0] = P.residual_sum[1] = P.residual_sum[2] = 0.0f;
}

const char *fc_imu_residual_state_name(FcImuResidualState s) {
    switch (s) {
    case FC_IMU_RESIDUAL_COLLECTING:
        return "идёт (доска должна быть неподвижна)";
    case FC_IMU_RESIDUAL_DONE:
        return "измерен";
    case FC_IMU_RESIDUAL_REJECTED:
        return "ВЕЛИК: калибровка не соответствует состоянию датчика";
    default:
        return "?";
    }
}

const char *fc_imu_pipe_verdict_name(FcImuPipeVerdict v) {
    switch (v) {
    case FC_IMU_PIPE_READ_FAILED:
        return "READ_FAILED";
    case FC_IMU_PIPE_DUPLICATE:
        return "DUPLICATE";
    case FC_IMU_PIPE_REJECTED:
        return "REJECTED";
    case FC_IMU_PIPE_ACCEPTED:
        return "ACCEPTED";
    default:
        return "?";
    }
}
