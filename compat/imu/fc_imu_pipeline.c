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

    float cal_sum[3];

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
    c.cal_samples = 500;
    c.cal_still_gyro_dps = 3.0f;
    c.cal_still_accel_tol_g = 0.1f;
    c.cal_max_bias_dps = 5.0f;
    return c;
}

void fc_imu_pipeline_init(const FcImuPipelineConfig *cfg, const FcImuHealthConfig *health) {
    memset(&P, 0, sizeof(P));
    P.cfg = *cfg;
    P.st.min_gap_us = UINT64_MAX;
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

    // --- валидация ----------------------------------------------------------
    FcImuRawSample raw;
    memset(&raw, 0, sizeof(raw));
    memcpy(raw.accel_g, accel_g, sizeof(raw.accel_g));
    memcpy(raw.gyro_dps, gyro_dps, sizeof(raw.gyro_dps));
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

    // --- стартовая калибровка смещения гироскопа ---------------------------
    //
    // Идёт только пока доска неподвижна. Любое движение сбрасывает накопление:
    // калибровка на движущейся доске записала бы реальное вращение как
    // смещение и внесла бы постоянную ошибку в контур.
    if (P.st.cal_state == FC_IMU_CAL_COLLECTING) {
        float amag = sqrtf(accel_g[0] * accel_g[0] + accel_g[1] * accel_g[1] +
                           accel_g[2] * accel_g[2]);
        bool still = fabsf(amag - 1.0f) <= P.cfg.cal_still_accel_tol_g;
        for (int i = 0; i < 3 && still; ++i) {
            if (fabsf(gyro_dps[i]) > P.cfg.cal_still_gyro_dps) {
                still = false;
            }
        }
        if (!still) {
            if (P.st.cal_samples) {
                ++P.st.cal_restarts;
            }
            P.st.cal_samples = 0;
            P.cal_sum[0] = P.cal_sum[1] = P.cal_sum[2] = 0.0f;
        } else {
            for (int i = 0; i < 3; ++i) {
                P.cal_sum[i] += gyro_dps[i];
            }
            if (++P.st.cal_samples >= P.cfg.cal_samples) {
                float n = (float) P.st.cal_samples;
                bool ok = true;
                for (int i = 0; i < 3; ++i) {
                    float b = P.cal_sum[i] / n;
                    if (fabsf(b) > P.cfg.cal_max_bias_dps) {
                        ok = false;
                    }
                    P.st.gyro_bias_dps[i] = b;
                }
                if (!ok) {
                    // Неправдоподобное смещение не применяется: лучше жить с
                    // известной ошибкой, чем вычесть выдуманную величину.
                    P.st.gyro_bias_dps[0] = 0.0f;
                    P.st.gyro_bias_dps[1] = 0.0f;
                    P.st.gyro_bias_dps[2] = 0.0f;
                    P.st.cal_state = FC_IMU_CAL_REJECTED;
                } else {
                    P.st.cal_state = FC_IMU_CAL_DONE;
                }
            }
        }
    }

    // Смещение вычитается ПОСЛЕ валидации и только из того, что уходит дальше:
    // диагностика обязана видеть то, что реально отдал датчик.
    float gyro_corrected[3];
    float gyro_rad[3];
    for (int i = 0; i < 3; ++i) {
        gyro_corrected[i] = gyro_dps[i] - P.st.gyro_bias_dps[i];
        gyro_rad[i] = gyro_corrected[i] * DEG2RAD;
    }

    // Первый принятый семпл задаёт начальную ориентацию: иначе фильтр
    // несколько секунд «доворачивал» бы доску из единичного кватерниона, и
    // всё это время Refloat видел бы неверный тангаж.
    if (!P.ahrs.initialised) {
        fc_ahrs_set_from_accel(&P.ahrs, accel_g);
    } else {
        fc_ahrs_update(&P.ahrs, gyro_rad, accel_g, dt);
    }

    FcImuSample s;
    memset(&s, 0, sizeof(s));
    s.timestamp_us = now_us;
    s.sample_counter = raw.sample_counter;
    s.dt_s = dt;
    memcpy(s.accel_g, accel_g, sizeof(s.accel_g));
    memcpy(s.gyro_rad_s, gyro_rad, sizeof(s.gyro_rad_s));
    memcpy(s.gyro_dps, gyro_corrected, sizeof(s.gyro_dps));
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

bool fc_imu_pipeline_calibrated(void) {
    return P.st.cal_state != FC_IMU_CAL_COLLECTING;
}

const char *fc_imu_cal_state_name(FcImuCalState s) {
    switch (s) {
    case FC_IMU_CAL_COLLECTING:
        return "идёт (доска должна быть неподвижна)";
    case FC_IMU_CAL_DONE:
        return "выполнена";
    case FC_IMU_CAL_REJECTED:
        return "ОТКЛОНЕНА: смещение неправдоподобно, не применено";
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
