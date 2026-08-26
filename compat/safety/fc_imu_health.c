// Реализация диагностики сырого IMU (ТЗ v0.6A §7).

#include "fc_imu_health.h"

#include <math.h>
#include <string.h>

static struct {
    FcImuHealthConfig cfg;
    FcImuHealthStatus st;
    bool have_prev;
    FcImuRawSample prev;
    uint64_t last_update_us;
} H;

FcImuHealthConfig fc_imu_health_default_config(void) {
    FcImuHealthConfig c;
    memset(&c, 0, sizeof(c));
    // Нижняя граница модуля ускорения. Свободное падение даёт 0 g, но датчик
    // в этом состоянии на доске оказаться не должен; 0.25 g — консервативная
    // политика FloatCore, отделяющая «датчик что-то меряет» от нулей, которые
    // ICM-20948 выдаёт в режиме сна.
    c.accel_mag_min_g = 0.25f;
    // Верхняя граница: полная шкала ±2 g по каждой оси, модуль трёхосного
    // вектора при насыщении = 2*sqrt(3) = 3.46 g. Больше физически прийти
    // не может, а значит превышение — признак порчи данных.
    c.accel_mag_max_g = 3.5f;
    // Полная шкала гироскопа по умолчанию ±250 °/с (ICM-20948 DS-000189).
    c.gyro_abs_max_dps = 255.0f;
    c.stuck_limit = 5;
    c.stale_us = 40000;    // 20 периодов при 500 Гц
    c.timeout_us = 200000; // 100 периодов: связи нет
    return c;
}

FcImuHealthConfig fc_imu_health_config_for(float accel_fs_g, float gyro_fs_dps) {
    FcImuHealthConfig c = fc_imu_health_default_config();
    // Потолок модуля трёхосного вектора при насыщении всех осей: fs*sqrt(3).
    // 5 % сверху — на разброс чувствительности между экземплярами.
    c.accel_mag_max_g = accel_fs_g * 1.7320508f * 1.05f;
    c.gyro_abs_max_dps = gyro_fs_dps * 1.02f;
    return c;
}

void fc_imu_health_init(const FcImuHealthConfig *cfg) {
    memset(&H, 0, sizeof(H));
    H.cfg = cfg ? *cfg : fc_imu_health_default_config();
    H.st.state = FC_IMU_NOT_INITIALIZED;
}

static bool sample_is_finite(const FcImuRawSample *s) {
    for (int i = 0; i < 3; ++i) {
        if (!isfinite(s->accel_g[i]) || !isfinite(s->gyro_dps[i])) {
            return false;
        }
    }
    return isfinite(s->temperature_c);
}

static bool sample_is_plausible(const FcImuHealthConfig *c, const FcImuRawSample *s) {
    float mag = sqrtf(s->accel_g[0] * s->accel_g[0] + s->accel_g[1] * s->accel_g[1] +
                      s->accel_g[2] * s->accel_g[2]);
    if (mag < c->accel_mag_min_g || mag > c->accel_mag_max_g) {
        return false;
    }
    for (int i = 0; i < 3; ++i) {
        if (fabsf(s->gyro_dps[i]) > c->gyro_abs_max_dps) {
            return false;
        }
    }
    return true;
}

static bool same_raw(const FcImuRawSample *a, const FcImuRawSample *b) {
    // Побитовое сравнение измеряемых полей: у живого датчика младшие разряды
    // шумят, поэтому длинная серия точных совпадений означает зависание.
    return memcmp(a->accel_g, b->accel_g, sizeof(a->accel_g)) == 0 &&
           memcmp(a->gyro_dps, b->gyro_dps, sizeof(a->gyro_dps)) == 0;
}

FcImuHealthState fc_imu_health_update(bool ok, const FcImuRawSample *sample, uint64_t now_us) {
    H.last_update_us = now_us;

    if (!ok || !sample || !sample->valid) {
        ++H.st.read_errors;
        H.st.state = FC_IMU_READ_ERROR;
        return H.st.state;
    }

    ++H.st.samples_total;

    if (!sample_is_finite(sample)) {
        ++H.st.invalid_samples;
        H.st.state = FC_IMU_INVALID;
        return H.st.state;
    }

    // Время обязано идти вперёд: обратный ход означает порчу источника времени
    // и делает бессмысленными все проверки возраста.
    if (H.have_prev && sample->timestamp_us < H.prev.timestamp_us) {
        ++H.st.invalid_samples;
        H.st.state = FC_IMU_INVALID;
        return H.st.state;
    }

    if (!sample_is_plausible(&H.cfg, sample)) {
        ++H.st.invalid_samples;
        H.st.state = FC_IMU_INVALID;
        return H.st.state;
    }

    if (H.have_prev) {
        bool counter_moved = sample->sample_counter != H.prev.sample_counter;
        if (same_raw(sample, &H.prev) || !counter_moved) {
            ++H.st.consecutive_identical;
            if (H.st.consecutive_identical >= H.cfg.stuck_limit) {
                ++H.st.stuck_events;
                H.st.state = FC_IMU_STUCK;
                H.prev = *sample;
                return H.st.state;
            }
        } else {
            H.st.consecutive_identical = 0;
        }
    }

    H.prev = *sample;
    H.have_prev = true;
    H.st.last_counter = sample->sample_counter;
    H.st.last_good = *sample;
    H.st.last_good_us = sample->timestamp_us;
    H.st.state = FC_IMU_OK;
    return H.st.state;
}

FcImuHealthState fc_imu_health_poll(uint64_t now_us) {
    if (H.st.state == FC_IMU_NOT_INITIALIZED || H.st.last_good_us == 0) {
        return H.st.state;
    }
    uint64_t age = now_us - H.st.last_good_us;
    if (age > H.cfg.timeout_us) {
        if (H.st.state != FC_IMU_TIMEOUT) {
            ++H.st.timeout_events;
        }
        H.st.state = FC_IMU_TIMEOUT;
    } else if (age > H.cfg.stale_us) {
        if (H.st.state != FC_IMU_STALE) {
            ++H.st.stale_events;
        }
        H.st.state = FC_IMU_STALE;
    }
    return H.st.state;
}

FcImuHealthStatus fc_imu_health_status(void) {
    return H.st;
}

bool fc_imu_health_is_ok(void) {
    return H.st.state == FC_IMU_OK;
}

const char *fc_imu_health_state_name(FcImuHealthState s) {
    switch (s) {
    case FC_IMU_NOT_INITIALIZED:
        return "NOT_INITIALIZED";
    case FC_IMU_OK:
        return "OK";
    case FC_IMU_READ_ERROR:
        return "READ_ERROR";
    case FC_IMU_TIMEOUT:
        return "TIMEOUT";
    case FC_IMU_STALE:
        return "STALE";
    case FC_IMU_STUCK:
        return "STUCK";
    case FC_IMU_INVALID:
        return "INVALID";
    default:
        return "?";
    }
}
