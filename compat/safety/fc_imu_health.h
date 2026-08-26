// Диагностика сырого IMU, независимая от модели датчика и от механического
// монтажа (ТЗ v0.6A §7).
//
// Никаких допущений об ориентации: проверяются только величины, которые
// осмысленны для любого положения датчика — модуль вектора ускорения,
// модуль угловой скорости, конечность чисел, монотонность времени и
// продвижение счётчика семплов.
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    FC_IMU_NOT_INITIALIZED = 0,
    FC_IMU_OK,
    FC_IMU_READ_ERROR,
    FC_IMU_TIMEOUT,
    FC_IMU_STALE,
    FC_IMU_STUCK,
    FC_IMU_INVALID,
    FC_IMU_HEALTH_COUNT
} FcImuHealthState;

typedef struct {
    float accel_g[3];
    float gyro_dps[3];
    float temperature_c;
    uint64_t timestamp_us;
    uint32_t sample_counter;
    bool valid;
} FcImuRawSample;

typedef struct {
    // Границы модуля вектора ускорения. Взяты из datasheet ICM-20948
    // (DS-000189): полная шкала по умолчанию ±2 g, то есть физически датчик
    // не может показать больше 2 g по оси и ~3.46 g по модулю. Нижняя
    // граница отражает свободное падение, верхняя — насыщение шкалы.
    float accel_mag_min_g;
    float accel_mag_max_g;
    // Полная шкала гироскопа по умолчанию ±250 °/с (там же).
    float gyro_abs_max_dps;
    // Сколько подряд идентичных семплов считать зависанием. Политика
    // FloatCore: датчик с шумом в младших разрядах не выдаёт два побитово
    // одинаковых семпла подряд, но одиночное совпадение возможно.
    uint32_t stuck_limit;
    // Возраст, после которого семпл считается протухшим.
    uint64_t stale_us;
    // Возраст, после которого считаем, что связи нет вовсе.
    uint64_t timeout_us;
} FcImuHealthConfig;

typedef struct {
    FcImuHealthState state;
    uint64_t samples_total;
    uint64_t read_errors;
    uint64_t invalid_samples;
    uint64_t stuck_events;
    uint64_t stale_events;
    uint64_t timeout_events;
    uint64_t last_good_us;
    uint32_t consecutive_identical;
    uint32_t last_counter;
    FcImuRawSample last_good;
} FcImuHealthStatus;

void fc_imu_health_init(const FcImuHealthConfig *cfg);
FcImuHealthConfig fc_imu_health_default_config(void);

/**
 * Конфигурация под фактические шкалы датчика. Пороги правдоподобия обязаны
 * следовать за настройкой драйвера: при ±4 g физический потолок модуля
 * вектора — 4*sqrt(3) g, и порог от шкалы ±2 g отвергал бы валидные данные.
 */
FcImuHealthConfig fc_imu_health_config_for(float accel_fs_g, float gyro_fs_dps);

/** Учесть очередной результат чтения. ok=false — транзакция не удалась. */
FcImuHealthState fc_imu_health_update(bool ok, const FcImuRawSample *sample, uint64_t now_us);

/** Проверка возраста без нового семпла. Вызывается периодически. */
FcImuHealthState fc_imu_health_poll(uint64_t now_us);

FcImuHealthStatus fc_imu_health_status(void);
bool fc_imu_health_is_ok(void);
const char *fc_imu_health_state_name(FcImuHealthState s);
