// Драйвер ICM-20948 для ESP32 (ТЗ v0.6A §2, §3, §5).
//
// Драйвер отдаёт СЫРЫЕ величины в осях датчика и в физических единицах.
// Он ничего не знает ни о доске, ни о Refloat, ни о том, как модуль
// закреплён: никаких перестановок осей, инверсий знаков и вычисления
// pitch/roll здесь нет и быть не должно (ТЗ v0.6A §4).
//
// Все номера регистров и значения взяты из datasheet InvenSense ICM-20948,
// документ DS-000189 rev 1.6, раздел «Register Map».
#pragma once

#include "esp_err.h"

#include <stdbool.h>
#include <stdint.h>

#define ICM20948_WHO_AM_I_VALUE 0xEA
#define ICM20948_I2C_ADDR_LOW 0x68   // AD0 = 0
#define ICM20948_I2C_ADDR_HIGH 0x69  // AD0 = 1

// Полная шкала акселерометра, DS-000189 §7.1 «ACCEL_CONFIG».
typedef enum {
    ICM20948_ACCEL_FS_2G = 0,
    ICM20948_ACCEL_FS_4G = 1,
    ICM20948_ACCEL_FS_8G = 2,
    ICM20948_ACCEL_FS_16G = 3,
} icm20948_accel_fs_t;

// Полная шкала гироскопа, DS-000189 §7.1 «GYRO_CONFIG_1».
typedef enum {
    ICM20948_GYRO_FS_250DPS = 0,
    ICM20948_GYRO_FS_500DPS = 1,
    ICM20948_GYRO_FS_1000DPS = 2,
    ICM20948_GYRO_FS_2000DPS = 3,
} icm20948_gyro_fs_t;

typedef struct {
    int sda_gpio;
    int scl_gpio;
    uint8_t i2c_addr;
    uint32_t i2c_hz;
    icm20948_accel_fs_t accel_fs;
    icm20948_gyro_fs_t gyro_fs;
    // Делитель частоты выдачи: ODR = 1125 / (1 + div) Гц для обоих датчиков
    // при включённом DLPF (DS-000189 §7.1, GYRO_SMPLRT_DIV/ACCEL_SMPLRT_DIV).
    uint8_t smplrt_div;
    // Конфигурация цифрового ФНЧ, 0…7. 0 — самая широкая полоса и наименьшая
    // фазовая задержка (гироскоп 196.6 Гц, акселерометр 246.0 Гц).
    uint8_t dlpf_cfg;
} icm20948_config_t;

// Сырой семпл в осях датчика.
typedef struct {
    float accel_g[3];      // g, оси датчика X/Y/Z как они подписаны на кристалле
    float gyro_dps[3];     // °/с, те же оси
    float temperature_c;   // °C
    int16_t raw[7];        // ax, ay, az, gx, gy, gz, temp — как пришли по шине
    uint64_t timestamp_us; // момент завершения транзакции, монотонный
    uint32_t sample_counter;
    bool valid;
} icm20948_sample_t;

typedef struct {
    uint64_t reads_ok;
    uint64_t reads_failed;
    uint64_t last_error_us;
    esp_err_t last_error;
    uint32_t max_transaction_us;
    uint64_t sum_transaction_us;
} icm20948_stats_t;

/** Конфигурация по умолчанию, обоснование каждого поля — в .c и docs. */
icm20948_config_t icm20948_default_config(void);

/** Поднять шину и проверить WHO_AM_I. Датчик не настраивается. */
esp_err_t icm20948_bus_init(const icm20948_config_t *cfg);

/** Прочитать WHO_AM_I из USER BANK 0. */
esp_err_t icm20948_who_am_i(uint8_t *out);

/** Полная последовательность: сброс, пробуждение, шкалы, частота выдачи. */
esp_err_t icm20948_init(const icm20948_config_t *cfg);

/** Один семпл: 14 байт одной транзакцией начиная с ACCEL_XOUT_H. */
esp_err_t icm20948_read(icm20948_sample_t *out);

icm20948_stats_t icm20948_stats(void);
const icm20948_config_t *icm20948_active_config(void);

/** Чувствительности для текущих шкал: LSB на 1 g и LSB на 1 °/с. */
float icm20948_accel_lsb_per_g(icm20948_accel_fs_t fs);
float icm20948_gyro_lsb_per_dps(icm20948_gyro_fs_t fs);
float icm20948_accel_fs_g(icm20948_accel_fs_t fs);
float icm20948_gyro_fs_dps(icm20948_gyro_fs_t fs);
/** Фактическая частота выдачи для делителя. */
float icm20948_odr_hz(uint8_t smplrt_div);

/** Диагностика: заставить следующие N чтений завершиться ошибкой (ТЗ §28). */
void icm20948_inject_read_failures(int count);
/** Диагностика: заставить драйвер повторять последний семпл (ТЗ §28). */
void icm20948_inject_frozen(int count);
