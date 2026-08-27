// Явный выбор источника данных IMU (ТЗ v0.6D §7).
//
// Источник ровно один на всю сборку и на всё время работы. Смешивать их в
// одной итерации запрещено: Refloat берёт углы из AHRS платформы, а сырые
// массивы — из callback, и если эти два пути придут из разных источников,
// Refloat получит противоречивые данные, а обнаружится это только под ногами.
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    FC_IMU_SOURCE_MOCK = 0,  // покоящаяся заглушка; используется host-тестами
    FC_IMU_SOURCE_REAL,      // физический ICM-20948 через единый realtime-тракт
} FcImuSource;

// Источник для этой сборки. Константа, а не переменная: выбор источника —
// свойство прошивки, а не то, что может перещёлкнуться во время работы.
// Host-тесты собирают mock, прошивка ESP32 с v0.6D — реальный датчик.
#define FC_IMU_SOURCE_DEFAULT FC_IMU_SOURCE_REAL

/** Имя источника без его запуска — нужно баннеру загрузки. */
const char *fc_imu_source_name_of(FcImuSource s);

/** Поднять выбранный источник. Возвращает false, если реальный не поднялся. */
bool fc_imu_source_start(FcImuSource source);

FcImuSource fc_imu_source(void);
const char *fc_imu_source_name(void);
/** Готов ли источник отдавать данные (для реального — датчик стабилен). */
bool fc_imu_source_available(void);
