// Постоянное хранение калибровки IMU. Реализация и обоснование —
// fc_imu_cal_store.c. Отдельный заголовок, а не запись в fc_platform.h:
// сигнатуры используют типы слоя калибровки, а fc_platform.h намеренно не
// зависит ни от чего, кроме стандартных заголовков.
#pragma once

#include "../../../compat/imu/fc_imu_calibration.h"

/** Прочитать калибровку из NVS. При любой проблеме отдаёт единичную. */
FcImuCalStatus fc_imu_cal_store_load(FcImuCalibration *out);

/** Сохранить. Отвергается вне DISARMED и при неправдоподобных значениях. */
bool fc_imu_cal_store_save(const FcImuCalibration *c);

/** Стереть запись: носитель возвращается в состояние «калибровки нет». */
bool fc_imu_cal_store_clear(void);
