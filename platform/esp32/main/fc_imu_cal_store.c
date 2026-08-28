// Постоянное хранение калибровки IMU (ТЗ v0.6E §6).
//
// Калибровка хранится ОТДЕЛЬНО от конфигурации Refloat: она принадлежит
// платформенному слою, как imu_config в прошивке VESC, и Refloat про неё не
// знает. Физически используется тот же массив слов NVS, но в диапазоне
// адресов, которого Refloat не касается.
//
// Политика записи не меняется: запись разрешена только в DISARMED и никогда не
// выполняется из realtime-задачи. Коммит во flash делает отдельная задача
// хранилища (docs/config_persistence_policy.md).

#include "fc_imu_cal_store.h"

#include "fc_platform.h"

#include "../../../compat/safety/fc_supervisor.h"

#include "esp_log.h"

static const char *TAG = "imu_cal";

// Адреса слов. Refloat занимает 0…79, счётчик загрузок стоит на 100.
// Калибровка живёт с 104: между ними оставлен зазор, чтобы случайная ошибка на
// единицу не портила чужие данные.
#define FC_IMU_CAL_ADDR 104

_Static_assert(FC_IMU_CAL_ADDR + FC_IMU_CAL_WORDS <= 128, "калибровка не помещается в хранилище");

FcImuCalStatus fc_imu_cal_store_load(FcImuCalibration *out) {
    uint32_t words[FC_IMU_CAL_WORDS];
    for (int i = 0; i < FC_IMU_CAL_WORDS; ++i) {
        if (!fc_storage_read(&words[i], FC_IMU_CAL_ADDR + i)) {
            // Хранилище не поднялось — это не «калибровки нет», это «мы не
            // знаем». Считаем некалиброванным: безопасная сторона.
            *out = fc_imu_calibration_identity();
            return FC_IMU_CAL_NOT_CALIBRATED;
        }
    }
    return fc_imu_calibration_deserialize(words, out);
}

bool fc_imu_cal_store_save(const FcImuCalibration *c) {
    // Проверка политики здесь, а не у вызывающего: запись конфигурации вне
    // DISARMED запрещена независимо от того, кто её просит.
    if (!fc_supervisor_config_write_allowed()) {
        fc_storage_note_rejected_write();
        ESP_LOGW(TAG, "запись калибровки отклонена: состояние %s",
                 fc_supervisor_state_name(fc_supervisor_state()));
        return false;
    }
    if (!fc_imu_calibration_plausible(c)) {
        ESP_LOGW(TAG, "запись калибровки отклонена: значения неправдоподобны");
        return false;
    }

    uint32_t words[FC_IMU_CAL_WORDS];
    fc_imu_calibration_serialize(c, words);
    for (int i = 0; i < FC_IMU_CAL_WORDS; ++i) {
        if (!fc_storage_write(words[i], FC_IMU_CAL_ADDR + i)) {
            return false;
        }
    }
    // Коммит просит отдельная задача хранилища: синхронная запись во flash из
    // вызывающего контекста останавливала бы систему на миллисекунды.
    return fc_storage_request_commit();
}

bool fc_imu_cal_store_clear(void) {
    if (!fc_supervisor_config_write_allowed()) {
        fc_storage_note_rejected_write();
        return false;
    }
    // Стёртая ячейка, а не нули: так же выглядит носитель до первой записи, и
    // разбор вернёт NOT_CALIBRATED, а не INVALID.
    for (int i = 0; i < FC_IMU_CAL_WORDS; ++i) {
        if (!fc_storage_write(0xFFFFFFFFu, FC_IMU_CAL_ADDR + i)) {
            return false;
        }
    }
    return fc_storage_request_commit();
}
