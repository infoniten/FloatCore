// Идентификация устройства для VESC Tool (COMM_FW_VERSION).
//
// FloatCore не выдаёт себя за VESC-контроллер: hw_type = CUSTOM_MODULE,
// собственные имена железа и прошивки. Версия протокола (major.minor)
// сообщается совместимая, потому что именно по ней VESC Tool решает,
// какие команды он вправе использовать.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FW_INFO_UUID_LEN 12

typedef struct {
    // Версия протокола/прошивки. VESC Tool сверяет её со своим списком
    // поддерживаемых версий: незнакомая версия переводит его в limited mode,
    // где недоступны custom config и QML.
    uint8_t fw_major;
    uint8_t fw_minor;

    const char *hw_name;  // показывается как "Hardware"
    const char *fw_name;  // показывается как имя прошивки

    uint8_t uuid[FW_INFO_UUID_LEN];  // виртуальный, стабильный между запусками

    uint8_t hw_type;             // VescHwType
    uint8_t custom_config_num;   // сколько конфигураций отдаёт устройство
    bool has_phase_filters;
    uint8_t qml_hw;              // 0 нет, 1 есть, 2 fullscreen
    uint8_t qml_app;             // 0 нет, 1 есть, 2 fullscreen
    uint8_t nrf_flags;
    bool is_paired;
    uint8_t test_fw_number;      // 0 — не тестовая прошивка
    uint32_t hw_conf_crc;
} FirmwareInfo;

/** Значения по умолчанию для FloatCore. */
void firmware_info_defaults(FirmwareInfo *info);

/**
 * Сгенерировать стабильный виртуальный UUID из строки-seed.
 * Одинаковый seed даёт одинаковый UUID при каждом запуске — VESC Tool
 * использует UUID для привязки кэшей и профилей.
 */
void firmware_info_make_uuid(const char *seed, uint8_t out[FW_INFO_UUID_LEN]);

/**
 * Собрать payload ответа COMM_FW_VERSION (включая байт команды).
 * Порядок полей соответствует парсеру vesc_tool/commands.cpp.
 * Возвращает длину или 0, если не хватило места.
 */
size_t firmware_info_encode(const FirmwareInfo *info, uint8_t *out, size_t cap);
