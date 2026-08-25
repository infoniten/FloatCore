#include "firmware_info.h"

#include "commands.h"
#include "vesc_buffer.h"

#include <string.h>

void firmware_info_defaults(FirmwareInfo *info) {
    memset(info, 0, sizeof(*info));

    // 6.06 — версия, которую VESC Tool знает и для которой у него включены
    // COMM_GET_CUSTOM_CONFIG*, COMM_GET_QML_UI_APP и остальное, что нам нужно.
    // Незнакомая версия (например, 99.0) переводит VESC Tool в limited mode,
    // и Refloat UI не открывается.
    info->fw_major = 6;
    info->fw_minor = 6;

    info->hw_name = "FloatCore";
    info->fw_name = "FloatCore";

    // Не VESC-контроллер. VESC Tool при этом не пытается читать mcconf/appconf
    // и не предлагает обновлять прошивку ESC.
    info->hw_type = VESC_HW_TYPE_CUSTOM_MODULE;

    info->custom_config_num = 1;  // конфигурация Refloat
    info->has_phase_filters = false;
    info->qml_hw = 0;
    info->qml_app = 1;  // UI Refloat отдаётся как QML приложения
    info->nrf_flags = 0;
    info->is_paired = false;
    info->test_fw_number = 0;
    info->hw_conf_crc = 0;

    firmware_info_make_uuid("FloatCore", info->uuid);
}

void firmware_info_make_uuid(const char *seed, uint8_t out[FW_INFO_UUID_LEN]) {
    // FNV-1a, развёрнутый в 12 байт. Криптостойкость не нужна: требуется лишь
    // стабильность и различимость устройств.
    uint32_t h = 2166136261u;
    for (const char *s = seed; *s; ++s) {
        h ^= (uint8_t) *s;
        h *= 16777619u;
    }
    for (size_t i = 0; i < FW_INFO_UUID_LEN; ++i) {
        h ^= (uint32_t) (i + 1);
        h *= 16777619u;
        out[i] = (uint8_t) (h >> 24);
    }
}

size_t firmware_info_encode(const FirmwareInfo *info, uint8_t *out, size_t cap) {
    const size_t needed = 1 + 2 + strlen(info->hw_name) + 1 + FW_INFO_UUID_LEN + 7 +
        strlen(info->fw_name) + 1 + 4;
    if (cap < needed) {
        return 0;
    }

    size_t ind = 0;
    vb_append_uint8(out, COMM_FW_VERSION, &ind);
    vb_append_uint8(out, info->fw_major, &ind);
    vb_append_uint8(out, info->fw_minor, &ind);
    vb_append_string(out, info->hw_name, &ind);

    memcpy(out + ind, info->uuid, FW_INFO_UUID_LEN);
    ind += FW_INFO_UUID_LEN;

    vb_append_uint8(out, info->is_paired ? 1 : 0, &ind);
    vb_append_uint8(out, info->test_fw_number, &ind);
    vb_append_uint8(out, info->hw_type, &ind);
    vb_append_uint8(out, info->custom_config_num, &ind);
    vb_append_uint8(out, info->has_phase_filters ? 1 : 0, &ind);
    vb_append_uint8(out, info->qml_hw, &ind);
    vb_append_uint8(out, info->qml_app, &ind);
    vb_append_uint8(out, info->nrf_flags, &ind);
    vb_append_string(out, info->fw_name, &ind);
    vb_append_uint32(out, info->hw_conf_crc, &ind);

    return ind;
}
