#include "config_bridge.h"

#include "commands.h"
#include "packet.h"
#include "vesc_buffer.h"

#include <string.h>

size_t config_bridge_encode_xml_chunk(
    const ConfigBridge *cb, int conf_ind, int32_t req_len, int32_t req_offset, uint8_t *out,
    size_t cap
) {
    if (!cb->get_xml || conf_ind < 0 || conf_ind >= cb->config_count) {
        return 0;
    }

    const uint8_t *data = NULL;
    int total = cb->get_xml(cb->ctx, conf_ind, &data);
    if (total <= 0 || !data) {
        return 0;
    }

    // Клип запроса в границы данных: VESC Tool просит не более 400 байт,
    // но доверять входу нельзя.
    if (req_offset < 0 || req_offset > total) {
        return 0;
    }
    if (req_len < 0) {
        req_len = 0;
    }
    int32_t available = total - req_offset;
    if (req_len > available) {
        req_len = available;
    }

    const size_t header = 1 + 1 + 4 + 4;
    if (cap < header + (size_t) req_len || header + (size_t) req_len > VESC_PACKET_MAX_PL_LEN) {
        return 0;
    }

    size_t ind = 0;
    vb_append_uint8(out, COMM_GET_CUSTOM_CONFIG_XML, &ind);
    vb_append_int8(out, (int8_t) conf_ind, &ind);
    vb_append_int32(out, total, &ind);
    vb_append_int32(out, req_offset, &ind);
    memcpy(out + ind, data + req_offset, (size_t) req_len);
    ind += (size_t) req_len;
    return ind;
}

size_t config_bridge_encode_config(
    const ConfigBridge *cb, int conf_ind, bool is_default, uint8_t *out, size_t cap
) {
    if (!cb->get || conf_ind < 0 || conf_ind >= cb->config_count) {
        return 0;
    }
    if (cap < 2) {
        return 0;
    }

    size_t ind = 0;
    vb_append_uint8(out, is_default ? COMM_GET_CUSTOM_CONFIG_DEFAULT : COMM_GET_CUSTOM_CONFIG, &ind);
    vb_append_int8(out, (int8_t) conf_ind, &ind);

    int len = cb->get(cb->ctx, conf_ind, out + ind, cap - ind, is_default);
    if (len < 0) {
        return 0;
    }
    ind += (size_t) len;
    return ind;
}
