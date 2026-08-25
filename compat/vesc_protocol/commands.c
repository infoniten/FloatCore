#include "commands.h"

const char *vesc_command_name(uint8_t id) {
    switch (id) {
    case COMM_FW_VERSION:
        return "FW_VERSION";
    case COMM_GET_VALUES:
        return "GET_VALUES";
    case COMM_SET_DUTY:
        return "SET_DUTY";
    case COMM_SET_CURRENT:
        return "SET_CURRENT";
    case COMM_SET_CURRENT_BRAKE:
        return "SET_CURRENT_BRAKE";
    case COMM_SET_RPM:
        return "SET_RPM";
    case COMM_SET_POS:
        return "SET_POS";
    case COMM_SET_HANDBRAKE:
        return "SET_HANDBRAKE";
    case COMM_SET_DETECT:
        return "SET_DETECT";
    case COMM_SET_MCCONF:
        return "SET_MCCONF";
    case COMM_GET_MCCONF:
        return "GET_MCCONF";
    case COMM_GET_MCCONF_DEFAULT:
        return "GET_MCCONF_DEFAULT";
    case COMM_SET_APPCONF:
        return "SET_APPCONF";
    case COMM_GET_APPCONF:
        return "GET_APPCONF";
    case COMM_GET_APPCONF_DEFAULT:
        return "GET_APPCONF_DEFAULT";
    case COMM_TERMINAL_CMD:
        return "TERMINAL_CMD";
    case COMM_PRINT:
        return "PRINT";
    case COMM_REBOOT:
        return "REBOOT";
    case COMM_ALIVE:
        return "ALIVE";
    case COMM_FORWARD_CAN:
        return "FORWARD_CAN";
    case COMM_CUSTOM_APP_DATA:
        return "CUSTOM_APP_DATA";
    case COMM_GET_VALUES_SELECTIVE:
        return "GET_VALUES_SELECTIVE";
    case COMM_PING_CAN:
        return "PING_CAN";
    case COMM_APP_DISABLE_OUTPUT:
        return "APP_DISABLE_OUTPUT";
    case COMM_GET_IMU_DATA:
        return "GET_IMU_DATA";
    case COMM_SET_CURRENT_REL:
        return "SET_CURRENT_REL";
    case COMM_GET_MCCONF_TEMP:
        return "GET_MCCONF_TEMP";
    case COMM_GET_CUSTOM_CONFIG_XML:
        return "GET_CUSTOM_CONFIG_XML";
    case COMM_GET_CUSTOM_CONFIG:
        return "GET_CUSTOM_CONFIG";
    case COMM_GET_CUSTOM_CONFIG_DEFAULT:
        return "GET_CUSTOM_CONFIG_DEFAULT";
    case COMM_SET_CUSTOM_CONFIG:
        return "SET_CUSTOM_CONFIG";
    case COMM_GET_QML_UI_HW:
        return "GET_QML_UI_HW";
    case COMM_GET_QML_UI_APP:
        return "GET_QML_UI_APP";
    case COMM_CUSTOM_HW_DATA:
        return "CUSTOM_HW_DATA";
    case COMM_GET_STATS:
        return "GET_STATS";
    case COMM_LISP_READ_CODE:
        return "LISP_READ_CODE";
    case COMM_LISP_GET_STATS:
        return "LISP_GET_STATS";
    case COMM_GET_GNSS:
        return "GET_GNSS";
    default:
        return "UNKNOWN";
    }
}

// ---------------------------------------------------------------------------
// Диспетчер команд
// ---------------------------------------------------------------------------

#include "vesc_buffer.h"

#include <stdio.h>
#include <string.h>

static void trace(VescServer *s, const char *dir, uint8_t cmd, size_t len, const char *note) {
    if (!s->trace || !s->trace_sink) {
        return;
    }
    char line[160];
    snprintf(
        line, sizeof(line), "%s  id=%-3u %-24s len=%zu%s%s", dir, cmd, vesc_command_name(cmd), len,
        note ? "  " : "", note ? note : ""
    );
    s->trace_sink(s->ctx, line);
}

static bool reply_buf(VescServer *s, const uint8_t *buf, size_t len) {
    if (len == 0) {
        return false;
    }
    trace(s, "TX", buf[0], len, NULL);
    ++s->stats.tx_frames;
    return vesc_packet_send(&s->packet, buf, len);
}

#define reply(s, len) reply_buf((s), (s)->scratch, (len))

/**
 * Команды управления мотором. На этом этапе физический вывод невозможен:
 * они только считаются и логируются (ТЗ v0.4 §10).
 */
static bool is_motor_command(uint8_t cmd) {
    switch (cmd) {
    case COMM_SET_DUTY:
    case COMM_SET_CURRENT:
    case COMM_SET_CURRENT_BRAKE:
    case COMM_SET_RPM:
    case COMM_SET_POS:
    case COMM_SET_HANDBRAKE:
    case COMM_SET_DETECT:
    case COMM_SET_CURRENT_REL:
        return true;
    default:
        return false;
    }
}

static bool is_known_command(uint8_t cmd) {
    return strcmp(vesc_command_name(cmd), "UNKNOWN") != 0;
}

static void handle_payload(void *ctx, const uint8_t *payload, size_t len) {
    VescServer *s = (VescServer *) ctx;
    ++s->stats.rx_frames;

    if (len == 0) {
        ++s->stats.empty_payloads;
        return;
    }

    const uint8_t cmd = payload[0];
    const uint8_t *args = payload + 1;
    const size_t args_len = len - 1;

    trace(s, "RX", cmd, len, NULL);

    if (is_motor_command(cmd)) {
        ++s->stats.motor_commands_blocked;
        trace(s, "!!", cmd, len, "motor command BLOCKED (mock backend, no CAN output)");
        return;
    }

    switch (cmd) {
    case COMM_FW_VERSION: {
        reply(s, firmware_info_encode(&s->fw, s->scratch, sizeof(s->scratch)));
        return;
    }

    case COMM_ALIVE:
        // Прошивка VESC на COMM_ALIVE не отвечает, только сбрасывает таймаут.
        return;

    case COMM_GET_VALUES:
    case COMM_GET_VALUES_SELECTIVE: {
        if (!s->telemetry_provider) {
            ++s->stats.unsupported_commands;
            return;
        }
        uint32_t mask = 0xFFFFFFFF;
        bool selective = cmd == COMM_GET_VALUES_SELECTIVE;
        if (selective) {
            if (args_len < 4) {
                ++s->stats.truncated_payloads;
                return;
            }
            size_t ind = 0;
            mask = vb_get_uint32(args, &ind);
        }
        VescValues values;
        s->telemetry_provider(s->ctx, &values);
        reply(s, telemetry_encode(&values, mask, selective, s->scratch, sizeof(s->scratch)));
        return;
    }

    case COMM_GET_CUSTOM_CONFIG_XML: {
        if (args_len < 9) {
            ++s->stats.truncated_payloads;
            return;
        }
        size_t ind = 0;
        int conf_ind = vb_get_int8(args, &ind);
        int32_t req_len = vb_get_int32(args, &ind);
        int32_t req_ofs = vb_get_int32(args, &ind);
        reply(
            s,
            config_bridge_encode_xml_chunk(
                &s->config, conf_ind, req_len, req_ofs, s->scratch, sizeof(s->scratch)
            )
        );
        return;
    }

    case COMM_GET_CUSTOM_CONFIG:
    case COMM_GET_CUSTOM_CONFIG_DEFAULT: {
        if (args_len < 1) {
            ++s->stats.truncated_payloads;
            return;
        }
        size_t ind = 0;
        int conf_ind = vb_get_int8(args, &ind);
        reply(
            s,
            config_bridge_encode_config(
                &s->config, conf_ind, cmd == COMM_GET_CUSTOM_CONFIG_DEFAULT, s->scratch,
                sizeof(s->scratch)
            )
        );
        return;
    }

    case COMM_SET_CUSTOM_CONFIG: {
        if (args_len < 1) {
            ++s->stats.truncated_payloads;
            return;
        }
        size_t ind = 0;
        int conf_ind = vb_get_int8(args, &ind);
        bool ok = false;
        if (s->config.set) {
            ok = s->config.set(s->config.ctx, conf_ind, args + ind, args_len - ind);
        }
        trace(s, "..", cmd, args_len, ok ? "config applied" : "config REJECTED");
        // Прошивка VESC подтверждает запись, отдавая конфигурацию обратно.
        reply(
            s,
            config_bridge_encode_config(&s->config, conf_ind, false, s->scratch, sizeof(s->scratch))
        );
        return;
    }

    case COMM_GET_QML_UI_APP: {
        if (args_len < 8) {
            ++s->stats.truncated_payloads;
            return;
        }
        if (!s->qml_app_provider) {
            ++s->stats.unsupported_commands;
            return;
        }
        size_t ind = 0;
        int32_t req_len = vb_get_int32(args, &ind);
        int32_t req_ofs = vb_get_int32(args, &ind);

        const uint8_t *data = NULL;
        int total = s->qml_app_provider(s->ctx, &data);
        if (total <= 0 || !data || req_ofs < 0 || req_ofs > total) {
            return;
        }
        if (req_len < 0) {
            req_len = 0;
        }
        if (req_len > total - req_ofs) {
            req_len = total - req_ofs;
        }
        const size_t header = 1 + 4 + 4;
        if (header + (size_t) req_len > sizeof(s->scratch)) {
            return;
        }

        size_t o = 0;
        vb_append_uint8(s->scratch, COMM_GET_QML_UI_APP, &o);
        vb_append_int32(s->scratch, total, &o);
        vb_append_int32(s->scratch, req_ofs, &o);
        memcpy(s->scratch + o, data + req_ofs, (size_t) req_len);
        o += (size_t) req_len;
        reply(s, o);
        return;
    }

    case COMM_CUSTOM_APP_DATA: {
        if (s->custom_app.to_firmware) {
            s->custom_app.to_firmware(s->custom_app.ctx, args, args_len);
        } else {
            ++s->stats.unsupported_commands;
        }
        return;
    }

    case COMM_PING_CAN: {
        // Устройств на CAN не публикуем: FloatCore сам является узлом,
        // а проксирование CAN на этом этапе запрещено.
        size_t o = 0;
        vb_append_uint8(s->scratch, COMM_PING_CAN, &o);
        reply(s, o);
        return;
    }

    default:
        if (is_known_command(cmd)) {
            ++s->stats.unsupported_commands;
            trace(s, "..", cmd, len, "known but not implemented — ignored");
        } else {
            ++s->stats.unknown_commands;
            trace(s, "..", cmd, len, "unknown command — ignored");
        }
        return;
    }
}

void vesc_server_init(VescServer *s, VescPacketSendFn send, void *ctx) {
    memset(s, 0, sizeof(*s));
    s->ctx = ctx;
    vesc_packet_init(&s->packet, send, handle_payload, s);
    firmware_info_defaults(&s->fw);
    s->config.config_count = 0;
}

void vesc_server_feed(VescServer *s, const uint8_t *data, size_t len) {
    vesc_packet_process_buffer(&s->packet, data, len);
}

bool vesc_server_send_custom_app_data(VescServer *s, const uint8_t *data, size_t len) {
    // Отдельный буфер: этот путь вызывается из потоков Refloat, а scratch
    // используется обработчиком входящих пакетов.
    size_t n = custom_app_encode(data, len, s->out_scratch, sizeof(s->out_scratch));
    return reply_buf(s, s->out_scratch, n);
}

void vesc_server_set_trace(VescServer *s, bool on) {
    s->trace = on;
}
