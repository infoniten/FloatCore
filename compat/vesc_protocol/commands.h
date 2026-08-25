// Идентификаторы команд протокола VESC.
//
// Перечислены только те, что реально участвуют в работе FloatCore, плюс
// несколько соседних для внятной диагностики. Полный список — в
// bldc/datatypes.h (COMM_PACKET_ID), инвентарь — в docs/vesc_protocol_inventory.md.
#pragma once

#include <stdint.h>

typedef enum {
    COMM_FW_VERSION = 0,
    COMM_JUMP_TO_BOOTLOADER = 1,
    COMM_ERASE_NEW_APP = 2,
    COMM_WRITE_NEW_APP_DATA = 3,
    COMM_GET_VALUES = 4,
    COMM_SET_DUTY = 5,
    COMM_SET_CURRENT = 6,
    COMM_SET_CURRENT_BRAKE = 7,
    COMM_SET_RPM = 8,
    COMM_SET_POS = 9,
    COMM_SET_HANDBRAKE = 10,
    COMM_SET_DETECT = 11,
    COMM_SET_SERVO_POS = 12,
    COMM_SET_MCCONF = 13,
    COMM_GET_MCCONF = 14,
    COMM_GET_MCCONF_DEFAULT = 15,
    COMM_SET_APPCONF = 16,
    COMM_GET_APPCONF = 17,
    COMM_GET_APPCONF_DEFAULT = 18,
    COMM_TERMINAL_CMD = 20,
    COMM_PRINT = 21,
    COMM_REBOOT = 29,
    COMM_ALIVE = 30,
    COMM_FORWARD_CAN = 34,
    COMM_CUSTOM_APP_DATA = 36,
    COMM_GET_VALUES_SELECTIVE = 50,
    COMM_PING_CAN = 62,
    COMM_APP_DISABLE_OUTPUT = 63,
    COMM_GET_IMU_DATA = 65,
    COMM_SET_CURRENT_REL = 84,
    COMM_GET_MCCONF_TEMP = 91,
    COMM_GET_CUSTOM_CONFIG_XML = 92,
    COMM_GET_CUSTOM_CONFIG = 93,
    COMM_GET_CUSTOM_CONFIG_DEFAULT = 94,
    COMM_SET_CUSTOM_CONFIG = 95,
    COMM_GET_QML_UI_HW = 117,
    COMM_GET_QML_UI_APP = 118,
    COMM_CUSTOM_HW_DATA = 119,
    COMM_GET_STATS = 128,
    COMM_LISP_READ_CODE = 130,
    COMM_LISP_GET_STATS = 134,
    COMM_LISP_PRINT = 135,
    COMM_GET_GNSS = 150,
} VescCommandId;

/** Тип оборудования в COMM_FW_VERSION (bldc/datatypes.h HW_TYPE). */
typedef enum {
    VESC_HW_TYPE_VESC = 0,
    VESC_HW_TYPE_VESC_BMS = 1,
    VESC_HW_TYPE_CUSTOM_MODULE = 2,
} VescHwType;

/** Человекочитаемое имя команды (для трассировки). Никогда не NULL. */
const char *vesc_command_name(uint8_t id);

// ---------------------------------------------------------------------------
// Диспетчер команд
// ---------------------------------------------------------------------------

#include "config_bridge.h"
#include "custom_app.h"
#include "firmware_info.h"
#include "packet.h"
#include "telemetry.h"

typedef struct {
    uint32_t rx_frames;
    uint32_t tx_frames;
    uint32_t unsupported_commands;   // команда известна, но FloatCore её не реализует
    uint32_t unknown_commands;       // идентификатор вне известного набора
    uint32_t motor_commands_blocked; // попытки управления мотором — заблокированы
    uint32_t empty_payloads;
    uint32_t truncated_payloads;     // payload короче, чем требуют аргументы команды
} VescServerStats;

typedef struct {
    VescPacket packet;

    FirmwareInfo fw;
    ConfigBridge config;
    CustomAppBridge custom_app;

    /** Провайдер телеметрии. Может быть NULL — тогда COMM_GET_VALUES не отвечает. */
    void (*telemetry_provider)(void *ctx, VescValues *out);

    /** Провайдер QML приложения. Возвращает длину сжатых данных. */
    int (*qml_app_provider)(void *ctx, const uint8_t **data);

    void *ctx;

    bool trace;
    void (*trace_sink)(void *ctx, const char *line);

    VescServerStats stats;

    uint8_t scratch[VESC_PACKET_MAX_PL_LEN];      // ответы на входящие команды
    uint8_t out_scratch[VESC_PACKET_MAX_PL_LEN];  // инициативные посылки прошивки
} VescServer;

/**
 * `send` вызывается с готовым кадром для отправки в транспорт.
 * `ctx` передаётся во все коллбэки.
 */
void vesc_server_init(VescServer *s, VescPacketSendFn send, void *ctx);

/** Скормить принятые из транспорта байты. */
void vesc_server_feed(VescServer *s, const uint8_t *data, size_t len);

/** Отправить данные прошивки в VESC Tool (Refloat → send_app_data). */
bool vesc_server_send_custom_app_data(VescServer *s, const uint8_t *data, size_t len);

void vesc_server_set_trace(VescServer *s, bool on);
