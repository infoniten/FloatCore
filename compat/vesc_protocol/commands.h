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
    COMM_GET_DECODED_PPM = 31,
    COMM_GET_DECODED_ADC = 32,
    COMM_GET_DECODED_CHUK = 33,
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
#include "decoded_inputs.h"
#include "firmware_info.h"
#include "packet.h"
#include "telemetry.h"
#include "virtual_mcconf.h"

typedef struct {
    uint32_t rx_frames;
    uint32_t tx_frames;
    uint32_t unsupported_commands;   // команда известна, но FloatCore её не реализует
    uint32_t unknown_commands;       // идентификатор вне известного набора
    uint32_t motor_commands_blocked; // попытки управления мотором — заблокированы
    uint32_t empty_payloads;
    uint32_t truncated_payloads;     // payload короче, чем требуют аргументы команды
    uint32_t mcconf_sent;            // отдано Virtual mcConfig
    uint32_t rt_inputs_sent;         // отдано отчётов о входах RT App (PPM/ADC/CHUK)
    uint32_t mcconf_writes_rejected; // попытки записать конфигурацию мотора
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

    /**
     * Virtual mcConfig. NULL — Refloat UI будет пользоваться значениями
     * по умолчанию из самого VESC Tool (поведение до версии 0.4.1).
     */
    void (*mcconf_provider)(void *ctx, VirtualMcConfValues *out);

    /**
     * Входы RT App (COMM_GET_DECODED_PPM/_ADC/_CHUK).
     *
     * NULL — отвечаем нейтральными значениями: страница RT App заполняется
     * нулями, а не остаётся без ответа. Провайдер только читает состояние
     * платформы; создать через него запрос к мотору невозможно.
     */
    void (*decoded_inputs_provider)(void *ctx, VescDecodedInputs *out);

    /** Схема Motor Configuration. NULL — используется самая новая известная. */
    const McConfSchema *mcconf_schema;

    /**
     * Отправлять Virtual mcConfig сразу после ответа на COMM_FW_VERSION.
     *
     * При hw_type = CUSTOM_MODULE сам VESC Tool никогда не запрашивает
     * конфигурацию мотора (mobile/main.qml проверяет hwTypeStr() === "VESC"),
     * поэтому единственный способ наполнить `VescIf.mcConfig()` — отдать её
     * инициативно. Приёмник в VESC Tool это допускает: Commands::processPacket
     * обрабатывает COMM_GET_MCCONF независимо от того, запрашивал он его или нет.
     */
    bool mcconf_push_on_connect;

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

/** Отправить Virtual mcConfig (текущую проекцию либо значения по умолчанию). */
bool vesc_server_send_mcconf(VescServer *s, bool is_default);
