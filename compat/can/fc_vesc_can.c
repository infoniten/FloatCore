#include "fc_vesc_can.h"

#include <stddef.h>

// Таблица из bldc/datatypes.h, перечисление CAN_PACKET_ID. Заполнены только
// те номера, которые в нём действительно есть; пропуски дают NULL и приводят
// к known_type = false.
static const char *const NAMES[] = {
    [0] = "SET_DUTY",
    [1] = "SET_CURRENT",
    [2] = "SET_CURRENT_BRAKE",
    [3] = "SET_RPM",
    [4] = "SET_POS",
    [5] = "FILL_RX_BUFFER",
    [6] = "FILL_RX_BUFFER_LONG",
    [7] = "PROCESS_RX_BUFFER",
    [8] = "PROCESS_SHORT_BUFFER",
    [9] = "STATUS",
    [10] = "SET_CURRENT_REL",
    [11] = "SET_CURRENT_BRAKE_REL",
    [12] = "SET_CURRENT_HANDBRAKE",
    [13] = "SET_CURRENT_HANDBRAKE_REL",
    [14] = "STATUS_2",
    [15] = "STATUS_3",
    [16] = "STATUS_4",
    [17] = "PING",
    [18] = "PONG",
    [19] = "DETECT_APPLY_ALL_FOC",
    [20] = "DETECT_APPLY_ALL_FOC_RES",
    [21] = "CONF_CURRENT_LIMITS",
    [22] = "CONF_STORE_CURRENT_LIMITS",
    [23] = "CONF_CURRENT_LIMITS_IN",
    [24] = "CONF_STORE_CURRENT_LIMITS_IN",
    [25] = "CONF_FOC_ERPMS",
    [26] = "CONF_STORE_FOC_ERPMS",
    [27] = "STATUS_5",
    [28] = "POLL_TS5700N8501_STATUS",
    [29] = "CONF_BATTERY_CUT",
    [30] = "CONF_STORE_BATTERY_CUT",
    [31] = "SHUTDOWN",
    [32] = "IO_BOARD_ADC_1_TO_4",
    [33] = "IO_BOARD_ADC_5_TO_8",
    [34] = "IO_BOARD_ADC_9_TO_12",
    [35] = "IO_BOARD_DIGITAL_IN",
    [36] = "IO_BOARD_SET_OUTPUT_DIGITAL",
    [37] = "IO_BOARD_SET_OUTPUT_PWM",
    [38] = "BMS_V_TOT",
    [39] = "BMS_I",
    [40] = "BMS_AH_WH",
    [41] = "BMS_V_CELL",
    [42] = "BMS_BAL",
    [43] = "BMS_TEMPS",
    [44] = "BMS_HUM",
    [45] = "BMS_SOC_SOH_TEMP_STAT",
    [46] = "PSW_STAT",
    [47] = "PSW_SWITCH",
    [48] = "BMS_HW_DATA_1",
    [49] = "BMS_HW_DATA_2",
    [50] = "BMS_HW_DATA_3",
    [51] = "BMS_HW_DATA_4",
    [52] = "BMS_HW_DATA_5",
    [53] = "BMS_AH_WH_CHG_TOTAL",
    [54] = "BMS_AH_WH_DIS_TOTAL",
    [55] = "UPDATE_PID_POS_OFFSET",
    [56] = "POLL_ROTOR_POS",
    [57] = "NOTIFY_BOOT",
    [58] = "STATUS_6",
};

#define NAMES_COUNT (sizeof(NAMES) / sizeof(NAMES[0]))

const char *fc_vesc_can_type_name(uint32_t packet_type) {
    if (packet_type >= NAMES_COUNT) {
        return NULL;
    }
    return NAMES[packet_type];
}

// Типы, способные задать выход на мотор. Перечислены явно, а не выведены по
// префиксу имени: список нужен для диагностики «кто-то командует мотором»,
// и ошибка в нём означала бы пропущенную команду.
static bool is_motor_command(uint32_t t) {
    switch (t) {
    case 0:   // SET_DUTY
    case 1:   // SET_CURRENT
    case 2:   // SET_CURRENT_BRAKE
    case 3:   // SET_RPM
    case 4:   // SET_POS
    case 10:  // SET_CURRENT_REL
    case 11:  // SET_CURRENT_BRAKE_REL
    case 12:  // SET_CURRENT_HANDBRAKE
    case 13:  // SET_CURRENT_HANDBRAKE_REL
        return true;
    default:
        return false;
    }
}

static bool is_status(uint32_t t) {
    return t == 9 || t == 14 || t == 15 || t == 16 || t == 27 || t == 58;
}

FcVescCanId fc_vesc_can_decode(uint32_t id, bool extended) {
    FcVescCanId r;
    r.vesc_format = false;
    r.controller_id = 0;
    r.packet_type = 0;
    r.known_type = false;
    r.is_status = false;
    r.is_motor_command = false;
    r.name = NULL;

    if (!extended) {
        // Протокол VESC использует только расширенные идентификаторы.
        // Стандартный кадр на этой шине принадлежит кому-то ещё, и толковать
        // его по формату VESC нельзя.
        return r;
    }

    r.vesc_format = true;
    r.controller_id = (uint8_t) (id & 0xFF);
    r.packet_type = id >> 8;
    r.name = fc_vesc_can_type_name(r.packet_type);
    r.known_type = r.name != NULL;
    if (r.known_type) {
        r.is_status = is_status(r.packet_type);
        r.is_motor_command = is_motor_command(r.packet_type);
    }
    return r;
}
