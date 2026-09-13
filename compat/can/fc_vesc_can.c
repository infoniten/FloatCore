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
    // Номера 59…68 добавлены на v0.7B: на стенде стоит прошивка 6.6, и в её
    // datatypes.h перечисление длиннее того, что было учтено на v0.7A.
    [59] = "GNSS_TIME",
    [60] = "GNSS_LAT",
    [61] = "GNSS_LON",
    [62] = "GNSS_ALT_SPEED_HDOP",
    [63] = "UPDATE_BAUD",
    [64] = "BMS_STATUS_1",
    [65] = "BMS_STATUS_2",
    [66] = "BMS_STATUS_3",
    [67] = "BMS_STATUS_4",
    [68] = "BMS_STATUS_5",
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
bool fc_vesc_can_is_motor_packet(uint32_t t) {
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

// Всё, что меняет состояние ESC. Шире команд мотору: сюда попадает запись и
// сохранение конфигурации, детекция FOC, выключение, смена скорости шины,
// выходы IO-платы и балансировка BMS. Ни один из этих типов не должен быть
// собираем в профиле ACTIVE_DIAG, и tests/can проверяет каждый поимённо.
//
// Отдельно про 5/6/7/8 (буферы): сами по себе они состояние не меняют, но
// несут внутри COMM-пакет, который может менять что угодно. Они не в этом
// списке намеренно — их безопасность решается на уровне COMM, см.
// fc_vesc_comm_is_read_only().
bool fc_vesc_can_is_state_changing(uint32_t t) {
    if (fc_vesc_can_is_motor_packet(t)) {
        return true;
    }
    switch (t) {
    case 19:  // DETECT_APPLY_ALL_FOC — детекция и применение параметров
    case 21:  // CONF_CURRENT_LIMITS
    case 22:  // CONF_STORE_CURRENT_LIMITS — запись во flash
    case 23:  // CONF_CURRENT_LIMITS_IN
    case 24:  // CONF_STORE_CURRENT_LIMITS_IN
    case 25:  // CONF_FOC_ERPMS
    case 26:  // CONF_STORE_FOC_ERPMS
    case 29:  // CONF_BATTERY_CUT
    case 30:  // CONF_STORE_BATTERY_CUT
    case 31:  // SHUTDOWN
    case 36:  // IO_BOARD_SET_OUTPUT_DIGITAL
    case 37:  // IO_BOARD_SET_OUTPUT_PWM
    case 42:  // BMS_BAL
    case 47:  // PSW_SWITCH
    case 55:  // UPDATE_PID_POS_OFFSET
    case 63:  // UPDATE_BAUD — смена скорости шины
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
    r.is_state_changing = false;
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
        r.is_motor_command = fc_vesc_can_is_motor_packet(r.packet_type);
        r.is_state_changing = fc_vesc_can_is_state_changing(r.packet_type);
    }
    return r;
}

// --------------------------------------------------- уровень COMM (v0.7B)

// Имена только тех номеров, которые нам действительно нужны: разрешённые
// запросы и те опасные, против которых написаны тесты. Полная таблица COMM
// здесь не нужна и вводила бы в заблуждение — «имя есть» легко прочитать как
// «пакет поддержан».
const char *fc_vesc_comm_name(uint8_t comm_id) {
    switch (comm_id) {
    case 0:  return "COMM_FW_VERSION";
    case 1:  return "COMM_JUMP_TO_BOOTLOADER";
    case 2:  return "COMM_ERASE_NEW_APP";
    case 3:  return "COMM_WRITE_NEW_APP_DATA";
    case 4:  return "COMM_GET_VALUES";
    case 5:  return "COMM_SET_DUTY";
    case 6:  return "COMM_SET_CURRENT";
    case 7:  return "COMM_SET_CURRENT_BRAKE";
    case 8:  return "COMM_SET_RPM";
    case 9:  return "COMM_SET_POS";
    case 10: return "COMM_SET_HANDBRAKE";
    case 11: return "COMM_SET_DETECT";
    case 12: return "COMM_SET_SERVO_POS";
    case 13: return "COMM_SET_MCCONF";
    case 14: return "COMM_GET_MCCONF";
    case 15: return "COMM_GET_MCCONF_DEFAULT";
    case 16: return "COMM_SET_APPCONF";
    case 17: return "COMM_GET_APPCONF";
    case 18: return "COMM_GET_APPCONF_DEFAULT";
    case 20: return "COMM_TERMINAL_CMD";
    case 24: return "COMM_DETECT_MOTOR_PARAM";
    case 25: return "COMM_DETECT_MOTOR_R_L";
    case 26: return "COMM_DETECT_MOTOR_FLUX_LINKAGE";
    case 27: return "COMM_DETECT_ENCODER";
    case 28: return "COMM_DETECT_HALL_FOC";
    case 29: return "COMM_REBOOT";
    case 30: return "COMM_ALIVE";
    case 36: return "COMM_CUSTOM_APP_DATA";
    case 47: return "COMM_GET_VALUES_SETUP";
    case 48: return "COMM_SET_MCCONF_TEMP";
    case 49: return "COMM_SET_MCCONF_TEMP_SETUP";
    case 50: return "COMM_GET_VALUES_SELECTIVE";
    default: return NULL;
    }
}

// Белый список. Умолчание — «нельзя»: любой номер, которого здесь нет,
// считается опасным, включая номера из будущих версий прошивки.
//
// Каждая ветка проверена по bldc release_6_06, comm/commands.c:
//   COMM_FW_VERSION (0)          строки 231…  только чтение констант и UUID
//   COMM_GET_VALUES (4)          строки 384…  упаковка телеметрии
//   COMM_GET_MCCONF (14)         строки 591…  копия mc_interface_get_configuration()
//   COMM_GET_MCCONF_DEFAULT (15) строки 591…  значения по умолчанию
//   COMM_GET_APPCONF (17)        строки 654…  копия app_get_configuration()
//   COMM_GET_APPCONF_DEFAULT (18) строки 654… значения по умолчанию
//   COMM_GET_VALUES_SELECTIVE (50) строки 384… та же ветка, что GET_VALUES
//
// Проверялось именно отсутствие побочного эффекта, а не «название начинается
// с GET»: timeout_reset() в commands.c встречается только в ветках
// COMM_SET_DUTY, COMM_ALIVE, COMM_SET_ODOMETER и COMM_SET_CURRENT_REL, ни
// одной из которых в списке нет.
bool fc_vesc_comm_is_read_only(uint8_t comm_id) {
    switch (comm_id) {
    case 0:   // COMM_FW_VERSION
    case 4:   // COMM_GET_VALUES
    case 14:  // COMM_GET_MCCONF
    case 15:  // COMM_GET_MCCONF_DEFAULT
    case 17:  // COMM_GET_APPCONF
    case 18:  // COMM_GET_APPCONF_DEFAULT
    case 50:  // COMM_GET_VALUES_SELECTIVE
        return true;
    default:
        return false;
    }
}
