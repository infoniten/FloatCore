// Разбор идентификаторов CAN протокола VESC (ТЗ v0.7A §9).
//
// Формат взят из bldc/comm/comm_can.c, а не выведен из наблюдаемых данных:
//
//     передача:  eid = controller_id | ((uint32_t) packet_id << 8)
//     приём:     uint8_t id = eid & 0xFF;
//                CAN_PACKET_ID cmd = eid >> 8;
//
// Номера типов пакетов — из bldc/datatypes.h, перечисление CAN_PACKET_ID.
// Всё, чего там нет, помечается как неизвестное и таковым и остаётся:
// догадываться по структуре идентификатора запрещено.
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool vesc_format;      // расширенный кадр, разбираемый как VESC
    uint8_t controller_id; // младшие 8 бит
    uint32_t packet_type;  // старшие биты
    bool known_type;       // тип есть в перечислении bldc
    bool is_status;        // периодический статус (не команда)
    bool is_motor_command; // тип, способный задать выход на мотор
    bool is_state_changing; // тип, меняющий состояние ESC (шире, чем мотор)
    const char *name;      // имя из bldc или NULL
} FcVescCanId;

/**
 * Разобрать идентификатор.
 *
 * Стандартные (11-битные) кадры протоколом VESC не используются, поэтому для
 * них vesc_format = false и разбор не производится — иначе чужой кадр на
 * общей шине был бы истолкован как VESC.
 */
FcVescCanId fc_vesc_can_decode(uint32_t id, bool extended);

/** Имя типа пакета или NULL, если такого номера в bldc нет. */
const char *fc_vesc_can_type_name(uint32_t packet_type);

/**
 * Тип пакета способен задать выход на мотор.
 *
 * Список явный, а не по префиксу имени: он используется и для диагностики
 * «кто-то командует мотором», и как запрет в белом списке передачи v0.7B.
 */
bool fc_vesc_can_is_motor_packet(uint32_t packet_type);

/**
 * Тип пакета меняет состояние ESC — шире, чем мотор.
 *
 * Сюда входят команды мотору, запись и сохранение конфигурации, детекция,
 * выключение, смена скорости шины, выходы IO-платы и балансировка BMS. В
 * профиле ACTIVE_DIAG собрать ни один из них нельзя.
 */
bool fc_vesc_can_is_state_changing(uint32_t packet_type);

// ------------------------------------------- уровень COMM внутри CAN (v0.7B)
//
// Пакеты FILL_RX_BUFFER / PROCESS_RX_BUFFER / PROCESS_SHORT_BUFFER несут
// внутри себя обычный пакет COMM_*, который целевой ESC отдаёт в
// commands_process_packet(). То есть сам номер типа CAN о безопасности не
// говорит ничего: PROCESS_SHORT_BUFFER одинаково пригоден и для запроса
// версии прошивки, и для записи конфигурации. Поэтому белый список v0.7B
// проверяется на уровне COMM, а не CAN.

/** Имя COMM-пакета, если оно нам известно, иначе NULL. */
const char *fc_vesc_comm_name(uint8_t comm_id);

/**
 * COMM-пакет доказанно не меняет состояние ESC.
 *
 * Значение по умолчанию — false: неизвестный номер считается опасным. Список
 * подтверждён по bldc release_6_06, comm/commands.c — в перечисленных ветках
 * нет ни записи конфигурации, ни timeout_reset(), ни обращения к
 * mc_interface_set_*().
 */
bool fc_vesc_comm_is_read_only(uint8_t comm_id);
