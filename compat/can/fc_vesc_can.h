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
