#include "fc_can_diag.h"

#if FC_CAN_DIAG_TX_AVAILABLE

#include "../vesc_protocol/packet.h" // vesc_crc16 — та же CRC, что у bldc

#include <string.h>

// Таблица запросов. Единственное место в проекте, где номер типа пакета CAN и
// номер COMM появляются рядом со смыслом запроса.
//
// Поле comm_id = 0xFF означает, что запрос не пользуется COMM-уровнем (PING
// работает собственным типом пакета).
static const struct {
    const char *name;
    uint32_t packet_type;
    uint8_t comm_id;
} REQ[FC_CAN_DIAG_REQUEST_COUNT] = {
    [FC_CAN_DIAG_PING] = {"PING", FC_CAN_PACKET_PING, 0xFFu},
    [FC_CAN_DIAG_FW_VERSION] = {"FW_VERSION", FC_CAN_PACKET_PROCESS_SHORT_BUFFER,
                                FC_COMM_FW_VERSION},
    [FC_CAN_DIAG_VALUES] = {"GET_VALUES", FC_CAN_PACKET_PROCESS_SHORT_BUFFER, FC_COMM_GET_VALUES},
    [FC_CAN_DIAG_MCCONF] = {"GET_MCCONF", FC_CAN_PACKET_PROCESS_SHORT_BUFFER, FC_COMM_GET_MCCONF},
    [FC_CAN_DIAG_APPCONF] = {"GET_APPCONF", FC_CAN_PACKET_PROCESS_SHORT_BUFFER,
                             FC_COMM_GET_APPCONF},
};

// Режим ответа для PROCESS_SHORT_BUFFER, bldc comm_can.c:1749-1787.
//
//   0 — обработать пакет и отправить ответ обратно тому, кто прислал;
//   1 — переслать дальше без обработки;
//   2 — обработать молча, ответа не будет;
//   3 — обработать и ответить без обёртки.
//
// Нужен ровно 0: запрос должен быть обработан и ответ должен вернуться нам.
#define FC_CAN_SEND_MODE_PROCESS_AND_REPLY 0u

const char *fc_can_diag_request_name(FcCanDiagRequest req) {
    if ((unsigned) req >= FC_CAN_DIAG_REQUEST_COUNT) {
        return "?";
    }
    return REQ[req].name;
}

uint8_t fc_can_diag_request_comm_id(FcCanDiagRequest req) {
    if ((unsigned) req >= FC_CAN_DIAG_REQUEST_COUNT) {
        return 0xFFu;
    }
    return REQ[req].comm_id;
}

uint32_t fc_can_diag_request_packet_type(FcCanDiagRequest req) {
    if ((unsigned) req >= FC_CAN_DIAG_REQUEST_COUNT) {
        return 0xFFFFFFFFu;
    }
    return REQ[req].packet_type;
}

FcCanDiagBuildResult fc_can_diag_build(FcCanDiagRequest req, uint8_t self_id, uint8_t target_id,
                                       FcCanDiagFrame *out) {
    if (!out || (unsigned) req >= FC_CAN_DIAG_REQUEST_COUNT) {
        return FC_CAN_DIAG_BUILD_UNKNOWN_REQUEST;
    }
    if (target_id == 255u) {
        // На широковещательный адрес ответят обе половины, и разобрать, чей
        // ответ пришёл первым, будет нечем. Сканирование шины — не задача
        // этого слоя.
        return FC_CAN_DIAG_BUILD_BROADCAST;
    }
    if (target_id == self_id) {
        return FC_CAN_DIAG_BUILD_SELF;
    }

    memset(out, 0, sizeof(*out));
    out->eid = (uint32_t) target_id | (REQ[req].packet_type << 8);

    if (req == FC_CAN_DIAG_PING) {
        // bldc comm_can.c:1819-1825: получатель отвечает PONG на адрес
        // data8[0], поэтому в кадре лежит наш собственный номер.
        out->data[0] = self_id;
        out->len = 1;
        return FC_CAN_DIAG_BUILD_OK;
    }

    // bldc comm_can.c:1749-1777: [кому отвечать][режим][пакет COMM…]
    out->data[0] = self_id;
    out->data[1] = FC_CAN_SEND_MODE_PROCESS_AND_REPLY;
    out->data[2] = REQ[req].comm_id;
    out->len = 3;
    return FC_CAN_DIAG_BUILD_OK;
}

// ---------------------------------------------------------------- приём

void fc_can_diag_rx_init(FcCanDiagRx *rx, uint8_t self_id) {
    memset(rx, 0, sizeof(*rx));
    rx->self_id = self_id;
}

void fc_can_diag_rx_abort(FcCanDiagRx *rx) {
    rx->asm_filled = 0;
    rx->complete = false;
    rx->payload_len = 0;
}

static FcCanDiagRxResult fill(FcCanDiagRx *rx, uint16_t offset, const uint8_t *data, uint8_t len) {
    if ((uint32_t) offset + len > FC_CAN_DIAG_RX_MAX) {
        ++rx->overflows;
        rx->asm_filled = 0;
        return FC_CAN_DIAG_RX_OVERFLOW;
    }
    memcpy(rx->asm_buf + offset, data, len);
    uint16_t end = (uint16_t) (offset + len);
    if (end > rx->asm_filled) {
        rx->asm_filled = end;
    }
    return FC_CAN_DIAG_RX_PARTIAL;
}

FcCanDiagRxResult fc_can_diag_rx_frame(FcCanDiagRx *rx, uint32_t eid, const uint8_t *data,
                                       uint8_t len) {
    uint8_t addressee = (uint8_t) (eid & 0xFFu);
    uint32_t type = eid >> 8;

    if (type != FC_CAN_PACKET_PONG && type != FC_CAN_PACKET_FILL_RX_BUFFER &&
        type != FC_CAN_PACKET_FILL_RX_BUFFER_LONG && type != FC_CAN_PACKET_PROCESS_RX_BUFFER &&
        type != FC_CAN_PACKET_PROCESS_SHORT_BUFFER) {
        return FC_CAN_DIAG_RX_IGNORED;
    }
    if (addressee != rx->self_id) {
        // Половины Dual FSESC разговаривают между собой теми же типами
        // пакетов. Чужой обмен считаем и не трогаем.
        ++rx->foreign;
        return FC_CAN_DIAG_RX_IGNORED;
    }

    switch (type) {
    case FC_CAN_PACKET_PONG:
        // bldc comm_can.c:1819-1824: [номер ответившего][тип железа]
        if (len < 1) {
            return FC_CAN_DIAG_RX_IGNORED;
        }
        rx->from_id = data[0];
        rx->payload_len = len;
        memcpy(rx->payload, data, len);
        rx->complete = true;
        ++rx->pongs;
        return FC_CAN_DIAG_RX_COMPLETE;

    case FC_CAN_PACKET_PROCESS_SHORT_BUFFER:
        // [кто ответил][режим][полезная нагрузка…]
        if (len < 2) {
            return FC_CAN_DIAG_RX_IGNORED;
        }
        rx->from_id = data[0];
        rx->payload_len = (uint16_t) (len - 2);
        memcpy(rx->payload, data + 2, rx->payload_len);
        rx->complete = true;
        ++rx->short_replies;
        return FC_CAN_DIAG_RX_COMPLETE;

    case FC_CAN_PACKET_FILL_RX_BUFFER:
        if (len < 1) {
            return FC_CAN_DIAG_RX_IGNORED;
        }
        return fill(rx, data[0], data + 1, (uint8_t) (len - 1));

    case FC_CAN_PACKET_FILL_RX_BUFFER_LONG:
        if (len < 2) {
            return FC_CAN_DIAG_RX_IGNORED;
        }
        return fill(rx, (uint16_t) (((uint16_t) data[0] << 8) | data[1]), data + 2,
                    (uint8_t) (len - 2));

    case FC_CAN_PACKET_PROCESS_RX_BUFFER: {
        // [кто ответил][режим][len_hi][len_lo][crc_hi][crc_lo]
        if (len < 6) {
            return FC_CAN_DIAG_RX_IGNORED;
        }
        uint16_t total = (uint16_t) (((uint16_t) data[2] << 8) | data[3]);
        uint16_t crc = (uint16_t) (((uint16_t) data[4] << 8) | data[5]);
        if (total > FC_CAN_DIAG_RX_MAX || total > rx->asm_filled) {
            ++rx->overflows;
            rx->asm_filled = 0;
            return FC_CAN_DIAG_RX_OVERFLOW;
        }
        if (vesc_crc16(rx->asm_buf, total) != crc) {
            ++rx->crc_errors;
            rx->asm_filled = 0;
            return FC_CAN_DIAG_RX_CRC_ERROR;
        }
        rx->from_id = data[0];
        rx->payload_len = total;
        memcpy(rx->payload, rx->asm_buf, total);
        rx->asm_filled = 0;
        rx->complete = true;
        ++rx->long_replies;
        return FC_CAN_DIAG_RX_COMPLETE;
    }

    default:
        return FC_CAN_DIAG_RX_IGNORED;
    }
}

// ------------------------------------------------------------ темп запросов

void fc_can_diag_rate_init(FcCanDiagRate *r, uint32_t min_interval_us) {
    memset(r, 0, sizeof(*r));
    r->min_interval_us = min_interval_us;
}

bool fc_can_diag_rate_allow(FcCanDiagRate *r, uint64_t now_us) {
    if (r->armed && now_us - r->last_us < r->min_interval_us) {
        ++r->throttled;
        return false;
    }
    r->last_us = now_us;
    r->armed = true;
    ++r->allowed;
    return true;
}

#else

// В профиле без диагностической передачи файл пуст по построению. Пустая
// единица трансляции — не ошибка, но и не то, что стоит оставлять молча:
// объявление ниже делает намерение видимым в объектном файле.
typedef int fc_can_diag_not_available_in_this_profile;

#endif // FC_CAN_DIAG_TX_AVAILABLE
