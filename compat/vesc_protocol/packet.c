#include "packet.h"

#include <string.h>

// CRC-16/XMODEM: poly 0x1021, init 0x0000, без рефлексии и финального XOR.
// Побитовая реализация вместо таблицы: экономит 512 байт и не влияет на
// realtime-путь (протокол работает вне контура баланса).
uint16_t vesc_crc16(const uint8_t *buf, size_t len) {
    uint16_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t) buf[i] << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? (uint16_t) ((crc << 1) ^ 0x1021) : (uint16_t) (crc << 1);
        }
    }
    return crc;
}

void vesc_packet_init(
    VescPacket *p, VescPacketSendFn send, VescPacketProcessFn process, void *ctx
) {
    memset(p, 0, sizeof(*p));
    p->send = send;
    p->process = process;
    p->ctx = ctx;
}

void vesc_packet_reset(VescPacket *p) {
    p->rx_read_ptr = 0;
    p->rx_write_ptr = 0;
    p->bytes_left = 0;
}

size_t vesc_packet_encode(const uint8_t *payload, size_t len, uint8_t *out, size_t out_cap) {
    if (len == 0 || len > VESC_PACKET_MAX_PL_LEN) {
        return 0;
    }

    size_t ind = 0;
    if (len <= 255) {
        if (out_cap < len + 5) {
            return 0;
        }
        out[ind++] = 2;
        out[ind++] = (uint8_t) len;
    } else if (len <= 65535) {
        if (out_cap < len + 6) {
            return 0;
        }
        out[ind++] = 3;
        out[ind++] = (uint8_t) (len >> 8);
        out[ind++] = (uint8_t) (len & 0xFF);
    } else {
        if (out_cap < len + 7) {
            return 0;
        }
        out[ind++] = 4;
        out[ind++] = (uint8_t) (len >> 16);
        out[ind++] = (uint8_t) ((len >> 8) & 0xFF);
        out[ind++] = (uint8_t) (len & 0xFF);
    }

    memcpy(out + ind, payload, len);
    ind += len;

    uint16_t crc = vesc_crc16(payload, len);
    out[ind++] = (uint8_t) (crc >> 8);
    out[ind++] = (uint8_t) (crc & 0xFF);
    out[ind++] = 3;
    return ind;
}

bool vesc_packet_send(VescPacket *p, const uint8_t *payload, size_t len) {
    size_t frame_len = vesc_packet_encode(payload, len, p->tx_buffer, sizeof(p->tx_buffer));
    if (frame_len == 0) {
        ++p->stats.send_rejected;
        return false;
    }
    ++p->stats.frames_sent;
    if (p->send) {
        p->send(p->ctx, p->tx_buffer, frame_len);
    }
    return true;
}

/**
 * Возврат:
 *   >0  — успех, столько байт кадра разобрано
 *   -1  — структура неверна, сдвинуться на байт и попробовать снова
 *   -2  — данных пока мало
 */
static int try_decode(VescPacket *p, uint8_t *buffer, size_t in_len, int *bytes_left) {
    *bytes_left = 0;

    if (in_len == 0) {
        *bytes_left = 1;
        return -2;
    }

    const bool is_len_8b = buffer[0] == 2;
    const bool is_len_16b = buffer[0] == 3;
    const bool is_len_24b = buffer[0] == 4;
    const size_t data_start = buffer[0];

    if (!is_len_8b && !is_len_16b && !is_len_24b) {
        ++p->stats.framing_errors;
        return -1;
    }

    if (in_len < data_start) {
        *bytes_left = (int) (data_start - in_len);
        return -2;
    }

    size_t len = 0;
    if (is_len_8b) {
        len = buffer[1];
        // Пакеты нулевой длины не поддерживаются
        if (len < 1) {
            ++p->stats.framing_errors;
            return -1;
        }
    } else if (is_len_16b) {
        len = ((size_t) buffer[1] << 8) | buffer[2];
        // Более короткий пакет обязан использовать меньше байт длины
        if (len < 255) {
            ++p->stats.framing_errors;
            return -1;
        }
    } else {
        len = ((size_t) buffer[1] << 16) | ((size_t) buffer[2] << 8) | buffer[3];
        if (len < 65535) {
            ++p->stats.framing_errors;
            return -1;
        }
    }

    if (len > VESC_PACKET_MAX_PL_LEN) {
        ++p->stats.oversized;
        return -1;
    }

    if (in_len < len + data_start + 3) {
        *bytes_left = (int) (len + data_start + 3 - in_len);
        return -2;
    }

    if (buffer[data_start + len + 2] != 3) {
        ++p->stats.framing_errors;
        return -1;
    }

    const uint16_t crc_calc = vesc_crc16(buffer + data_start, len);
    const uint16_t crc_rx =
        (uint16_t) ((uint16_t) buffer[data_start + len] << 8 | buffer[data_start + len + 1]);

    if (crc_calc != crc_rx) {
        ++p->stats.crc_errors;
        return -1;
    }

    ++p->stats.frames_ok;
    if (p->process) {
        p->process(p->ctx, buffer + data_start, len);
    }
    return (int) (len + data_start + 3);
}

void vesc_packet_process_byte(VescPacket *p, uint8_t byte) {
    ++p->stats.bytes_in;

    size_t data_len = p->rx_write_ptr - p->rx_read_ptr;

    // Места нет (не должно случаться) — сброс
    if (data_len >= VESC_PACKET_BUFFER_LEN) {
        p->rx_write_ptr = 0;
        p->rx_read_ptr = 0;
        p->bytes_left = 0;
        p->rx_buffer[p->rx_write_ptr++] = byte;
        return;
    }

    // Всё должно быть выровнено: сдвигаем буфер, когда кончилось место
    if (p->rx_write_ptr >= VESC_PACKET_BUFFER_LEN) {
        memmove(p->rx_buffer, p->rx_buffer + p->rx_read_ptr, data_len);
        p->rx_read_ptr = 0;
        p->rx_write_ptr = data_len;
    }

    p->rx_buffer[p->rx_write_ptr++] = byte;
    data_len++;

    if (p->bytes_left > 1) {
        p->bytes_left--;
        return;
    }

    for (;;) {
        int res = try_decode(p, p->rx_buffer + p->rx_read_ptr, data_len, &p->bytes_left);

        if (res == -2) {
            break;
        }

        if (res > 0) {
            data_len -= (size_t) res;
            p->rx_read_ptr += (size_t) res;
        } else if (res == -1) {
            p->rx_read_ptr++;
            data_len--;
        }
    }

    if (data_len == 0) {
        p->rx_read_ptr = 0;
        p->rx_write_ptr = 0;
    }
}

void vesc_packet_process_buffer(VescPacket *p, const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        vesc_packet_process_byte(p, data[i]);
    }
}
