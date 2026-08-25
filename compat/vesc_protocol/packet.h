// Оригинальный framing пакетов VESC.
//
// Реализация повторяет bldc/comm/packet.c: тот же формат кадра, тот же CRC16
// (CCITT/XMODEM, poly 0x1021, init 0), та же логика повторного разбора со сдвигом.
//
// Никаких зависимостей от платформы: только C99 и <stdint.h>/<stdbool.h>/<stddef.h>.
// Один и тот же код собирается на host, на ESP32 и в юнит-тестах.
//
// Формат кадра:
//   [2][len8]                 payload  [crc_hi][crc_lo][3]     len <= 255
//   [3][len_hi][len_lo]       payload  [crc_hi][crc_lo][3]     255 <= len <= 65535
//   [4][len_hi][len_mid][len_lo] payload [crc_hi][crc_lo][3]   len > 65535
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef VESC_PACKET_MAX_PL_LEN
// То же значение, что в прошивке VESC по умолчанию.
#define VESC_PACKET_MAX_PL_LEN 512
#endif

#define VESC_PACKET_BUFFER_LEN (VESC_PACKET_MAX_PL_LEN + 8)

typedef struct VescPacket VescPacket;

/** Отправка готового кадра в транспорт. */
typedef void (*VescPacketSendFn)(void *ctx, const uint8_t *data, size_t len);

/** Доставка разобранного payload наверх. */
typedef void (*VescPacketProcessFn)(void *ctx, const uint8_t *payload, size_t len);

/** Счётчики для диагностики и негативных тестов. */
typedef struct {
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t framing_errors;  // неверный стартовый/стоповый байт, недопустимая длина
    uint32_t oversized;       // заявленная длина больше VESC_PACKET_MAX_PL_LEN
    uint32_t bytes_in;
    uint32_t frames_sent;
    uint32_t send_rejected;   // попытка отправить пустой или слишком длинный payload
} VescPacketStats;

struct VescPacket {
    VescPacketSendFn send;
    VescPacketProcessFn process;
    void *ctx;

    size_t rx_read_ptr;
    size_t rx_write_ptr;
    int bytes_left;

    uint8_t rx_buffer[VESC_PACKET_BUFFER_LEN];
    uint8_t tx_buffer[VESC_PACKET_BUFFER_LEN];

    VescPacketStats stats;
};

uint16_t vesc_crc16(const uint8_t *buf, size_t len);

void vesc_packet_init(VescPacket *p, VescPacketSendFn send, VescPacketProcessFn process, void *ctx);
void vesc_packet_reset(VescPacket *p);

/** Скормить один принятый байт. Никогда не выходит за границы буфера. */
void vesc_packet_process_byte(VescPacket *p, uint8_t byte);

/** Скормить блок байт. */
void vesc_packet_process_buffer(VescPacket *p, const uint8_t *data, size_t len);

/**
 * Упаковать и отправить payload.
 * Возвращает false, если payload пустой или длиннее VESC_PACKET_MAX_PL_LEN
 * (кадр не отправляется — ровно как в прошивке).
 */
bool vesc_packet_send(VescPacket *p, const uint8_t *payload, size_t len);

/**
 * Собрать кадр в предоставленный буфер без отправки (для тестов).
 * Возвращает длину кадра или 0.
 */
size_t vesc_packet_encode(const uint8_t *payload, size_t len, uint8_t *out, size_t out_cap);
