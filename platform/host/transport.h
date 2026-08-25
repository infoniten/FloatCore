// Абстракция транспорта.
//
// Протокольный слой не знает, что под ним: USB Serial, UART, TCP или BLE.
// На этом этапе реализован только TCP (platform/host/tcp_transport.c) —
// его достаточно, чтобы подключить настоящий VESC Tool.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct Transport Transport;

struct Transport {
    const char *name;

    /** Дождаться подключения клиента. Блокирующий. false — ошибка/остановка. */
    bool (*accept)(Transport *t);

    /** Прочитать доступные байты. >0 — количество, 0 — нет данных, <0 — разрыв. */
    int (*recv)(Transport *t, uint8_t *buf, size_t cap);

    /** Отправить байты. false — разрыв. */
    bool (*send)(Transport *t, const uint8_t *data, size_t len);

    /** Закрыть текущее соединение (но не сам транспорт). */
    void (*disconnect)(Transport *t);

    /** Полностью освободить ресурсы. */
    void (*destroy)(Transport *t);

    bool (*is_connected)(Transport *t);

    void *impl;
};

/** TCP-сервер на указанном порту. VESC Tool: Connection → TCP. */
Transport *tcp_transport_create(uint16_t port);
