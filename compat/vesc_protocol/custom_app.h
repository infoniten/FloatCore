// COMM_CUSTOM_APP_DATA — канал между QML-интерфейсом Refloat и самим Refloat.
//
// FloatCore здесь только транспорт: он не разбирает и не изменяет содержимое.
// Все ~25 команд протокола Refloat (doc/commands/ в upstream) уже реализованы
// внутри Refloat, их не нужно дублировать.
//
//   QML  --sendCustomAppData-->  VESC Tool  --COMM_CUSTOM_APP_DATA-->  FloatCore
//        --> set_app_data_handler() Refloat
//
//   Refloat --send_app_data()--> FloatCore --COMM_CUSTOM_APP_DATA--> VESC Tool --> QML
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    /** Передать данные в прошивку (обработчик, зарегистрированный Refloat). */
    void (*to_firmware)(void *ctx, const uint8_t *data, size_t len);
    void *ctx;
} CustomAppBridge;

/**
 * Обернуть данные от прошивки в payload COMM_CUSTOM_APP_DATA.
 * Возвращает длину или 0, если данные не помещаются в пакет.
 */
size_t custom_app_encode(const uint8_t *data, size_t len, uint8_t *out, size_t cap);
