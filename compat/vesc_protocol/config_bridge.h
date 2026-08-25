// Мост конфигурации: единственный источник истины — конфигурация Refloat.
//
// Второй системы параметров не существует. VESC Tool получает ту же самую
// XML-схему (settings.xml Refloat) и тот же самый сериализованный блоб, что
// Refloat пишет в постоянную память. Реализация коллбэков в платформенном
// слое просто проксирует их в conf_custom_add_config(), которую Refloat
// зарегистрировал при старте.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    /** Сжатый XML описания параметров. Возвращает длину, data — указатель на данные. */
    int (*get_xml)(void *ctx, int conf_ind, const uint8_t **data);

    /** Текущая (или дефолтная) конфигурация в буфер. Возвращает длину или <0. */
    int (*get)(void *ctx, int conf_ind, uint8_t *buf, size_t cap, bool is_default);

    /** Применить конфигурацию. Возвращает false при отказе. */
    bool (*set)(void *ctx, int conf_ind, const uint8_t *buf, size_t len);

    void *ctx;
    int config_count;
} ConfigBridge;

/** Кодирует ответ на COMM_GET_CUSTOM_CONFIG_XML (чанк). Возвращает длину или 0. */
size_t config_bridge_encode_xml_chunk(
    const ConfigBridge *cb, int conf_ind, int32_t req_len, int32_t req_offset, uint8_t *out,
    size_t cap
);

/** Кодирует ответ на COMM_GET_CUSTOM_CONFIG(_DEFAULT). Возвращает длину или 0. */
size_t config_bridge_encode_config(
    const ConfigBridge *cb, int conf_ind, bool is_default, uint8_t *out, size_t cap
);
