// Сериализация чисел в формате VESC (big-endian), как в bldc/util/buffer.c.
// Собственная реализация, чтобы протокольный слой не зависел ни от Refloat,
// ни от платформы.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void vb_append_int8(uint8_t *b, int8_t v, size_t *i);
void vb_append_uint8(uint8_t *b, uint8_t v, size_t *i);
void vb_append_int16(uint8_t *b, int16_t v, size_t *i);
void vb_append_uint16(uint8_t *b, uint16_t v, size_t *i);
void vb_append_int32(uint8_t *b, int32_t v, size_t *i);
void vb_append_uint32(uint8_t *b, uint32_t v, size_t *i);

/** Масштабированное 16-битное значение (VESC Tool: vbPopFrontDouble16). */
void vb_append_float16(uint8_t *b, float v, float scale, size_t *i);
/** Масштабированное 32-битное значение (VESC Tool: vbPopFrontDouble32). */
void vb_append_float32(uint8_t *b, float v, float scale, size_t *i);

/** Строка с завершающим нулём. */
void vb_append_string(uint8_t *b, const char *s, size_t *i);

int8_t vb_get_int8(const uint8_t *b, size_t *i);
uint8_t vb_get_uint8(const uint8_t *b, size_t *i);
int16_t vb_get_int16(const uint8_t *b, size_t *i);
uint16_t vb_get_uint16(const uint8_t *b, size_t *i);
int32_t vb_get_int32(const uint8_t *b, size_t *i);
uint32_t vb_get_uint32(const uint8_t *b, size_t *i);
