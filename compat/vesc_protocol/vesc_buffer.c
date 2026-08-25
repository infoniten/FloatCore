#include "vesc_buffer.h"

#include <math.h>
#include <string.h>

void vb_append_int8(uint8_t *b, int8_t v, size_t *i) {
    b[(*i)++] = (uint8_t) v;
}

void vb_append_uint8(uint8_t *b, uint8_t v, size_t *i) {
    b[(*i)++] = v;
}

void vb_append_int16(uint8_t *b, int16_t v, size_t *i) {
    vb_append_uint16(b, (uint16_t) v, i);
}

void vb_append_uint16(uint8_t *b, uint16_t v, size_t *i) {
    b[(*i)++] = (uint8_t) (v >> 8);
    b[(*i)++] = (uint8_t) (v & 0xFF);
}

void vb_append_int32(uint8_t *b, int32_t v, size_t *i) {
    vb_append_uint32(b, (uint32_t) v, i);
}

void vb_append_uint32(uint8_t *b, uint32_t v, size_t *i) {
    b[(*i)++] = (uint8_t) (v >> 24);
    b[(*i)++] = (uint8_t) (v >> 16);
    b[(*i)++] = (uint8_t) (v >> 8);
    b[(*i)++] = (uint8_t) (v & 0xFF);
}

// Нефинитные значения передаются как 0: VESC Tool отобразил бы NaN как мусор,
// а сама возможность передать NaN наружу противоречит требованиям безопасности.
static float sanitize(float v) {
    return isfinite(v) ? v : 0.0f;
}

void vb_append_float16(uint8_t *b, float v, float scale, size_t *i) {
    vb_append_int16(b, (int16_t) lrintf(sanitize(v) * scale), i);
}

void vb_append_float32(uint8_t *b, float v, float scale, size_t *i) {
    vb_append_int32(b, (int32_t) lrintf(sanitize(v) * scale), i);
}

void vb_append_string(uint8_t *b, const char *s, size_t *i) {
    size_t n = strlen(s);
    memcpy(b + *i, s, n + 1);
    *i += n + 1;
}

int8_t vb_get_int8(const uint8_t *b, size_t *i) {
    return (int8_t) b[(*i)++];
}

uint8_t vb_get_uint8(const uint8_t *b, size_t *i) {
    return b[(*i)++];
}

int16_t vb_get_int16(const uint8_t *b, size_t *i) {
    return (int16_t) vb_get_uint16(b, i);
}

uint16_t vb_get_uint16(const uint8_t *b, size_t *i) {
    uint16_t v = (uint16_t) ((uint16_t) b[*i] << 8 | b[*i + 1]);
    *i += 2;
    return v;
}

int32_t vb_get_int32(const uint8_t *b, size_t *i) {
    return (int32_t) vb_get_uint32(b, i);
}

uint32_t vb_get_uint32(const uint8_t *b, size_t *i) {
    uint32_t v = (uint32_t) b[*i] << 24 | (uint32_t) b[*i + 1] << 16 | (uint32_t) b[*i + 2] << 8 |
        (uint32_t) b[*i + 3];
    *i += 4;
    return v;
}
