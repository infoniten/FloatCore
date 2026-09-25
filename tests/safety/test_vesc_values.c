// Разбор телеметрии VESC (ТЗ v0.9J §11, §25).
//
// Главное, что проверяется: смещения отсчитаны от байта COMM. Ровно на этом
// месте на v0.9A была ошибка в один байт.

#include "../../compat/can/fc_vesc_values.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

void fc_test_check(bool ok, const char *what);
void fc_test_note(const char *fmt, ...);
#define check fc_test_check
#define note fc_test_note

static void put16(uint8_t *p, int16_t v) {
    p[0] = (uint8_t) ((uint16_t) v >> 8);
    p[1] = (uint8_t) v;
}

static void put32(uint8_t *p, int32_t v) {
    p[0] = (uint8_t) ((uint32_t) v >> 24);
    p[1] = (uint8_t) ((uint32_t) v >> 16);
    p[2] = (uint8_t) ((uint32_t) v >> 8);
    p[3] = (uint8_t) v;
}

// Синтетический ответ с известными значениями, от байта COMM.
static size_t synth(uint8_t *b) {
    memset(b, 0, 70);
    b[0] = FC_COMM_GET_VALUES_ID;
    put16(b + 1, 312);      // 31.2 °C
    put16(b + 3, -845);     // -84.5 °C: термистор не подключён
    put32(b + 5, 47);       // 0.47 А
    put32(b + 9, -3);       // -0.03 А
    put16(b + 21, 12);      // 0.012
    put32(b + 23, -1530);   // ERPM
    put16(b + 27, 405);     // 40.5 В
    put32(b + 45, 123456);  // тахометр
    put32(b + 49, 234567);
    b[53] = 0;              // FAULT_CODE_NONE
    return 60;
}

static void test_values_from_comm_byte(void) {
    note("GET_VALUES: поля на местах, смещения от байта COMM");
    uint8_t b[70];
    size_t n = synth(b);
    FcVescValues v = fc_vesc_values_decode(b, n);
    check(v.valid, "ответ разобран");
    check(fabsf(v.v_in - 40.5f) < 0.01f, "напряжение 40.5 В");
    check(fabsf(v.current_motor_a - 0.47f) < 0.001f, "ток мотора 0.47 А");
    check(v.erpm == -1530, "ERPM со знаком");
    check(fabsf(v.duty - 0.012f) < 1e-4f, "скважность 0.012");
    check(v.tachometer == 123456, "тахометр");
    check(fabsf(v.temp_motor_c + 84.5f) < 0.01f, "отрицательная температура не ломает разбор");
    check(v.has_fault && v.fault_code == 0, "код отказа NONE");
}

static void test_values_without_comm_byte(void) {
    note("GET_VALUES без байта COMM: сдвиг распознан, а не принят молча");
    uint8_t b[70];
    size_t n = synth(b);
    FcVescValues v = fc_vesc_values_decode(b + 1, n - 1);
    check(v.valid, "тело без байта COMM разобрано");
    check(fabsf(v.v_in - 40.5f) < 0.01f, "то же напряжение");
    check(v.tachometer == 123456, "тот же тахометр");
}

static void test_values_rejects_garbage(void) {
    note("неправдоподобный ответ не выдаёт сдвинутых чисел");
    uint8_t b[70];
    memset(b, 0xFF, sizeof(b));
    FcVescValues v = fc_vesc_values_decode(b, 60);
    check(!v.valid, "мусор отвергнут");
    FcVescValues s = fc_vesc_values_decode(b, 20);
    check(!s.valid, "короткий ответ отвергнут");
}

static void test_status1(void) {
    note("кадр STATUS: ERPM, ток ×10, скважность ×1000");
    uint8_t d[8];
    put32(d, -2400);
    put16(d + 4, 5);     // 0.5 А
    put16(d + 6, -21);   // -0.021
    FcVescStatus1 s = fc_vesc_status1_decode(d, 8);
    check(s.valid, "кадр разобран");
    check(s.erpm == -2400, "ERPM");
    check(fabsf(s.current_a - 0.5f) < 1e-4f, "ток 0.5 А");
    check(fabsf(s.duty + 0.021f) < 1e-5f, "скважность со знаком");
    check(!fc_vesc_status1_decode(d, 6).valid, "короткий кадр отвергнут");
}

static void test_fault_names(void) {
    note("имена кодов отказа");
    check(!strcmp(fc_vesc_fault_name(0), "NONE"), "0 — NONE");
    check(!strcmp(fc_vesc_fault_name(2), "UNDER_VOLTAGE"), "2 — UNDER_VOLTAGE");
    check(!strcmp(fc_vesc_fault_name(250), "?"), "неизвестный код не выдумывается");
}

static void test_appconf_timeout(void) {
    note("таймаут команды из appconf, с проверкой выравнивания по номеру половины");
    uint8_t b[32];
    memset(b, 0, sizeof(b));
    b[0] = FC_COMM_GET_APPCONF_ID;
    b[1 + 4] = 118;               // controller_id
    put32(b + 1 + 5, 50);         // timeout_msec
    // timeout_brake_current = 0.0: в float32_auto это нули
    FcVescAppTimeout t = fc_vesc_appconf_timeout(b, sizeof(b), 118);
    check(t.valid, "ответ разобран");
    check(t.timeout_ms == 50, "таймаут 50 мс");
    check(t.timeout_brake_current_a == 0.0f, "тормозной ток по таймауту 0 — выбег");
    FcVescAppTimeout w = fc_vesc_appconf_timeout(b, sizeof(b), 100);
    check(!w.valid, "ответ ЧУЖОЙ половины отвергнут");
    FcVescAppTimeout s = fc_vesc_appconf_timeout(b + 1, sizeof(b) - 1, 118);
    check(s.valid && s.timeout_ms == 50, "тело без байта COMM тоже разбирается");
}

void test_vesc_values_all(void) {
    test_appconf_timeout();
    test_values_from_comm_byte();
    test_values_without_comm_byte();
    test_values_rejects_garbage();
    test_status1();
    test_fault_names();
}
