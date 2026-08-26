// Входы RT App: COMM_GET_DECODED_PPM / _ADC / _CHUK.
//
// VESC Tool открывает страницу RT App и начинает периодически опрашивать
// декодированные значения трёх приложений ввода: PPM (радиоприёмник),
// ADC (педали газа/тормоза) и Nunchuk. FloatCore ни одного из этих приложений
// не имеет, но обязан отвечать: иначе счётчик неизвестных команд растёт, а
// страница остаётся пустой.
//
// Формат ответов взят из bldc/comm/commands.c (case COMM_GET_DECODED_*) и
// сверен с vesc_tool/commands.cpp (processPacket): каждое значение — int32
// big-endian со шкалой 1e6, порядок полей фиксирован.
//
//   COMM_GET_DECODED_PPM   [31][level][pulse_len]                   9 байт
//   COMM_GET_DECODED_ADC   [32][level][voltage][level2][voltage2]  17 байт
//   COMM_GET_DECODED_CHUK  [33][level]                              5 байт
//
// Путь строго read-only: значения только сообщаются наружу. Ни один из
// обработчиков не трогает конфигурацию, состояние Refloat и мотор.
#pragma once

#include <stddef.h>
#include <stdint.h>

/** Длины ответов, включая байт команды. Заданы форматом, а не нашим выбором. */
#define DECODED_PPM_REPLY_LEN 9
#define DECODED_ADC_REPLY_LEN 17
#define DECODED_CHUK_REPLY_LEN 5

/** Полная шкала АЦП VESC: 0…3.3 В. Используется для нормировки level. */
#define DECODED_ADC_FULL_SCALE_V 3.3f

typedef struct {
    /** Ход ручки газа PPM, −1…1. У FloatCore приёмника нет: всегда 0. */
    float ppm_level;
    /** Длительность последнего импульса PPM, секунды. Нет импульсов — 0. */
    float ppm_pulse_len;

    /** Канал 1 (вывод ADC1): доля полной шкалы 0…1 и напряжение в вольтах. */
    float adc_level;
    float adc_voltage;
    /** Канал 2 (вывод ADC2). */
    float adc_level2;
    float adc_voltage2;

    /** Ось Y нунчака, −1…1. Нунчака нет: всегда 0 — нейтраль. */
    float chuk_level;
} VescDecodedInputs;

/**
 * Нейтральные значения: никакого ввода нет.
 *
 * Это же — безопасное значение по умолчанию, если провайдер не задан:
 * ноль по всем осям означает «газ отпущен», а не «газ в пол».
 */
void decoded_inputs_neutral(VescDecodedInputs *out);

/**
 * Заполнить ADC-каналы из напряжений на выводах ADC1/ADC2 — тех самых, что
 * читает footpad_sensor.c Refloat через VESC_IF->io_read_analog().
 *
 * Порядок каналов — по выводам (ADC1 → канал 1), а не по «левой/правой»
 * педали: соответствие вывода педали задаёт настройка Refloat
 * `hardware.swap_footpad_adcs` и меняется вместе с ней, а страница RT App
 * показывает именно выводы (Voltage / Voltage 2).
 */
void decoded_inputs_from_adc(float adc1_volts, float adc2_volts, VescDecodedInputs *out);

/** Кодирование ответов. Возвращают длину или 0, если не хватило места. */
size_t decoded_ppm_encode(const VescDecodedInputs *in, uint8_t *out, size_t cap);
size_t decoded_adc_encode(const VescDecodedInputs *in, uint8_t *out, size_t cap);
size_t decoded_chuk_encode(const VescDecodedInputs *in, uint8_t *out, size_t cap);
