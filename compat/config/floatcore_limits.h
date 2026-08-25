// Единственный источник истины по ограничениям FloatCore.
//
// Здесь живут пределы, которые Refloat читает через VESC_IF->get_cfg_float()/
// get_cfg_int(), и из которых Virtual mcConfig строит своё представление для
// VESC Tool. Копий этих значений больше нигде нет: и балансировка, и интерфейс
// смотрят в одну и ту же структуру.
//
//     ESC A ─┐
//     ESC B ─┼─► агрегация (самое консервативное) ─► FloatCoreLimits
//  FloatCore ┘                                            │
//                                    ┌───────────────────┴────────────────┐
//                                    ▼                                    ▼
//                     VESC_IF->get_cfg_float()              Virtual mcConfig
//                          (Refloat)                          (VESC Tool UI)
//
// На host источники — mock. На ESP32 те же поля заполняются из конфигурации,
// вычитанной с реальных FSESC по CAN, — интерфейс при этом не меняется.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define FC_ESC_COUNT 2

/** Пределы одного источника: физического ESC либо самого FloatCore. */
typedef struct {
    bool present;  // false — источник не учитывается в агрегации

    float current_max;      // А, > 0
    float current_min;      // А, < 0 (тормозной)
    float in_current_max;   // А, > 0
    float in_current_min;   // А, < 0 (рекуперация)
    float temp_fet_start;   // °C
    float temp_fet_end;     // °C
    float temp_motor_start; // °C
    float temp_motor_end;   // °C
    float max_duty;         // 0..1
} FcSourceLimits;

/** Конфигурация батареи. */
typedef struct {
    uint8_t cell_count;
    float cell_v_min;
    float cell_v_max;
} FcBatteryConfig;

typedef struct {
    FcSourceLimits esc[FC_ESC_COUNT];  // пределы физических ESC (из CAN)
    FcSourceLimits floatcore;          // собственные пределы FloatCore
    FcBatteryConfig battery;
} FloatCoreLimits;

/** Инициализация значениями по умолчанию. */
void floatcore_limits_init(void);

/** Прямой доступ для диагностики. Менять только через сеттеры. */
const FloatCoreLimits *floatcore_limits(void);

void floatcore_limits_set_esc(int index, const FcSourceLimits *limits);
void floatcore_limits_set_floatcore(const FcSourceLimits *limits);
void floatcore_limits_set_battery(const FcBatteryConfig *battery);

// --------------------------------------------------------------- агрегация
//
// Правило одно для всех пределов: **самое консервативное значение**.
// Ничего не суммируется — ни токи, ни температуры. Обоснование по каждому
// параметру: docs/mcconfig_mapping.md.

/** min по всем присутствующим источникам. */
float fc_effective_current_max(void);

/** Ближайшее к нулю (то есть наименьшее по модулю) отрицательное значение. */
float fc_effective_current_min(void);

float fc_effective_in_current_max(void);
float fc_effective_in_current_min(void);

/** min: перегрев любого источника — повод снижать мощность. */
float fc_effective_temp_fet_start(void);
float fc_effective_temp_fet_end(void);
float fc_effective_temp_motor_start(void);
float fc_effective_temp_motor_end(void);

float fc_effective_max_duty(void);

uint8_t fc_battery_cell_count(void);
float fc_battery_v_min(void);
float fc_battery_v_max(void);
