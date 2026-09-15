#include "floatcore_limits.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static FloatCoreLimits L;

void floatcore_limits_init(void) {
    memset(&L, 0, sizeof(L));

    // Пределы самого FloatCore: то, что он не превысит независимо от настроек
    // ESC. Служат верхней границей и работают, даже если ESC ещё не опрошены.
    L.floatcore = (FcSourceLimits){
        .present = true,
        .current_max = 60.0f,
        .current_min = -60.0f,
        .in_current_max = 50.0f,
        .in_current_min = -20.0f,
        .temp_fet_start = 85.0f,
        .temp_fet_end = 100.0f,
        .temp_motor_start = 85.0f,
        .temp_motor_end = 100.0f,
        .max_duty = 0.95f,
    };

    // Число ячеек реальной батареи аппарата: 10S Li-ion. Подтверждено
    // владельцем и согласуется с каждой отсечкой в конфигурации ESC —
    // 34.0/31.0 В это 3.40/3.10 В на ячейку, а regen-отсечка 42.0 В это
    // ровно 4.20 В на ячейку (docs/battery_safety_model.md).
    //
    // Значение НЕ информационное. Refloat умножает на него пороги отката по
    // напряжению, если они заданы на ячейку (refloat-upstream/src/
    // motor_data.c:79-88), поэтому ошибка здесь сдвигает предупреждение
    // водителю в разы.
    L.battery = (FcBatteryConfig){
        .cell_count = FC_BATTERY_CELLS_DEFAULT,
        .cell_v_min = 3.0f,
        .cell_v_max = 4.2f,
    };
}

const FloatCoreLimits *floatcore_limits(void) {
    return &L;
}

void floatcore_limits_set_esc(int index, const FcSourceLimits *limits) {
    if (index < 0 || index >= FC_ESC_COUNT || !limits) {
        return;
    }
    L.esc[index] = *limits;
}

void floatcore_limits_set_floatcore(const FcSourceLimits *limits) {
    if (limits) {
        L.floatcore = *limits;
    }
}

void floatcore_limits_set_battery(const FcBatteryConfig *battery) {
    if (battery) {
        L.battery = *battery;
    }
}

// ----------------------------------------------------------------- агрегация

typedef enum { PICK_MIN, PICK_MAX } PickRule;

/** Обход всех присутствующих источников с выбором по правилу. */
static float aggregate(size_t offset, PickRule rule) {
    const FcSourceLimits *sources[FC_ESC_COUNT + 1] = {
        &L.esc[0], &L.esc[1], &L.floatcore
    };

    float result = NAN;
    for (size_t i = 0; i < FC_ESC_COUNT + 1; ++i) {
        if (!sources[i]->present) {
            continue;
        }
        float v = *(const float *) ((const uint8_t *) sources[i] + offset);
        if (!isfinite(v)) {
            continue;
        }
        if (!isfinite(result)) {
            result = v;
        } else if (rule == PICK_MIN ? v < result : v > result) {
            result = v;
        }
    }
    return isfinite(result) ? result : 0.0f;
}

#define AGG(field, rule) aggregate(offsetof(FcSourceLimits, field), rule)

float fc_effective_current_max(void) {
    return AGG(current_max, PICK_MIN);
}

float fc_effective_current_min(void) {
    // Значения отрицательные, поэтому «самое консервативное» — максимум,
    // то есть ближайшее к нулю.
    return AGG(current_min, PICK_MAX);
}

float fc_effective_in_current_max(void) {
    return AGG(in_current_max, PICK_MIN);
}

float fc_effective_in_current_min(void) {
    return AGG(in_current_min, PICK_MAX);
}

float fc_effective_temp_fet_start(void) {
    return AGG(temp_fet_start, PICK_MIN);
}

float fc_effective_temp_fet_end(void) {
    return AGG(temp_fet_end, PICK_MIN);
}

float fc_effective_temp_motor_start(void) {
    return AGG(temp_motor_start, PICK_MIN);
}

float fc_effective_temp_motor_end(void) {
    return AGG(temp_motor_end, PICK_MIN);
}

float fc_effective_max_duty(void) {
    return AGG(max_duty, PICK_MIN);
}

uint8_t fc_battery_cell_count(void) {
    // Ноль здесь недопустим. Refloat умножает пороги отката на число ячеек
    // только если оно больше нуля (motor_data.c:80), иначе он трактует
    // «3.0 В» как абсолютный порог для всей батареи — то есть откат по
    // низкому напряжению не сработает никогда. Забыть вызвать init нельзя,
    // но если это случится, отказ обязан быть громким, а не тихим.
    return L.battery.cell_count ? L.battery.cell_count : FC_BATTERY_CELLS_DEFAULT;
}

float fc_battery_v_min(void) {
    return L.battery.cell_v_min;
}

float fc_battery_v_max(void) {
    return L.battery.cell_v_max;
}
