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

    L.battery = (FcBatteryConfig){
        .cell_count = 20,
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
    return L.battery.cell_count;
}

float fc_battery_v_min(void) {
    return L.battery.cell_v_min;
}

float fc_battery_v_max(void) {
    return L.battery.cell_v_max;
}
