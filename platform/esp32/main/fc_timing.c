// Статистика периодичности задач FloatCore (ТЗ v0.5 §10).
//
// Измеряется монотонным таймером высокого разрешения esp_timer (микросекунды,
// 64 бита, без переполнения за время жизни прошивки). Ни одна итерация не
// печатается: печатает только консоль по запросу.

#include "fc_platform.h"

#include "esp_timer.h"

#include <stdbool.h>

#include <string.h>

// Итерация считается опоздавшей, если период превысил номинал более чем на 20 %.
// 20 % при 2000 мкс — это 400 мкс; меньший порог на tick-based планировщике
// FreeRTOS (1 мс) давал бы ложные срабатывания на границе тика.
#define FC_TIMING_LATE_NUM 12
#define FC_TIMING_LATE_DEN 10

static FcTimingStats g_stats[FC_TIMING_COUNT];
static uint64_t g_last_us[FC_TIMING_COUNT];
// Отдельный флаг, а не g_last_us == 0: сразу после старта esp_timer выдаёт
// значения около нуля, и нулевая метка неотличима от «отметок ещё не было».
static bool g_has_prev[FC_TIMING_COUNT];

static const char *const kNames[FC_TIMING_COUNT] = {
    [FC_TIMING_CONTROL] = "control (imu_ref_callback)",
    [FC_TIMING_MAIN] = "refloat_thd",
    [FC_TIMING_AUX] = "aux_thd",
};

uint64_t fc_uptime_us(void) {
    return (uint64_t) esp_timer_get_time();
}

void fc_timing_reset(void) {
    uint64_t now = fc_uptime_us();
    for (int i = 0; i < FC_TIMING_COUNT; ++i) {
        uint32_t nominal = g_stats[i].nominal_period_us;
        memset(&g_stats[i], 0, sizeof(g_stats[i]));
        g_stats[i].name = kNames[i];
        g_stats[i].nominal_period_us = nominal;
        g_stats[i].min_period_us = UINT32_MAX;
        g_stats[i].window_start_us = now;
        g_last_us[i] = 0;
        g_has_prev[i] = false;
    }
}

void fc_timing_set_nominal(FcTimingChannel ch, uint32_t period_us) {
    if (ch < FC_TIMING_COUNT) {
        g_stats[ch].nominal_period_us = period_us;
        g_stats[ch].name = kNames[ch];
    }
}

void fc_timing_tick(FcTimingChannel ch) {
    if (ch >= FC_TIMING_COUNT) {
        return;
    }
    uint64_t now = fc_uptime_us();
    FcTimingStats *s = &g_stats[ch];
    uint64_t prev = g_last_us[ch];
    bool had_prev = g_has_prev[ch];
    g_last_us[ch] = now;
    g_has_prev[ch] = true;

    if (!had_prev) {
        // Первая итерация: интервала ещё нет, окно начинается здесь.
        s->window_start_us = now;
        return;
    }

    uint32_t dt = (uint32_t) (now - prev);
    ++s->iterations;
    s->sum_period_us += dt;
    if (dt < s->min_period_us) {
        s->min_period_us = dt;
    }
    if (dt > s->max_period_us) {
        s->max_period_us = dt;
    }
    if (s->nominal_period_us &&
        dt > (uint64_t) s->nominal_period_us * FC_TIMING_LATE_NUM / FC_TIMING_LATE_DEN) {
        ++s->late;
    }
}

FcTimingStats fc_timing_get(FcTimingChannel ch) {
    FcTimingStats out = {0};
    if (ch < FC_TIMING_COUNT) {
        out = g_stats[ch];
        if (out.min_period_us == UINT32_MAX) {
            out.min_period_us = 0;
        }
        out.name = kNames[ch];
    }
    return out;
}
