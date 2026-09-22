// Статистика периодичности и времени исполнения задач FloatCore
// (ТЗ v0.6A §18, §19).
//
// Мерять надо две разные величины, и их постоянно путают:
//   * период      — интервал между началами соседних итераций. Это джиттер
//                   планировщика: когда задачу разбудили.
//   * исполнение  — сколько итерация работала. Это нагрузка.
// Средний период всегда равен номиналу, если нет дрейфа, поэтому по нему
// одному судить нельзя — нужны перцентили и максимум.
//
// Всё считается в ОЗУ, в serial не уходит ни одна итерация: печать со
// скоростью контура сама испортила бы тайминг (при 115200 бод одна строка
// занимает около 9 мс).

#include "fc_platform.h"

#include "esp_timer.h"

#include <stdbool.h>
#include <string.h>

// Итерация считается опоздавшей, если период превысил номинал более чем на
// 20 %, и пропущенной, если превысил номинал вдвое.
#define FC_TIMING_LATE_NUM 12
#define FC_TIMING_LATE_DEN 10

typedef struct {
    FcTimingStats s;
    uint64_t last_us;
    bool has_prev;
    uint64_t exec_begin_us;
    uint64_t preempt_begin_us;
    bool exec_open;
    uint32_t bin_width_us;
    uint32_t bins[FC_TIMING_BINS + 1];       // последняя — переполнение
    uint32_t exec_bins[FC_TIMING_BINS + 1];  // шкала та же, но от 0 до 4×nominal
} FcChannel;

static FcChannel g_ch[FC_TIMING_COUNT];

// Снимок фазы загрузки (ТЗ v0.7C §7).
//
// Загрузка печатает в UART десятки строк, и контур в это время неизбежно
// опаздывает. Смешивать это с установившимся режимом нельзя: единичный
// пропуск на старте иначе навсегда останется в цифрах и будет маскировать
// настоящие пропуски, которые появятся потом. Поэтому статистика делится на
// две фазы явным событием, а не отбрасывается.
static FcTimingStats g_boot[FC_TIMING_COUNT];
static bool g_steady;

static const char *const kNames[FC_TIMING_COUNT] = {
    [FC_TIMING_CONTROL] = "control (imu_ref_callback)",
    [FC_TIMING_MAIN] = "refloat_thd",
    [FC_TIMING_AUX] = "aux_thd",
    [FC_TIMING_IMU_READ] = "icm20948 read",
    [FC_TIMING_SUPERVISOR] = "fc_super",
};

uint64_t fc_uptime_us(void) {
    return (uint64_t) esp_timer_get_time();
}

static void recompute_bin_width(FcChannel *c) {
    // Диапазон гистограммы — 4 номинала, разбитые на FC_TIMING_BINS корзин.
    uint32_t w = c->s.nominal_period_us * 4 / FC_TIMING_BINS;
    c->bin_width_us = w ? w : 1;
}

static void channel_reset(FcChannel *c, FcTimingChannel idx, uint64_t now) {
    uint32_t nominal = c->s.nominal_period_us;
    memset(c, 0, sizeof(*c));
    c->s.name = kNames[idx];
    c->s.nominal_period_us = nominal;
    c->s.min_period_us = UINT32_MAX;
    c->s.exec_min_us = UINT32_MAX;
    c->s.window_start_us = now;
    recompute_bin_width(c);
}

void fc_timing_reset(void) {
    uint64_t now = fc_uptime_us();
    for (int i = 0; i < FC_TIMING_COUNT; ++i) {
        channel_reset(&g_ch[i], (FcTimingChannel) i, now);
    }
}

void fc_timing_mark_steady(void) {
    if (g_steady) {
        return;
    }
    uint64_t now = fc_uptime_us();
    for (int i = 0; i < FC_TIMING_COUNT; ++i) {
        g_boot[i] = fc_timing_get((FcTimingChannel) i);
        channel_reset(&g_ch[i], (FcTimingChannel) i, now);
    }
    g_steady = true;
}

bool fc_timing_is_steady(void) {
    return g_steady;
}

FcTimingStats fc_timing_get_boot(FcTimingChannel ch) {
    if ((unsigned) ch >= FC_TIMING_COUNT) {
        FcTimingStats empty = {0};
        return empty;
    }
    return g_boot[ch];
}

void fc_timing_set_nominal(FcTimingChannel ch, uint32_t period_us) {
    if (ch >= FC_TIMING_COUNT) {
        return;
    }
    g_ch[ch].s.nominal_period_us = period_us;
    g_ch[ch].s.name = kNames[ch];
    recompute_bin_width(&g_ch[ch]);
}

static void bin_add(uint32_t *bins, uint32_t width, uint32_t value, uint32_t *overflow) {
    uint32_t idx = width ? value / width : 0;
    if (idx >= FC_TIMING_BINS) {
        ++bins[FC_TIMING_BINS];
        if (overflow) {
            ++*overflow;
        }
    } else {
        ++bins[idx];
    }
}

void fc_timing_tick(FcTimingChannel ch) {
    if (ch >= FC_TIMING_COUNT) {
        return;
    }
    uint64_t now = fc_uptime_us();
    FcChannel *c = &g_ch[ch];
    uint64_t prev = c->last_us;
    bool had_prev = c->has_prev;
    c->last_us = now;
    c->has_prev = true;

    if (!had_prev) {
        c->s.window_start_us = now;
        return;
    }

    uint32_t dt = (uint32_t) (now - prev);
    ++c->s.iterations;
    c->s.sum_period_us += dt;
    if (dt < c->s.min_period_us) {
        c->s.min_period_us = dt;
    }
    if (dt > c->s.max_period_us) {
        c->s.max_period_us = dt;
    }
    if (c->s.nominal_period_us) {
        if (dt > (uint64_t) c->s.nominal_period_us * FC_TIMING_LATE_NUM / FC_TIMING_LATE_DEN) {
            ++c->s.late;
        }
        // Ровно двойной период — это уже пропущенная итерация: одно
        // пробуждение не состоялось. Поэтому сравнение нестрогое.
        if (dt >= (uint64_t) c->s.nominal_period_us * 2) {
            ++c->s.missed;
        }
    }
    bin_add(c->bins, c->bin_width_us, dt, &c->s.overflow);
}

// Суммарная измеренная занятость ядра реального времени. Инкрементируется в
// конце КАЖДОЙ итерации любого канала, поэтому разность двух отсчётов есть
// чужая работа, прошедшая между ними.
static uint64_t g_core_busy_us;

void fc_timing_exec_begin(FcTimingChannel ch) {
    if (ch >= FC_TIMING_COUNT) {
        return;
    }
    g_ch[ch].exec_begin_us = fc_uptime_us();
    g_ch[ch].preempt_begin_us = g_core_busy_us;
    g_ch[ch].exec_open = true;
}

void fc_timing_exec_end(FcTimingChannel ch) {
    if (ch >= FC_TIMING_COUNT || !g_ch[ch].exec_open) {
        return;
    }
    FcChannel *c = &g_ch[ch];
    c->exec_open = false;
    uint32_t dur = (uint32_t) (fc_uptime_us() - c->exec_begin_us);

    // Разность берётся ДО собственного вклада, иначе итерация учла бы саму
    // себя. Вклад добавляется ниже, уже после расчёта.
    uint64_t preempt64 = g_core_busy_us - c->preempt_begin_us;
    uint32_t preempt = preempt64 > dur ? dur : (uint32_t) preempt64;
    uint32_t net = dur - preempt;
    c->s.preempt_sum_us += preempt;
    if (preempt > c->s.preempt_max_us) {
        c->s.preempt_max_us = preempt;
    }
    c->s.net_sum_us += net;
    if (net > c->s.net_max_us) {
        c->s.net_max_us = net;
    }
    g_core_busy_us += dur;

    ++c->s.exec_samples;
    c->s.exec_sum_us += dur;
    if (dur < c->s.exec_min_us) {
        c->s.exec_min_us = dur;
    }
    if (dur > c->s.exec_max_us) {
        c->s.exec_max_us = dur;
    }
    bin_add(c->exec_bins, c->bin_width_us, dur, NULL);
}

// Перцентиль по гистограмме: возвращается верхняя граница корзины, в которой
// накопленная доля впервые достигает заданной. Это оценка сверху с точностью
// до ширины корзины, и так это и надо читать.
static uint32_t percentile(const uint32_t *bins, uint32_t width, uint64_t total,
                           uint32_t permille) {
    if (total == 0) {
        return 0;
    }
    uint64_t target = (total * permille + 999) / 1000;
    uint64_t acc = 0;
    for (uint32_t i = 0; i <= FC_TIMING_BINS; ++i) {
        acc += bins[i];
        if (acc >= target) {
            return (i >= FC_TIMING_BINS) ? UINT32_MAX : (i + 1) * width;
        }
    }
    return (FC_TIMING_BINS + 1) * width;
}

FcTimingStats fc_timing_get(FcTimingChannel ch) {
    FcTimingStats out;
    memset(&out, 0, sizeof(out));
    if (ch >= FC_TIMING_COUNT) {
        return out;
    }
    FcChannel *c = &g_ch[ch];
    out = c->s;
    out.name = kNames[ch];
    if (out.min_period_us == UINT32_MAX) {
        out.min_period_us = 0;
    }
    if (out.exec_min_us == UINT32_MAX) {
        out.exec_min_us = 0;
    }
    out.p50_us = percentile(c->bins, c->bin_width_us, out.iterations, 500);
    out.p95_us = percentile(c->bins, c->bin_width_us, out.iterations, 950);
    out.p99_us = percentile(c->bins, c->bin_width_us, out.iterations, 990);
    out.p999_us = percentile(c->bins, c->bin_width_us, out.iterations, 999);
    out.exec_p99_us = percentile(c->exec_bins, c->bin_width_us, out.exec_samples, 990);
    return out;
}

uint32_t fc_timing_histogram(FcTimingChannel ch, uint32_t *bins, uint32_t max_bins,
                             uint32_t *bin_width_us) {
    if (ch >= FC_TIMING_COUNT || !bins) {
        return 0;
    }
    FcChannel *c = &g_ch[ch];
    uint32_t n = FC_TIMING_BINS + 1;
    if (n > max_bins) {
        n = max_bins;
    }
    memcpy(bins, c->bins, n * sizeof(uint32_t));
    if (bin_width_us) {
        *bin_width_us = c->bin_width_us;
    }
    return n;
}
