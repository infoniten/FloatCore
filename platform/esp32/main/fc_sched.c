#include "fc_sched.h"

#include <string.h>

static FcSchedStats S[FC_SCHED_MAX_THREADS];

void fc_sched_note_sleep(size_t idx, uint32_t requested_us, uint32_t actual_us) {
    if (idx >= FC_SCHED_MAX_THREADS) {
        return;
    }
    FcSchedStats *s = &S[idx];
    if (s->iterations == 0) {
        s->latency_min_us = 0xFFFFFFFFu;
    }
    ++s->iterations;
    s->requested_sum_us += requested_us;
    s->actual_sum_us += actual_us;

    // Задержка не может быть отрицательной: таймер не будит раньше срока.
    // Но если такое случится, считать это «нулевой задержкой» честнее, чем
    // получить огромное число из беззнакового вычитания.
    uint32_t lat = actual_us > requested_us ? actual_us - requested_us : 0;
    s->latency_sum_us += lat;
    if (lat < s->latency_min_us) {
        s->latency_min_us = lat;
    }
    if (lat > s->latency_max_us) {
        s->latency_max_us = lat;
    }
    uint32_t bin = lat / 100;
    if (bin > 9) {
        bin = 9;
    }
    ++s->latency_hist[bin];
}

void fc_sched_reset(void) {
    memset(S, 0, sizeof(S));
}

FcSchedStats fc_sched_stats(size_t idx) {
    FcSchedStats empty;
    memset(&empty, 0, sizeof(empty));
    if (idx >= FC_SCHED_MAX_THREADS) {
        return empty;
    }
    return S[idx];
}
