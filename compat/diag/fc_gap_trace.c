#include "fc_gap_trace.h"

#include <string.h>

static struct {
    FcGapEvent slots[FC_GAP_TRACE_SLOTS];
    uint32_t stored;
    FcGapTraceStats st;
} T;

void fc_gap_trace_init(uint32_t threshold_us) {
    memset(&T, 0, sizeof T);
    T.st.threshold_us = threshold_us ? threshold_us : FC_GAP_TRACE_DEFAULT_THRESHOLD_US;
}

void fc_gap_trace_set_threshold(uint32_t threshold_us) {
    if (threshold_us) {
        T.st.threshold_us = threshold_us;
    }
}

uint32_t fc_gap_trace_threshold(void) {
    return T.st.threshold_us;
}

bool fc_gap_trace_is_event(uint32_t gap_us) {
    return gap_us >= T.st.threshold_us;
}

void fc_gap_trace_capture(const FcGapEvent *e) {
    if (e == NULL) {
        return;
    }
    ++T.st.events_total;
    if (e->gap_us >= FC_GAP_TRACE_UNACCEPTABLE_US) {
        ++T.st.events_over_20ms;
    }
    if (e->gap_us > T.st.max_gap_us) {
        T.st.max_gap_us = e->gap_us;
        T.st.max_gap_at_us = e->timestamp_us;
    }

    if (T.stored >= FC_GAP_TRACE_SLOTS) {
        // Кольцо полно. Сохраняем ПЕРВЫЕ, а не последние: первое событие
        // объясняет причину, десятое — только последствия.
        ++T.st.dropped;
        return;
    }

    T.slots[T.stored] = *e;
    T.slots[T.stored].seq = T.st.events_total;
    T.slots[T.stored].deferred_filled = false;
    ++T.stored;
    T.st.stored = T.stored;
}

uint32_t fc_gap_trace_count(void) {
    return T.stored;
}

const FcGapEvent *fc_gap_trace_get(uint32_t index) {
    return index < T.stored ? &T.slots[index] : NULL;
}

FcGapTraceStats fc_gap_trace_stats(void) {
    return T.st;
}

void fc_gap_trace_reset(void) {
    uint32_t th = T.st.threshold_us;
    memset(&T, 0, sizeof T);
    T.st.threshold_us = th;
}

int fc_gap_trace_next_unfilled(void) {
    for (uint32_t i = 0; i < T.stored; ++i) {
        if (!T.slots[i].deferred_filled) {
            return (int) i;
        }
    }
    return -1;
}

void fc_gap_trace_fill_deferred(uint32_t index, uint32_t heap_free, uint32_t heap_min,
                                uint32_t stack_imu, uint32_t stack_control, uint32_t stack_aux) {
    if (index >= T.stored) {
        return;
    }
    FcGapEvent *e = &T.slots[index];
    e->heap_free = heap_free;
    e->heap_min = heap_min;
    e->stack_imu = stack_imu;
    e->stack_control = stack_control;
    e->stack_aux = stack_aux;
    e->deferred_filled = true;
}
