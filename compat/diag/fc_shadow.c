#include "fc_shadow.h"

#include <math.h>
#include <string.h>

static struct {
    FcShadowSample ring[FC_SHADOW_RING];
    uint32_t head;
    uint32_t count;
    FcShadowStats st;
    float prev_current;
    bool have_prev;
    bool in_deadzone;
    uint32_t decim;
} S;

void fc_shadow_init(void) {
    memset(&S, 0, sizeof S);
}

void fc_shadow_reset(void) {
    fc_shadow_init();
}

void fc_shadow_record(const FcShadowSample *s) {
    if (s == NULL) {
        return;
    }
    ++S.st.samples;

    float a = fabsf(s->current);
    if (a < FC_SHADOW_DEADZONE_LOW_A) {
        ++S.st.below_low;
    } else if (a <= FC_SHADOW_DEADZONE_HIGH_A) {
        ++S.st.in_band;
    } else {
        ++S.st.above_high;
    }

    if (s->current > S.st.peak_positive) {
        S.st.peak_positive = s->current;
    }
    if (s->current < S.st.peak_negative) {
        S.st.peak_negative = s->current;
    }
    if (fabsf(s->pid_i) > S.st.peak_abs_i_term) {
        S.st.peak_abs_i_term = fabsf(s->pid_i);
    }
    S.st.last_i_term = s->pid_i;

    if (S.have_prev && ((S.prev_current < 0.0f && s->current > 0.0f) ||
                        (S.prev_current > 0.0f && s->current < 0.0f))) {
        ++S.st.zero_crossings;
    }
    S.prev_current = s->current;
    S.have_prev = true;

    if (s->sat != 0) {
        ++S.st.saturation_samples;
    }
    if (s->deliverable) {
        ++S.st.deliverable_samples;
    }

    // Интегратор внутри мёртвой зоны. Вход в зону запоминает значение,
    // внутри считается прирост. Смысл: если команда ничего не делает, а
    // интеграл при этом растёт, то при выходе из зоны он сработает скачком —
    // и это надо ИЗМЕРИТЬ, а не компенсировать.
    bool dz = a < FC_SHADOW_DEADZONE_LOW_A;
    if (dz && !S.in_deadzone) {
        S.in_deadzone = true;
        S.st.i_term_at_deadzone_entry = s->pid_i;
        ++S.st.deadzone_entries;
    } else if (!dz && S.in_deadzone) {
        S.in_deadzone = false;
    } else if (dz) {
        float growth = fabsf(s->pid_i - S.st.i_term_at_deadzone_entry);
        if (growth > S.st.max_i_growth_in_deadzone) {
            S.st.max_i_growth_in_deadzone = growth;
        }
    }

    if (!S.st.have_pitch) {
        S.st.have_pitch = true;
        S.st.min_pitch = S.st.max_pitch = s->pitch;
        S.st.current_at_min_pitch = S.st.current_at_max_pitch = s->current;
    } else {
        if (s->pitch < S.st.min_pitch) {
            S.st.min_pitch = s->pitch;
            S.st.current_at_min_pitch = s->current;
        }
        if (s->pitch > S.st.max_pitch) {
            S.st.max_pitch = s->pitch;
            S.st.current_at_max_pitch = s->current;
        }
    }

    // В историю — каждая десятая. Статистика выше уже учла эту итерацию.
    if (++S.decim < FC_SHADOW_DECIMATION) {
        return;
    }
    S.decim = 0;

    S.ring[S.head] = *s;
    S.head = (S.head + 1u) % FC_SHADOW_RING;
    if (S.count < FC_SHADOW_RING) {
        ++S.count;
    } else {
        ++S.st.dropped;
    }
}

FcShadowStats fc_shadow_stats(void) {
    return S.st;
}

uint32_t fc_shadow_tail(FcShadowSample *out, uint32_t max) {
    if (out == NULL || max == 0) {
        return 0;
    }
    uint32_t n = S.count < max ? S.count : max;
    for (uint32_t i = 0; i < n; ++i) {
        // Сначала самые свежие: при разборе интересен конец, а не начало.
        uint32_t idx = (S.head + FC_SHADOW_RING - 1u - i) % FC_SHADOW_RING;
        out[i] = S.ring[idx];
    }
    return n;
}
