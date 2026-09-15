#include "fc_log.h"

#include <stdio.h>
#include <string.h>

void fc_log_init(FcLog *l) {
    memset(l, 0, sizeof(*l));
}

bool fc_log_push(FcLog *l, FcLogLevel level, const char *tag, uint64_t t_us, const char *fmt,
                 va_list ap) {
    if (l->count >= FC_LOG_SLOTS) {
        // Переполнение. Отбрасываем НОВУЮ запись, а не самую старую: при
        // всплеске первые строки обычно и объясняют причину, а последние
        // повторяют следствие. Терять объяснение хуже, чем повтор.
        ++l->dropped;
        return false;
    }

    FcLogEntry *e = &l->slot[l->head];
    e->t_us = t_us;
    e->level = (uint8_t) level;
    e->seq = ++l->seq;

    if (tag) {
        snprintf(e->tag, sizeof(e->tag), "%s", tag);
    } else {
        e->tag[0] = '\0';
    }

    int n = vsnprintf(e->msg, sizeof(e->msg), fmt, ap);
    if (n < 0) {
        e->msg[0] = '\0';
    } else if ((size_t) n >= sizeof(e->msg)) {
        ++l->truncated;
    }

    l->head = (l->head + 1) % FC_LOG_SLOTS;
    ++l->count;
    ++l->emitted;
    if (l->count > l->high_water) {
        l->high_water = l->count;
    }
    return true;
}

bool fc_log_pop(FcLog *l, FcLogEntry *out) {
    if (l->count == 0) {
        return false;
    }
    *out = l->slot[l->tail];
    l->tail = (l->tail + 1) % FC_LOG_SLOTS;
    --l->count;
    return true;
}

uint32_t fc_log_pending(const FcLog *l) {
    return l->count;
}

const char *fc_log_level_name(FcLogLevel level) {
    switch (level) {
    case FC_LOG_ERROR:
        return "E";
    case FC_LOG_WARN:
        return "W";
    case FC_LOG_INFO:
        return "I";
    default:
        return "?";
    }
}
