#include "fc_can_health.h"

#include <string.h>

static FcCanNode *find(FcCanHealth *h, uint8_t id) {
    for (uint32_t i = 0; i < h->count; ++i) {
        if (h->nodes[i].id == id) {
            return &h->nodes[i];
        }
    }
    return NULL;
}

static FcCanNode *find_or_add(FcCanHealth *h, uint8_t id) {
    FcCanNode *n = find(h, id);
    if (n) {
        return n;
    }
    if (h->count >= FC_CAN_MAX_NODES) {
        ++h->table_overflow;
        return NULL;
    }
    n = &h->nodes[h->count++];
    memset(n, 0, sizeof(*n));
    n->id = id;
    n->known = true;
    return n;
}

void fc_can_health_init(FcCanHealth *h, uint32_t stale_us) {
    memset(h, 0, sizeof(*h));
    h->stale_us = stale_us ? stale_us : FC_CAN_NODE_STALE_US;
}

void fc_can_health_expect(FcCanHealth *h, uint8_t id) {
    FcCanNode *n = find_or_add(h, id);
    if (n) {
        n->expected = true;
    }
}

static void seen(FcCanHealth *h, FcCanNode *n, uint64_t now_us) {
    (void) h;
    n->last_seen_us = now_us;
    n->ever_seen = true;
    n->healthy = true;
}

void fc_can_health_on_status(FcCanHealth *h, uint8_t id, uint64_t now_us) {
    FcCanNode *n = find_or_add(h, id);
    if (!n) {
        return;
    }
    ++n->statuses;
    n->last_status_us = now_us;
    seen(h, n, now_us);
}

void fc_can_health_on_diag_response(FcCanHealth *h, uint8_t id, uint64_t now_us) {
    FcCanNode *n = find_or_add(h, id);
    if (!n) {
        return;
    }
    ++n->diag_responses;
    n->last_diag_us = now_us;
    seen(h, n, now_us);
}

void fc_can_health_on_diag_timeout(FcCanHealth *h, uint8_t id) {
    FcCanNode *n = find_or_add(h, id);
    if (!n) {
        return;
    }
    // Таймаут запроса сам по себе узел не хоронит: периодический STATUS —
    // независимый и более частый признак жизни. Молчание в ответ на запрос
    // при живом статусе означает проблему с запросом, а не с узлом, и это
    // разные диагнозы.
    ++n->diag_timeouts;
}

void fc_can_health_tick(FcCanHealth *h, uint64_t now_us) {
    for (uint32_t i = 0; i < h->count; ++i) {
        FcCanNode *n = &h->nodes[i];
        bool ok = n->ever_seen && (now_us - n->last_seen_us) < h->stale_us;
        if (n->healthy && !ok) {
            ++n->health_drops;
        }
        n->healthy = ok;
    }
}

const FcCanNode *fc_can_health_node(const FcCanHealth *h, uint8_t id) {
    for (uint32_t i = 0; i < h->count; ++i) {
        if (h->nodes[i].id == id) {
            return &h->nodes[i];
        }
    }
    return NULL;
}

bool fc_can_health_all_expected_healthy(const FcCanHealth *h) {
    bool any = false;
    for (uint32_t i = 0; i < h->count; ++i) {
        if (!h->nodes[i].expected) {
            continue;
        }
        any = true;
        if (!h->nodes[i].healthy) {
            return false;
        }
    }
    // Ни одного ожидаемого узла не объявлено — отвечать «всё здорово» нельзя:
    // это означало бы «проверять нечего», а не «проверено».
    return any;
}
