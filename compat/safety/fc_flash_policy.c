#include "fc_flash_policy.h"

#include <string.h>

static struct {
    FcFlashPolicyStats st;
    // Эпизод отсрочки считается один раз, а не на каждом опросе задачи сброса:
    // иначе счётчик показывал бы частоту опроса, а не число отложенных записей.
    bool deferral_counted;
} P;

void fc_flash_policy_init(void) {
    memset(&P, 0, sizeof(P));
}

void fc_flash_policy_request(uint64_t now_us) {
    ++P.st.requested;
    if (!P.st.pending) {
        P.st.pending = true;
        P.st.pending_since_us = now_us;
        P.deferral_counted = false;
    }
}

FcFlashDecision fc_flash_policy_decide(bool write_allowed, uint64_t now_us) {
    (void) now_us;
    if (!P.st.pending) {
        return FC_FLASH_IDLE;
    }
    if (!write_allowed) {
        if (!P.deferral_counted) {
            ++P.st.deferred;
            P.deferral_counted = true;
        }
        return FC_FLASH_DEFER;
    }
    return FC_FLASH_EXECUTE;
}

void fc_flash_policy_report(bool ok, uint32_t duration_us, uint64_t now_us) {
    ++P.st.executed;
    P.st.last_duration_us = duration_us;
    if (duration_us > P.st.max_duration_us) {
        P.st.max_duration_us = duration_us;
    }
    P.st.last_executed_at_us = now_us;
    if (ok) {
        P.st.pending = false;
        P.deferral_counted = false;
    } else {
        // Неудачная запись не снимает ожидание: данные по-прежнему не сохранены,
        // и молча забыть об этом значило бы потерять запрос.
        ++P.st.failed;
    }
}

FcFlashPolicyStats fc_flash_policy_stats(void) {
    return P.st;
}
