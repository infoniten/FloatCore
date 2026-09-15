#include "fc_command_watchdog.h"

#include <string.h>

bool fc_command_watchdog_policy_valid(const FcCommandWatchdogPolicy *p, const char **why) {
    const char *dummy = NULL;
    if (!why) {
        why = &dummy;
    }
    *why = NULL;

    if (p->command_period_us == 0) {
        *why = "период команд равен нулю";
        return false;
    }

    // Ноль в timeout_msec на VESC означает «проверка отключена», а не
    // «сработает мгновенно»: условие в timeout.c звучит как
    // `timeout_msec != 0 && прошло больше timeout_msec`. Значение, которое
    // читается как самое строгое, на деле снимает защиту целиком.
    if (p->esc_timeout_us == 0) {
        *why = "timeout_msec = 0 отключает сторожевой таймер ESC целиком";
        return false;
    }

    // Поток Timeout просыпается раз в тик. Задавать таймаут меньше двух тиков
    // бессмысленно: фактическое срабатывание всё равно округлится вверх, а
    // ложные срабатывания станут вероятными.
    uint32_t tick = p->esc_tick_us ? p->esc_tick_us : FC_ESC_TIMEOUT_TICK_US;
    if (p->esc_timeout_us < 2 * tick) {
        *why = "таймаут ESC меньше двух тиков его потока Timeout";
        return false;
    }

    // Главное правило. FloatCore обязан снять тягу САМ раньше, чем это
    // сделает ESC. Если порядок обратный, единственным работающим барьером
    // остаётся чужой таймер, а собственные проверки не успевают ни на что
    // повлиять — и вся многослойность оказывается декорацией.
    if (p->supervisor_timeout_us >= p->esc_timeout_us) {
        *why = "FloatCore снимает тягу не раньше ESC — свои барьеры бесполезны";
        return false;
    }

    // Допуск на пропуски должен укладываться в собственный таймаут, иначе
    // «разрешено потерять N команд» — пустые слова: супервизор вмешается
    // раньше, чем допуск будет исчерпан.
    uint64_t tolerated = (uint64_t) p->command_period_us * (p->tolerated_misses + 1);
    if (tolerated > p->supervisor_timeout_us) {
        *why = "допуск на пропуски больше собственного таймаута FloatCore";
        return false;
    }

    // Верхняя граница. Сколько времени доска едет с последней командой, если
    // FloatCore умер целиком (например, перезагрузка ESP32): в этот момент
    // никакой собственный барьер не работает, и держит только ESC.
    if (p->esc_timeout_us > 200000u) {
        *why = "ESC держит последнюю команду дольше 200 мс после отказа FloatCore";
        return false;
    }

    return true;
}

uint32_t fc_command_watchdog_worst_case_us(const FcCommandWatchdogPolicy *p) {
    // Худший случай — полный отказ FloatCore: собственные барьеры не
    // работают, и тягу снимает только ESC. Его поток просыпается раз в тик,
    // поэтому к таймауту добавляется целый тик округления.
    uint32_t tick = p->esc_tick_us ? p->esc_tick_us : FC_ESC_TIMEOUT_TICK_US;
    return p->esc_timeout_us + tick;
}

// ------------------------------------------------------------------ поток

void fc_command_stream_init(FcCommandStream *s, const FcCommandWatchdogPolicy *p) {
    memset(s, 0, sizeof(*s));
    s->policy = *p;
    s->state = FC_CMD_STREAM_IDLE;
}

void fc_command_stream_sent(FcCommandStream *s, uint64_t now_us) {
    if (s->armed) {
        uint32_t gap = (uint32_t) (now_us - s->last_command_us);
        if (gap > s->max_gap_us) {
            s->max_gap_us = gap;
        }
    }
    s->last_command_us = now_us;
    s->armed = true;
}

FcCommandStreamState fc_command_stream_poll(FcCommandStream *s, uint64_t now_us) {
    if (!s->armed) {
        s->state = FC_CMD_STREAM_IDLE;
        return s->state;
    }

    uint64_t age = now_us - s->last_command_us;
    uint64_t tolerated =
        (uint64_t) s->policy.command_period_us * (s->policy.tolerated_misses + 1);

    FcCommandStreamState next;
    if (age >= s->policy.supervisor_timeout_us) {
        next = FC_CMD_STREAM_LOST;
    } else if (age > tolerated) {
        next = FC_CMD_STREAM_LATE;
    } else {
        next = FC_CMD_STREAM_OK;
    }

    if (next != s->state) {
        if (next == FC_CMD_STREAM_LATE) {
            ++s->transitions_to_late;
        } else if (next == FC_CMD_STREAM_LOST) {
            ++s->transitions_to_lost;
        }
        s->state = next;
    }
    return s->state;
}

const char *fc_command_stream_state_name(FcCommandStreamState st) {
    switch (st) {
    case FC_CMD_STREAM_IDLE:
        return "IDLE";
    case FC_CMD_STREAM_OK:
        return "OK";
    case FC_CMD_STREAM_LATE:
        return "LATE";
    case FC_CMD_STREAM_LOST:
        return "LOST";
    default:
        return "?";
    }
}
