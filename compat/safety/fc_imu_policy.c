#include "fc_imu_policy.h"

FcImuPolicyConfig fc_imu_policy_default_config(void) {
    FcImuPolicyConfig c;
    c.torque_freshness_us = 20000; // как FC_DUAL_COMMAND_FRESH_US
    c.latch_age_us = 40000;        // как FC_SUP_IMU_TIMEOUT_US
    c.hold_samples = c.torque_freshness_us / FC_IMU_POLICY_PERIOD_US;  // 10
    c.latch_samples = c.latch_age_us / FC_IMU_POLICY_PERIOD_US;        // 20
    return c;
}

FcImuPermit fc_imu_policy_evaluate(const FcImuPolicyConfig *cfg, const FcImuPolicyInputs *in,
                                   uint32_t *reason_mask) {
    uint32_t m = 0;
    FcImuPermit worst = FC_IMU_PERMIT_OK;

    // Неустранимые условия проверяются первыми: они не зависят ни от какого
    // возраста и не восстанавливаются сами.
    if (in->bus_stuck) {
        m |= FC_IMU_POLICY_BUS_STUCK;
        worst = FC_IMU_PERMIT_LOST;
    }
    if (in->reinit_failed) {
        m |= FC_IMU_POLICY_REINIT_FAILED;
        worst = FC_IMU_PERMIT_LOST;
    }

    if (!in->have_valid) {
        // Годного семпла не было вовсе. Это не отказ: так выглядит загрузка,
        // пока датчик не отдал первый семпл. Тяги при этом нет.
        m |= FC_IMU_POLICY_NO_SAMPLE;
        if (worst < FC_IMU_PERMIT_HOLD) {
            worst = FC_IMU_PERMIT_HOLD;
        }
        if (reason_mask) {
            *reason_mask = m;
        }
        return worst;
    }

    uint64_t age = in->now_us > in->last_valid_us ? in->now_us - in->last_valid_us : 0;
    // Отметка из будущего означает рассогласование часов, а не свежесть.
    if (in->last_valid_us > in->now_us) {
        age = cfg->latch_age_us + 1;
    }

    if (age > cfg->latch_age_us) {
        m |= FC_IMU_POLICY_STALE;
        worst = FC_IMU_PERMIT_LOST;
    } else if (age > cfg->torque_freshness_us) {
        m |= FC_IMU_POLICY_STALE;
        if (worst < FC_IMU_PERMIT_HOLD) {
            worst = FC_IMU_PERMIT_HOLD;
        }
    }

    // Серии. Каждая из трёх ведёт себя одинаково: до hold_samples — ничего,
    // между hold и latch — запрет без защёлки, дальше — защёлка.
    //
    // Почему серии проверяются ОТДЕЛЬНО от возраста, хотя выведены из него.
    // Возраст растёт и тогда, когда задача просто не исполнялась; серия
    // означает, что датчик отвечает, но отвечает негодным. Второе хуже
    // первого: неисполнявшаяся задача восстановится сама, а датчик, стабильно
    // отдающий мусор, — нет.
    struct {
        uint32_t count;
        uint32_t bit;
    } bursts[] = {
        {in->consecutive_read_failures, FC_IMU_POLICY_COMM_BURST},
        {in->consecutive_invalid, FC_IMU_POLICY_INVALID_BURST},
        {in->consecutive_accel_low, FC_IMU_POLICY_ACCEL_LOW_BURST},
    };
    for (unsigned i = 0; i < sizeof bursts / sizeof bursts[0]; ++i) {
        if (bursts[i].count >= cfg->latch_samples) {
            m |= bursts[i].bit;
            worst = FC_IMU_PERMIT_LOST;
        } else if (bursts[i].count >= cfg->hold_samples) {
            m |= bursts[i].bit;
            if (worst < FC_IMU_PERMIT_HOLD) {
                worst = FC_IMU_PERMIT_HOLD;
            }
        }
    }

    if (reason_mask) {
        *reason_mask = m;
    }
    return worst;
}

const char *fc_imu_permit_name(FcImuPermit p) {
    switch (p) {
    case FC_IMU_PERMIT_OK: return "OK";
    case FC_IMU_PERMIT_HOLD: return "HOLD (восстановимо)";
    case FC_IMU_PERMIT_LOST: return "LOST (защёлка)";
    default: return "?";
    }
}

const char *fc_imu_policy_reason_name(FcImuPolicyReason r) {
    switch (r) {
    case FC_IMU_POLICY_STALE: return "ориентация устарела";
    case FC_IMU_POLICY_NO_SAMPLE: return "годного семпла ещё не было";
    case FC_IMU_POLICY_COMM_BURST: return "серия неудачных обменов";
    case FC_IMU_POLICY_INVALID_BURST: return "серия непригодных семплов";
    case FC_IMU_POLICY_ACCEL_LOW_BURST: return "затяжной провал модуля ускорения";
    case FC_IMU_POLICY_BUS_STUCK: return "шина датчика не отпускается";
    case FC_IMU_POLICY_REINIT_FAILED: return "переинициализация не удалась";
    default: return "?";
    }
}
