// Журнал прогона замкнутого контура (ТЗ v0.9K §9).
//
// Одна запись на каждый запрос тока от Refloat, пока журнал включён. Пишется
// из потока Refloat — поэтому только копирование полей, без печати, без
// блокировок и без расчётов. Разбор и печать — после прогона, в консоли.
//
// Две части. Предварительная — кольцо на 16 записей: циклы между нажатием
// датчиков и стартом прогона, когда Refloat уже просит ток, а гейт обязан
// отвергать каждую просьбу. Основная — линейный буфер от старта до срока плюс
// хвост после срока: по хвосту видно, что после срока не ушло ничего.
#pragma once

#include "../../../compat/safety/fc_build_profile.h"

#if FC_CLOSED_LOOP_AVAILABLE

#include "../../../compat/refloat_glue/refloat_facade.h"
#include "../../../compat/safety/fc_motor_gate.h"

#include <stdbool.h>
#include <stdint.h>

// 500 мс при 500 Гц — 250 записей, плюс 60 мс хвоста — ещё 30.
#define FC_CL_LOG_MAX 300u
#define FC_CL_LOG_PRE 16u

enum {
    FC_CL_F_PAIR = 1u << 0,    // в этом цикле ушла целая пара
    FC_CL_F_PARTIAL = 1u << 1, // частичная передача
    FC_CL_F_DENIED = 1u << 2,  // координатор отказал
    FC_CL_F_TX_A_OK = 1u << 3,
    FC_CL_F_TX_B_OK = 1u << 4,
};

typedef struct {
    uint64_t t_us;          // время запроса
    uint64_t tx_us[2];      // время кадров 118 и 100, если ушли в этом цикле
    float pitch, balance_pitch, pitch_rate, setpoint;
    float raw_a;            // что попросил Refloat
    float delivered_a;      // что ушло после оболочки (0, если не ушло)
    float tx_a[2];          // величина в кадрах 118 и 100
    uint32_t seq;
    uint32_t deny_mask;     // маска отказа координатора
    uint16_t gate_us;       // стоимость вызова гейта целиком
    uint16_t skew_us;
    uint16_t dry_us;        // холостой gather+plan в контуре (только cl-loop-dry)
    uint8_t verdict;        // FcGateVerdict
    uint8_t sup_state;
    uint8_t imu_permit;
    uint8_t refloat_state;
    uint8_t flags;
} FcClRecord;

/** Включить предварительное кольцо: с этого момента каждый запрос пишется. */
void fc_cl_log_arm(void);
/** Начать основную часть: от start до deadline + tail. */
void fc_cl_log_start(uint64_t start_us, uint64_t deadline_us, uint64_t tail_us);
/** Выключить журнал целиком. */
void fc_cl_log_stop(void);
/** Холостой режим: в каждом цикле ещё и gather+plan без исполнения. */
void fc_cl_log_set_dry(bool on);
bool fc_cl_log_active(void);

/**
 * Путь запроса тока Refloat в сборке контура: отметка свежести команды, гейт
 * и запись журнала. Вызывается из VESC_IF->mc_set_current.
 */
FcGateVerdict fc_cl_request_current(const RefloatShadowFields *rf, float current);

typedef struct {
    uint64_t start_us, deadline_us, end_us;
    uint32_t pre_n;         // записей в предварительном кольце (≤ FC_CL_LOG_PRE)
    uint32_t pre_total;     // всего запросов до старта
    uint32_t main_n;
    uint32_t dropped;       // не поместились в основной буфер
} FcClLogInfo;

FcClLogInfo fc_cl_log_info(void);
/** Предварительная часть в хронологическом порядке. Возвращает число записей. */
uint32_t fc_cl_log_pre(FcClRecord *out, uint32_t max);
const FcClRecord *fc_cl_log_main(uint32_t *n);

#endif // FC_CLOSED_LOOP_AVAILABLE
