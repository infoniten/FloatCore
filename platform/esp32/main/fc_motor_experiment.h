// Экспериментальный источник моторных команд (ТЗ v0.9A §11, §13, §16-§20).
//
// ЧТО ЭТО ТАКОЕ. Единственный источник, которому на v0.9A разрешено дойти до
// реального транспорта. Выход Refloat к транспорту НЕ подключён: контроллер
// балансировки с живым IMU, получив право слать, немедленно начал бы
// балансировать, а замкнутая балансировка на этом этапе запрещена. Поэтому у
// Motor Gate появилось понятие источника, и допущен ровно один — этот.
//
// ЧТО ЭТО НЕ ОТМЕНЯЕТ. Ни одной проверки. Команда отсюда проходит тот же путь:
//
//   producer -> Motor Gate -> Dual Motor Coordinator -> транспорт -> TWAI
//
// Вооружение — отдельное осознанное действие человека, с девятью
// предусловиями. После загрузки и после любого отказа система обезоружена, и
// восстановление входов её не вооружает.
#pragma once

#include "../../../compat/safety/fc_build_profile.h"

#include <stdbool.h>
#include <stdint.h>

#if FC_MOTOR_BACKEND_AVAILABLE

// Программный предел первой серии. НАМЕРЕННО много ниже предела ESC (5 А):
// доказывать поведение транспорта и отказных путей надо на минимальном токе,
// а не на максимальном (ТЗ v0.9A §7, §13).
#define FC_MOTOR_EXP_MAX_CURRENT_A 1.0f

// Темп источника. Совпадает с темпом контура: политика сторожевого таймера
// v0.7C построена вокруг 500 Гц, и менять его для теста значило бы проверять
// не ту систему.
#define FC_MOTOR_EXP_PERIOD_US 2000u

// Впрыск отказов. Только для проверки отказных путей на вывешенном колесе;
// каждый из них ЗАПРЕЩАЕТ тягу, ни один не разрешает.
typedef enum {
    FC_MOTOR_INJECT_IMU_STALE = 1u << 0,
    FC_MOTOR_INJECT_NODE_A_STALE = 1u << 1,
    FC_MOTOR_INJECT_NODE_B_STALE = 1u << 2,
    FC_MOTOR_INJECT_NODE_A_FAULT = 1u << 3,
    FC_MOTOR_INJECT_NODE_B_FAULT = 1u << 4,
    FC_MOTOR_INJECT_BUS_OFF = 1u << 5,
    FC_MOTOR_INJECT_CAN_DEGRADED = 1u << 6,
    FC_MOTOR_INJECT_FAIL_TX_A = 1u << 7, // транспорт «не смог» передать A
    FC_MOTOR_INJECT_FAIL_TX_B = 1u << 8,
    FC_MOTOR_INJECT_PRODUCER_STALL = 1u << 9, // источник замолкает, не снимая вооружения
} FcMotorInject;

typedef struct {
    bool running;
    float requested_a;
    uint32_t remaining_ms;

    uint64_t cycles;          // итераций источника
    uint64_t gate_allowed;
    uint64_t gate_rejected;
    uint64_t pairs_sent;      // пар, ушедших целиком
    uint64_t pairs_partial;
    uint64_t pairs_denied;

    uint32_t skew_last_us;
    uint32_t skew_max_us;
    uint32_t skew_p50_us;     // оценка по гистограмме
    uint32_t skew_p99_us;

    uint64_t last_command_us; // когда источник в последний раз просил тягу
    uint64_t last_tx_us;      // когда кадр в последний раз действительно ушёл
    uint32_t inject_mask;
    uint32_t last_deny_mask;
    uint32_t last_gate_verdict;
} FcMotorExpStats;

void fc_motor_experiment_init(void);

/** Вооружить. deny_mask заполняется всегда, даже при успехе (нулём). */
bool fc_motor_experiment_arm(uint32_t *deny_mask);
void fc_motor_experiment_disarm(void);
void fc_motor_experiment_clear_latch(void);
bool fc_motor_experiment_armed(void);

/**
 * Запустить источник на заданный ток и время. Ток зажимается НЕ будет: если
 * он выше программного предела, запуск отвергается.
 */
bool fc_motor_experiment_run(float amps, uint32_t ms);
void fc_motor_experiment_stop(void);

void fc_motor_experiment_inject(uint32_t mask);
uint32_t fc_motor_experiment_injected(void);

FcMotorExpStats fc_motor_experiment_stats(void);
void fc_motor_experiment_reset_stats(void);

#endif // FC_MOTOR_BACKEND_AVAILABLE
