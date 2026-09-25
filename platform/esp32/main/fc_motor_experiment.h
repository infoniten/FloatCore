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

// Предел одношаговой команды (ТЗ v0.9G §12). Строго ниже предела стенда и
// ниже любой будущей оболочки замкнутого контура: цель одношага — доказать
// сквозной путь и ЗНАК, а не управлять. Управление доказывается другим
// экспериментом, и смешивать эти две цели в одной команде нельзя.
#define FC_MOTOR_ONESHOT_MAX_A 0.5f

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
    // Стоимость реального пути передачи (ТЗ v0.9G §7). Измеряется вокруг
    // ОДНОГО вызова: Motor Gate -> координатор -> сериализация 118 ->
    // постановка в очередь TX -> сериализация 100 -> постановка TX. То есть
    // ровно то, что добавится к бюджету ядра при замыкании контура.
    //
    // Время настенное. Источник живёт на ядре 0 и на приоритете 6, где
    // вытеснять его почти некому, поэтому оно близко к процессорному; но
    // равенством это не является и так читаться не должно.
    uint64_t path_samples;
    uint64_t path_sum_us;
    uint32_t path_min_us;
    uint32_t path_max_us;
    uint32_t path_p50_us;
    uint32_t path_p99_us;
    uint32_t path_p999_us;

    // След передачи по половинам (ТЗ v0.9J §10): время, величина и результат
    // последнего кадра, ушедшего в шину, и сколько кадров ушло всего. Индекс
    // 0 — половина 118, 1 — половина 100; порядок передачи тот же.
    uint64_t tx_us[2];
    float tx_amps[2];
    bool tx_ok[2];
    uint64_t tx_count[2];

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

/**
 * Ровно ОДНА пара кадров, после неё — молчание (ТЗ v0.9G §11).
 *
 * Не повторяется и не обновляется. Именно поэтому она заодно проверяет
 * watchdog: ESC обязан снять момент по своему таймауту (50 мс), а не потому,
 * что кто-то прислал ноль. Если момент не снимется — значит таймаут не
 * работает, и это важнее самого знака.
 */
bool fc_motor_experiment_oneshot(float amps, uint32_t *gate_verdict);

void fc_motor_experiment_inject(uint32_t mask);
uint32_t fc_motor_experiment_injected(void);

FcMotorExpStats fc_motor_experiment_stats(void);
void fc_motor_experiment_reset_stats(void);

#endif // FC_MOTOR_BACKEND_AVAILABLE
