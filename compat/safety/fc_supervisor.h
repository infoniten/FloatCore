// Safety Supervisor — независимая от Refloat машина состояний, владеющая
// окончательным разрешением на выход к мотору (ТЗ v0.6A §8–§10).
//
// Refloat о супервизоре не знает и не может его обойти: единственный путь к
// мотору проходит через Motor Gate, а Gate спрашивает решение здесь.
//
// Модуль платформенно-нейтрален: все входы подаются снаружи вызовами
// fc_supervisor_report_*, время — параметром. Так он тестируется на host
// без платы и без таймеров.
#pragma once

#include "fc_imu_policy.h"

#include "fc_build_profile.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    FC_SUP_BOOT = 0,   // питание подано, платформа ещё не инициализирована
    FC_SUP_SELF_TEST,  // проверка подсистем
    FC_SUP_DISARMED,   // всё живо, тяга запрещена, запись конфигурации разрешена
    FC_SUP_READY,      // условия выполнены, ждём arm; запись конфигурации запрещена
    FC_SUP_ARMED,      // задел на будущее: в LAB_SAFE недостижимо
    FC_SUP_RUNNING,    // задел на будущее: в LAB_SAFE недостижимо
    FC_SUP_FAULT,      // залипающее состояние отказа
    FC_SUP_STATE_COUNT
} FcSupervisorState;

// Причины отказа — битовая маска: одновременно может быть несколько.
typedef enum {
    FC_FAULT_NONE = 0,
    FC_FAULT_SELF_TEST = 1u << 0,
    FC_FAULT_CONFIG_INVALID = 1u << 1,
    FC_FAULT_LOOP_STALL = 1u << 2,
    FC_FAULT_IMU_UNHEALTHY = 1u << 3,
    FC_FAULT_WATCHDOG = 1u << 4,
    FC_FAULT_INTERNAL = 1u << 5,
    FC_FAULT_STORAGE = 1u << 6,
    // Задел под этапы с CAN. Ни один из них на v0.6A не выставляется.
    FC_FAULT_ESC_A_LOST = 1u << 16,
    FC_FAULT_ESC_B_LOST = 1u << 17,
    FC_FAULT_CAN_STALE = 1u << 18,
    FC_FAULT_BATTERY = 1u << 19,
    FC_FAULT_THERMAL = 1u << 20,
} FcSupervisorFault;

typedef struct {
    // --- реализовано на v0.6A -------------------------------------------
    bool platform_initialized;
    bool config_valid;
    bool loop_alive;         // control loop отмечался в пределах таймаута
    bool imu_healthy;        // health-слой raw IMU не в ошибке
    bool watchdog_healthy;   // TWDT не срабатывал
    bool footpad_engaged;    // доска активирована (не отказ, но не даёт READY)
    // Нажатие датчиков ног — ИМИТАЦИЯ стенда, а не человек (v0.9K). Только при
    // этом признаке ARMED и RUNNING переживают нажатие: Refloat подаёт ток
    // лишь при нажатых датчиках. Настоящее нажатие снимает их, как и READY.
    bool footpad_simulated;

    // Ориентация датчика откалибрована и запись валидна (v0.6E).
    //
    // Условие READY, а не отказ. Обоснование: без калибровки платформа не
    // знает, как датчик стоит относительно доски, и любой угол, который она
    // отдаёт Refloat, — угол датчика, а не доски. Для балансирующей системы
    // это означает, что и порог входа в RUNNING, и отказ по крену считаются
    // не от того нуля. Отказом это не объявляется: некалиброванная плата не
    // сломана, она просто не готова.
    bool calibration_valid;

    // --- интерфейс под будущие этапы; на v0.6A всегда как ниже ----------
    bool esc_a_alive;        // false: ESC не подключён
    bool esc_b_alive;        // false
    bool can_fresh;          // false: шины нет
    bool battery_ok;         // true: нечего мерить
    bool thermal_ok;         // true
} FcSupervisorInputs;

// ------------------------------------- снимок отказа IMU (ТЗ v0.9B §4)
//
// В отказ IMU_UNHEALTHY ведут ДВА независимых пути, и до v0.9B нельзя было
// узнать, какой сработал. Это и породило мнимое противоречие приборов:
// трассировка не видела зазоров больше 12.7 мс, супервизор «отказывал по
// возрасту больше 40 мс», а на деле возраст не проверялся вовсе — отказ
// приходил от состояния модуля здоровья после одиночного сбоя чтения.
//
// Разбор — docs/imu_time_model.md.
typedef enum {
    FC_IMU_FAULT_CAUSE_NONE = 0,
    FC_IMU_FAULT_CAUSE_AGE,          // возраст принятого семпла превысил порог
    FC_IMU_FAULT_CAUSE_HEALTH_STATE, // модуль здоровья вернул состояние не OK
} FcImuFaultCause;

// Зеркало канонической шкалы (владелец — конвейер IMU). Здесь оно существует
// только чтобы снимок отказа содержал те же числа, что и трассировка, а не
// собственную версию событий.
typedef struct {
    uint64_t last_valid_us;
    uint64_t prev_valid_us;
    uint32_t last_gap_us;
    uint32_t sequence;
    uint32_t last_verdict;
    uint64_t last_poll_us;
} FcSupervisorImuTime;

typedef struct {
    FcImuFaultCause cause;
    uint32_t health_state;     // состояние модуля здоровья в момент отказа
    uint32_t policy_permit;    // вердикт политики (FcImuPermit)
    uint32_t policy_reasons;   // маска причин политики
    uint64_t now_us;
    uint32_t computed_age_us;  // now - last_valid, как его посчитал супервизор
    FcSupervisorImuTime time;
} FcImuFaultSnapshot;

typedef struct {
    FcSupervisorState state;
    uint32_t faults;              // маска FcSupervisorFault
    uint32_t faults_latched;      // всё, что когда-либо срабатывало
    uint64_t state_since_us;
    uint64_t last_loop_tick_us;
    uint64_t last_imu_sample_us;
    uint32_t transitions;
    uint32_t fault_entries;
    // Фактический возраст семпла IMU в момент срабатывания проверки
    // протухания, мкс. Заведено на v0.9A, когда супервизор защёлкнул отказ
    // по возрасту больше 40 мс, а трассировка зазоров не записала ни одного
    // интервала длиннее 12.7 мс. Два показания не могут быть верны
    // одновременно, и без этого числа нельзя сказать, чьё ошибочно.
    uint32_t imu_stale_age_us;
    uint32_t imu_worst_age_us;  // худший наблюдавшийся возраст, вне зависимости от отказа
    FcImuFaultSnapshot imu_fault;  // снимок ПЕРВОГО отказа IMU, не последнего
    FcSupervisorInputs inputs;
} FcSupervisorStatus;

/**
 * Таймаут «контур жив». Обоснование: номинальный период контура 2000 мкс
 * (MAIN_THREAD_FREQ = 500 в refloat-upstream/src/main.c:61). Порог выбран
 * как 25 периодов = 50 мс — это политика FloatCore, а не число из datasheet:
 * он должен быть заметно больше наблюдаемого джиттера (максимум ~2.8 мс,
 * docs/realtime_timing.md) и заметно меньше таймаута TWDT (5 с), чтобы
 * супервизор реагировал раньше watchdog-а.
 */
#define FC_SUP_LOOP_TIMEOUT_US 50000

/**
 * Таймаут свежести семпла IMU. 20 периодов при 500 Гц = 40 мс. Тоже политика
 * FloatCore: datasheet ICM-20948 задаёт частоту выдачи, но не то, сколько
 * пропусков допустимо.
 */
#define FC_SUP_IMU_TIMEOUT_US 40000

void fc_supervisor_init(uint64_t now_us);

/** BOOT -> SELF_TEST. */
void fc_supervisor_begin_self_test(uint64_t now_us);

/** Итог самопроверки: SELF_TEST -> DISARMED либо -> FAULT. */
void fc_supervisor_self_test_result(bool passed, uint64_t now_us);

/** Запрос перехода DISARMED -> READY. Возвращает false, если условия не выполнены. */
bool fc_supervisor_request_ready(uint64_t now_us);

// Замкнутый контур (ТЗ v0.9K). В сборке без FLOATCORE_REFLOAT_REAL_MOTOR_LOOP
// обе функции отказывают всегда.
#define FC_SUP_BURST_MAX_US 500000u  // v0.9K §18: не больше 500 мс непрерывного контура
bool fc_supervisor_request_closed_loop_ready(uint64_t now_us); // READY -> ARMED
bool fc_supervisor_begin_burst(uint64_t duration_us, uint64_t now_us); // ARMED -> RUNNING
uint64_t fc_supervisor_burst_deadline_us(void);

/** READY -> DISARMED. Всегда разрешён: снятие готовности безопасно. */
void fc_supervisor_disarm(uint64_t now_us);

/** Отметка живости контура управления. Вызывается из control loop. */
void fc_supervisor_report_loop_tick(uint64_t now_us);

/** Отметка свежего валидного семпла IMU. */
void fc_supervisor_report_imu_sample(uint64_t now_us);

void fc_supervisor_report_imu_healthy(bool healthy, uint64_t now_us);
void fc_supervisor_report_config_valid(bool valid, uint64_t now_us);
void fc_supervisor_report_platform_ready(bool ready, uint64_t now_us);
void fc_supervisor_report_watchdog(bool healthy, uint64_t now_us);
void fc_supervisor_report_footpad(bool engaged, uint64_t now_us);
void fc_supervisor_report_footpad_simulated(bool simulated, uint64_t now_us);
void fc_supervisor_report_calibration_valid(bool valid, uint64_t now_us);

/** Немедленный отказ с указанной причиной. */
void fc_supervisor_raise_fault(uint32_t fault_mask, uint64_t now_us);

/**
 * Периодическая проверка таймаутов. Вызывается не из realtime-пути.
 * Именно здесь протухание контура и IMU превращается в FAULT.
 */
void fc_supervisor_poll(uint64_t now_us);

/**
 * Снятие залипшего отказа. Возвращает false, если причина всё ещё активна:
 * автоматического восстановления нет намеренно (ТЗ v0.6 §10).
 */
bool fc_supervisor_clear_fault(uint64_t now_us);

/**
 * Сообщить состояние IMU вместе с канонической шкалой времени.
 *
 * Заменяет fc_supervisor_report_imu_healthy(): та форма не позволяла
 * записать, ПОЧЕМУ отказ произошёл, и оставляла снимок пустым.
 */
void fc_supervisor_report_imu(bool healthy, uint32_t health_state,
                              const FcSupervisorImuTime *t, uint64_t now_us);

/**
 * Сообщить вердикт политики. Трёхзначный, в отличие от прежнего
 * «здоров / не здоров»: HOLD снимает тягу, но НЕ защёлкивает отказ, и
 * положение восстанавливается само, когда ориентация снова становится
 * свежей. LOST защёлкивает.
 */
void fc_supervisor_report_imu_permit(FcImuPermit permit, uint32_t reasons, uint32_t health_state,
                                     const FcSupervisorImuTime *t, uint64_t now_us);

const char *fc_imu_fault_cause_name(FcImuFaultCause c);

/** Последний вердикт политики IMU. Дёшево: для пути контура 500 Гц. */
FcImuPermit fc_supervisor_last_imu_permit(void);

FcSupervisorStatus fc_supervisor_status(void);
FcSupervisorState fc_supervisor_state(void);

/** Разрешает ли текущее состояние физический выход на мотор. */
bool fc_supervisor_motor_output_permitted(void);

/** Разрешена ли сейчас запись в постоянное хранилище (ТЗ v0.6A §24). */
bool fc_supervisor_config_write_allowed(void);

const char *fc_supervisor_state_name(FcSupervisorState s);
/** Имя первой взведённой причины в маске, либо "none". */
const char *fc_supervisor_fault_name(uint32_t mask);
