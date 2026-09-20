// Координатор двух половин ESC (ТЗ v0.8C §2C, §3, §4, §5, §6, §7).
//
// ЧТО ЭТОТ МОДУЛЬ ДЕЛАЕТ. Он превращает ОДНО логическое требование тяги от
// доски в ПАРУ команд двум физическим VESC — и, что важнее, решает, имеет ли
// право эта пара вообще появиться.
//
// ЧЕГО ОН НЕ ДЕЛАЕТ. Он ничего не передаёт. Здесь нет и не может быть
// обращения к CAN, к TWAI или к любому транспорту: модуль платформенно
// нейтрален и собирается на host целиком. Передача — отдельный слой, который
// в профиле LAB_SAFE не существует как код.
//
// ПОРЯДОК СЛОЁВ, и он важен:
//
//   Refloat -> VESC_IF -> Motor Gate -> Dual Motor Coordinator -> transport
//                            |                   |
//                     «можно ли вообще»   «что именно двум половинам»
//
// Motor Gate отвечает на вопрос о праве, координатор — о содержании. Слить их
// в один модуль означало бы получить место, где решение о безопасности
// принимается вперемешку с арифметикой, и проверять пришлось бы всё сразу.
//
// СЕМАНТИКА ВЕЛИЧИНЫ. Запрошенный ток — это ток ОДНОГО мотора, а не сумма по
// доске. Обоснование в docs/motor_safety_contract.md §6; коротко: Refloat
// построен вокруг одного мотора, и его же соглашение уже зафиксировано в
// logical_motor.h и docs/motor_semantics.md, где ток агрегируется средним, а
// не суммой. Менять соглашение на полпути — верный способ получить вдвое
// больший момент, чем рассчитан.
#pragma once

#include "../safety/fc_build_profile.h"

#include <stdbool.h>
#include <stdint.h>

#define FC_DUAL_A 0u // ID 118, мотор A
#define FC_DUAL_B 1u // ID 100, мотор B
#define FC_DUAL_HALVES 2u

// --------------------------------------------------------------- свежесть
//
// Три границы, и каждая обязана быть заметно короче чужого сторожевого
// таймера ESC в 50 мс, иначе наш барьер не успевает ни на что повлиять.

// Команда от контура. Темп контура 2000 мкс, допуск 4 пропуска подряд —
// ровно политика, проверенная на v0.7C (fc_command_watchdog.h).
#define FC_DUAL_COMMAND_FRESH_US 20000u

// Семпл IMU. Та же граница: обе величины питают одно и то же решение, и
// разные пороги здесь дали бы окно, в котором тяга разрешена по одной
// проверке и запрещена по другой.
#define FC_DUAL_IMU_FRESH_US 20000u

// Обратная связь половины. Периодический STATUS идёт 50 Гц, то есть каждые
// 20 мс; три подряд пропущенных кадра — это уже не единичная потеря.
// Осознанно ДЛИННЕЕ командной границы: молчание половины опаснее
// подтверждать поспешно, чем собственный сбой.
#define FC_DUAL_NODE_FRESH_US 60000u

// Допустимый разбег между командами двум половинам при последовательной
// передаче. Расширенный кадр в 8 байт на 500 кбит/с это около 256 мкс;
// миллисекунда — запас вчетверо и 2 % от таймаута ESC.
#define FC_DUAL_MAX_SKEW_US 1000u

// ------------------------------------------------------------- причины отказа
//
// Маска, а не первая причина: «нельзя» без перечня всех невыполненных условий
// заставляет чинить по одному и каждый раз заново гадать, что ещё не так.
//
// Здесь НЕТ причины «транспорта не существует», и это осознанно. Наличие
// транспорта — свойство сборки, за которое отвечают Motor Gate (его backend
// объявлен только вне LAB_SAFE) и отсутствие реализации сериализатора.
// Координатор решает только вопрос о праве на тягу, исходя из состояния
// системы. Смешав это с профилем, мы получили бы модуль, который невозможно
// проверить в той сборке, в которой он живёт.
typedef enum {
    FC_DUAL_DENY_NOT_ARMED = 1u << 0,      // разрешение не запрашивалось
    FC_DUAL_DENY_SUPERVISOR = 1u << 1,     // supervisor не в разрешающем состоянии
    FC_DUAL_DENY_LATCHED = 1u << 2,        // залипший отказ, нужен явный re-arm
    FC_DUAL_DENY_IMU_STALE = 1u << 3,
    FC_DUAL_DENY_COMMAND_STALE = 1u << 4,
    FC_DUAL_DENY_NODE_A_STALE = 1u << 5,
    FC_DUAL_DENY_NODE_B_STALE = 1u << 6,
    FC_DUAL_DENY_NODE_A_FAULT = 1u << 7,   // ненулевой mc_fault_code половины A
    FC_DUAL_DENY_NODE_B_FAULT = 1u << 8,
    FC_DUAL_DENY_CAN_DEGRADED = 1u << 9,   // error passive, рост счётчиков
    FC_DUAL_DENY_BUS_OFF = 1u << 10,
    FC_DUAL_DENY_VALUE_INVALID = 1u << 11, // NaN или Inf
    FC_DUAL_DENY_VALUE_RANGE = 1u << 12,   // вне предела тока
    FC_DUAL_DENY_PARTIAL_SEND = 1u << 13,  // прошлая пара ушла не целиком
    FC_DUAL_DENY_REASON_COUNT = 14
} FcDualDenyReason;

// -------------------------------------------------------------------- входы
//
// Всё время подаётся снаружи: модуль не знает, откуда оно берётся, и потому
// проверяется на host детерминированно, без платы и без сна.
typedef struct {
    uint64_t now_us;

    bool supervisor_allows; // Supervisor в состоянии, разрешающем тягу
    uint64_t last_imu_us;
    uint64_t last_command_us;
    uint64_t last_feedback_us[FC_DUAL_HALVES];

    uint8_t esc_fault_code[FC_DUAL_HALVES]; // сырой mc_fault_code, 0 = нет
    bool can_bus_off;
    bool can_degraded; // error passive либо растущие счётчики ошибок

    float requested_current_a; // ток ОДНОГО мотора, см. заголовок файла
} FcDualMotorInputs;

// ------------------------------------------------------------- конфигурация
//
// Знак — конфигурационное свойство, а не «-1» внутри транспорта (ТЗ §7).
// На v0.8B физически доказано, что положительная команда обеим половинам даёт
// движение вперёд, поэтому обе инверсии по умолчанию выключены. Поле
// существует, чтобы будущая инверсия была видна в конфигурации и в отчёте, а
// не спряталась в арифметике.
typedef struct {
    bool invert[FC_DUAL_HALVES];
    float current_limit_a; // предел по модулю на КАЖДЫЙ мотор
} FcDualMotorConfig;

// --------------------------------------------------------------------- план
typedef struct {
    bool send;                          // пару вообще передавать
    float current_a[FC_DUAL_HALVES];    // что уйдёт каждой половине
    uint8_t order[FC_DUAL_HALVES];      // порядок передачи, всегда A затем B
    uint32_t deny_mask;                 // если send == false — почему
    // Номер разрешённой пары. Монотонно растёт и НЕ увеличивается на отказах:
    // разрыв в последовательности означает, что пара не ушла, а не что её
    // потеряли по дороге. По нему же сопоставляются команда и телеметрия.
    uint32_t sequence;
} FcDualMotorPlan;

typedef struct {
    uint64_t evaluations;
    uint64_t permits_granted;
    uint64_t permits_denied;
    uint64_t deny_by_reason[FC_DUAL_DENY_REASON_COUNT];
    uint64_t partial_sends;
    uint64_t latched_faults;
    uint32_t last_deny_mask;
    uint32_t sequence;
    bool latched;
    bool armed;
} FcDualMotorStats;

// ------------------------------------------------- вооружение оператором
//
// Способность на этапе компиляции и разрешение во время работы — разные вещи.
// Экспериментальный профиль означает лишь, что транспорт существует. Команда
// может уйти только после явного действия человека, и только когда выполнены
// все условия ниже.
//
// Автоматического вооружения нет ни при каких обстоятельствах: восстановление
// входов после отказа НЕ возвращает право на тягу.
typedef enum {
    FC_DUAL_ARM_DENY_SUPERVISOR = 1u << 0,  // supervisor нездоров или в FAULT
    FC_DUAL_ARM_DENY_IMU = 1u << 1,
    FC_DUAL_ARM_DENY_NODE_A = 1u << 2,
    FC_DUAL_ARM_DENY_NODE_B = 1u << 3,
    FC_DUAL_ARM_DENY_LATCHED = 1u << 4,
    FC_DUAL_ARM_DENY_CAN = 1u << 5,
    FC_DUAL_ARM_DENY_BATTERY_MODEL = 1u << 6,
    FC_DUAL_ARM_DENY_MOTOR_MODEL = 1u << 7,
    FC_DUAL_ARM_DENY_BOOT = 1u << 8,
    FC_DUAL_ARM_DENY_REALTIME = 1u << 9,
    FC_DUAL_ARM_DENY_COUNT = 10
} FcDualArmDeny;

typedef struct {
    bool supervisor_healthy;
    bool imu_healthy;
    bool node_healthy[FC_DUAL_HALVES];
    bool can_healthy;
    bool battery_model_valid;
    bool motor_model_valid;
    bool boot_complete;
    bool realtime_qualified;
} FcDualArmInputs;

/**
 * Попытаться вооружить. Возвращает true только если выполнены ВСЕ условия;
 * иначе в deny_mask лежат все невыполненные, а состояние не меняется.
 */
bool fc_dual_motor_try_arm(const FcDualArmInputs *in, uint32_t *deny_mask);

const char *fc_dual_motor_arm_deny_name(FcDualArmDeny r);

void fc_dual_motor_init(const FcDualMotorConfig *cfg);

/** Конфигурация по умолчанию: инверсий нет, предел — временный прокрутный. */
FcDualMotorConfig fc_dual_motor_default_config(void);

/**
 * Безусловное вооружение. Существует ТОЛЬКО для host-тестов, где условия
 * подаются сценарием, а не системой. На плате используется try_arm.
 */
void fc_dual_motor_arm(void);

/** Снять разрешение. Отдельного основания не требует. */
void fc_dual_motor_disarm(void);

/**
 * Снять залипший отказ. Предусмотренная процедура из ТЗ §3 I4: возврат к
 * тяге после отказа возможен только так, и только через повторный arm.
 */
void fc_dual_motor_clear_latch(void);

/**
 * Построить план. Единственная функция, решающая, появится ли пара команд.
 *
 * Отсутствие свежей команды даёт план с send == false и НЕ повторяет прошлое
 * значение: удержание последнего значения при пропавшем источнике — ровно то,
 * чего нельзя делать (ТЗ §3 I10).
 */
FcDualMotorPlan fc_dual_motor_plan(const FcDualMotorInputs *in);

/**
 * Сообщить, чем кончилась передача пары. Вызывается транспортом.
 * Частичная передача защёлкивает отказ: половины разошлись, и продолжать
 * нормальную работу нельзя (ТЗ §4 E).
 */
void fc_dual_motor_report_send(bool sent_a, bool sent_b, uint64_t now_us);

/** Разбег между фактическими передачами последней пары, мкс. */
uint32_t fc_dual_motor_last_skew_us(void);
void fc_dual_motor_note_skew(uint32_t skew_us);

FcDualMotorStats fc_dual_motor_stats(void);
const char *fc_dual_motor_reason_name(FcDualDenyReason r);

/** Худшая граница окна, в котором половины могут иметь разную тягу, мкс. */
uint32_t fc_dual_motor_worst_asymmetry_us(void);
