// Motor Gate — единственная точка, через которую может пройти запрос тяги
// (ТЗ v0.6A §11, §12).
//
// Идея: Refloat не разговаривает с backend-ом напрямую никогда. Любой из
// пяти motor-output вызовов VESC_IF сводится сюда, здесь применяется решение
// Safety Supervisor, и только после этого — если backend вообще существует —
// запрос уходит наружу.
//
//   Refloat -> VESC_IF->mc_set_* -> fc_motor_gate_request() -> policy -> backend
//
// Обходных путей нет по построению: backend доступен только через указатель
// внутри этого модуля, а сам модуль не экспортирует ничего, кроме запроса.
//
// Модуль платформенно-нейтрален: время подаётся снаружи, логирование — через
// callback. Это позволяет гонять его в host-тестах детерминированно.
#pragma once

#include "fc_build_profile.h"

#include <stdbool.h>
#include <stdint.h>

// Полный перечень выходов на мотор в SDK VESC. Список получен не по памяти,
// а вычиткой refloat-upstream/vesc_pkg_lib/vesc_c_if.h:436-447, 476, 653 —
// это все функции, способные подать что-либо на мотор, включая
// mc_release_motor (снятие удержания) и foc_play_tone (звук подаётся
// напряжением на обмотки, то есть это тоже выход).
typedef enum {
    FC_MOTOR_REQ_CURRENT = 0,
    FC_MOTOR_REQ_BRAKE_CURRENT,
    FC_MOTOR_REQ_CURRENT_REL,
    FC_MOTOR_REQ_BRAKE_CURRENT_REL,
    FC_MOTOR_REQ_DUTY,
    FC_MOTOR_REQ_DUTY_NORAMP,
    FC_MOTOR_REQ_PID_SPEED,
    FC_MOTOR_REQ_PID_POS,
    FC_MOTOR_REQ_HANDBRAKE,
    FC_MOTOR_REQ_HANDBRAKE_REL,
    FC_MOTOR_REQ_RELEASE,
    FC_MOTOR_REQ_TONE,
    FC_MOTOR_REQ_KIND_COUNT
} FcMotorRequestKind;

// Кто просит. Различать источники приходится потому, что на v0.9A выход
// Refloat НЕ подключается к реальному транспорту (ТЗ §24): балансировочный
// контроллер с живым IMU, получив право слать, немедленно начал бы
// балансировать, а это ровно то, что запрещено. Экспериментальный источник —
// отдельный и явный.
typedef enum {
    FC_MOTOR_ORIGIN_REFLOAT = 0, // контур балансировки через VESC_IF
    FC_MOTOR_ORIGIN_EXPERIMENT,  // явная команда оператора на вывешенном колесе
    FC_MOTOR_ORIGIN_COUNT
} FcMotorOrigin;

typedef enum {
    FC_GATE_ALLOWED = 0,       // политика разрешила (в LAB_SAFE недостижимо)
    FC_GATE_REJECTED_DISARMED, // supervisor не в состоянии, разрешающем тягу
    FC_GATE_REJECTED_FAULT,    // supervisor в FAULT
    FC_GATE_REJECTED_INVALID,  // NaN/Inf или значение вне допустимого диапазона
    FC_GATE_REJECTED_NO_BACKEND, // backend отсутствует (LAB_SAFE)
    FC_GATE_REJECTED_ORIGIN,     // источник не допущен в этом профиле
    FC_GATE_REJECTED_DEADLINE    // срок прогона замкнутого контура истёк (v0.9K)
} FcGateVerdict;

typedef struct {
    uint64_t requests_total;
    uint64_t allowed_by_policy;
    uint64_t rejected_disarmed;
    uint64_t rejected_fault;
    uint64_t rejected_invalid;
    uint64_t rejected_no_backend;
    uint64_t rejected_origin;
    uint64_t by_origin[FC_MOTOR_ORIGIN_COUNT];
    uint64_t delivered_to_backend;
    // Инкрементируется ИСКЛЮЧИТЕЛЬНО backend-ом, который физически передал
    // команду наружу. В LAB_SAFE такого backend-а не существует, поэтому
    // счётчик обязан всегда оставаться нулём.
    uint64_t physically_sent;

    uint64_t by_kind[FC_MOTOR_REQ_KIND_COUNT];
    float last_value[FC_MOTOR_REQ_KIND_COUNT];
    uint64_t keepalive_calls;
    uint64_t last_request_us;
} FcMotorGateStats;

// Backend — то, что умеет физически отправить команду. В LAB_SAFE не
// регистрируется вовсе.
typedef struct {
    const char *name;
    // Возвращает true, если команда действительно ушла в железо.
    bool (*send)(FcMotorRequestKind kind, float value, void *ctx);
    void *ctx;
} FcMotorBackend;

/**
 * Какие источники допущены до backend-а.
 *
 * На v0.9A допущен ТОЛЬКО экспериментальный: подключение выхода Refloat к
 * реальному транспорту — отдельное решение отдельного этапа.
 */
#define FC_GATE_ALLOWED_ORIGINS_V09A (1u << FC_MOTOR_ORIGIN_EXPERIMENT)

#if FC_CLOSED_LOOP_AVAILABLE
// Прогон замкнутого контура (ТЗ v0.9K §4, §7).
//
// Единственный способ впустить источник REFLOAT в гейт, и он работает только
// со СРОКОМ. Функция v0.9G, ставившая бит REFLOAT без срока, удалена.
//
// Пока прогон идёт, гейт на КАЖДОМ запросе REFLOAT:
//   * проверяет срок; по истечении снимает бит REFLOAT и отвечает
//     REJECTED_DEADLINE — повторного старта без нового вооружения нет;
//   * зажимает величину оболочкой ±envelope — единственный явный слой
//     зажатия; сколько раз он сработал, считается.
//
// Остальные двери на месте: супервизор обязан быть в RUNNING, координатор —
// вооружён.
bool fc_motor_gate_arm_burst(uint64_t deadline_us, float envelope_a);
void fc_motor_gate_revoke_burst(void);

typedef struct {
    bool active;
    uint64_t deadline_us;
    float envelope_a;
    uint64_t allowed;          // запросов REFLOAT, прошедших гейт
    uint64_t clamped;          // из них зажатых оболочкой
    uint64_t rejected_deadline;
    float last_requested;
    float last_delivered;      // после зажатия
} FcGateBurstStats;

FcGateBurstStats fc_motor_gate_burst_stats(void);
#endif

void fc_motor_gate_init(void);

/**
 * Зарегистрировать backend физической отправки.
 *
 * В сборке LAB_SAFE вызов этой функции запрещён на этапе компиляции: сам
 * прототип объявляется только вне LAB_SAFE (см. fc_build_profile.h), поэтому
 * появление backend-а в лабораторной прошивке невозможно, а не «запрещено
 * правилом».
 */
#if FC_MOTOR_BACKEND_AVAILABLE
void fc_motor_gate_set_backend(const FcMotorBackend *backend);
#endif

/** Имя текущего backend-а или "none (blocked)". */
const char *fc_motor_gate_backend_name(void);

/**
 * Единственная точка входа для любого запроса тяги.
 * now_us — монотонное время платформы.
 */
FcGateVerdict fc_motor_gate_request(FcMotorRequestKind kind, float value, uint64_t now_us);

/**
 * То же, но с явным источником. Старая форма остаётся синонимом источника
 * REFLOAT: все вызовы из VESC_IF приходят оттуда, и менять их не нужно.
 */
FcGateVerdict fc_motor_gate_request_from(FcMotorOrigin origin, FcMotorRequestKind kind, float value,
                                         uint64_t now_us);

/** Битовая маска допущенных источников. По умолчанию — только эксперимент. */
void fc_motor_gate_set_allowed_origins(uint32_t mask);
uint32_t fc_motor_gate_allowed_origins(void);
const char *fc_motor_gate_origin_name(FcMotorOrigin o);

/** Аналог VESC_IF->timeout_reset: продление watchdog, тяги не запрашивает. */
void fc_motor_gate_keepalive(void);

FcMotorGateStats fc_motor_gate_stats(void);
const char *fc_motor_gate_kind_name(FcMotorRequestKind kind);
const char *fc_motor_gate_verdict_name(FcGateVerdict v);

/** Максимальный по модулю запрос, который политика вообще считает осмысленным. */
void fc_motor_gate_set_value_limit(float amps_or_unit);
