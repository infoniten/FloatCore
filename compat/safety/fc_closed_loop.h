// Вход в замкнутый контур на реальный мотор (ТЗ v0.9G §13, §14).
//
// ЗАЧЕМ ОТДЕЛЬНЫЙ МОДУЛЬ. Вооружение координатора отвечает на вопрос «можно ли
// вообще слать команды паре». Здесь отвечают на другой: «можно ли отдать
// управление моментом программе». Разница в том, кто задаёт величину и
// длительность. При явной команде оператора их задаёт человек; в замкнутом
// контуре — расчёт, и ошибка в нём становится непрерывным моментом.
//
// ПОЧЕМУ ВХОД НЕ СРАЗУ ПОСЛЕ ARM. Между «связь и датчики живы» и «доска стоит
// так, что контур имеет смысл» лежит целый набор условий, которые вооружение
// не проверяет и проверять не должно.
//
// ПОРОГИ ВХОДА НЕ ВЫДУМАНЫ. Они выведены из оболочки тока и коэффициентов,
// подтверждённых теневым аудитом v0.9F, по одному правилу: **вход в контур не
// должен начинаться в насыщении**. Контур, который с первой итерации стоит в
// упоре, — это не регулятор, а реле, и поведение его нельзя ни предсказать,
// ни разобрать. Отсюда:
//
//     предельный угол   = оболочка / (ампер на градус)
//     предельная скорость = оболочка / (ампер на градус в секунду)
//
// Собственные пороги Refloat (startup_pitch_tolerance 4°,
// startup_roll_tolerance 45°) при этом остаются верхней границей: наши строже,
// и ослаблять их до значений Refloat здесь нельзя.
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    FC_CL_DENY_PROFILE = 0,      // сборка без замкнутого контура
    FC_CL_DENY_GATE_DISABLED,    // источник REFLOAT не впущен в маску гейта
    FC_CL_DENY_NOT_ARMED,        // координатор не вооружён
    FC_CL_DENY_SUPERVISOR,       // супервизор не в состоянии, откуда можно войти
    FC_CL_DENY_IMU_PERMIT,       // право на тягу от политики IMU не OK
    FC_CL_DENY_CALIBRATION,      // ориентация датчика не откалибрована
    FC_CL_DENY_NODE_STALE,       // хотя бы одна половина молчит
    FC_CL_DENY_CAN_UNHEALTHY,    // шина в отказе
    FC_CL_DENY_MOTOR_MODEL,      // потокосцепление/полюса не прочитаны
    FC_CL_DENY_BATTERY_MODEL,    // модель батареи недостоверна
    FC_CL_DENY_LIMITS,           // пределы тока не синхронизированы с ESC
    FC_CL_DENY_REALTIME,         // реальное время не квалифицировано
    FC_CL_DENY_TEST_CONDITION,   // состояние footpad не задано явно
    FC_CL_DENY_PITCH_WINDOW,     // угол вне окна входа
    FC_CL_DENY_RATE_WINDOW,      // угловая скорость вне окна входа
    FC_CL_DENY_ENVELOPE,         // оболочка не задана или вне допустимого
    FC_CL_DENY_COUNT
} FcClosedLoopDeny;

typedef struct {
    // Оболочка тока НА КАЖДЫЙ мотор, А. Строго ниже предела ESC.
    float envelope_a;
    // Масштаб контура: ампер на градус и ампер на градус в секунду.
    // Берутся из тех же коэффициентов, что подтверждены теневым аудитом.
    float amps_per_deg;
    float amps_per_deg_per_s;
    // Верхняя граница от самого Refloat: наше окно не смеет её превысить.
    float refloat_pitch_tolerance_deg;
} FcClosedLoopConfig;

typedef struct {
    bool gate_closed_loop_enabled;
    bool coordinator_armed;
    bool supervisor_entry_allowed;   // READY или ARMED, не FAULT
    uint32_t imu_permit;             // FcImuPermit: 0 = OK
    bool calibration_valid;
    bool node_a_fresh;
    bool node_b_fresh;
    bool can_healthy;
    bool motor_model_valid;
    bool battery_model_valid;
    bool limits_synchronized;
    bool realtime_qualified;
    bool test_condition_explicit;
    float balance_pitch_deg;
    float pitch_rate_dps;
} FcClosedLoopInputs;

/** Окна входа, посчитанные из конфигурации. Показываются оператору до входа. */
typedef struct {
    float pitch_window_deg;
    float rate_window_dps;
} FcClosedLoopWindows;

FcClosedLoopWindows fc_closed_loop_windows(const FcClosedLoopConfig *cfg);

/**
 * Проверить все условия входа. Возвращает true только если выполнено ВСЁ.
 *
 * deny_mask заполняется целиком, а не до первого отказа: оператору нужно
 * видеть все причины сразу, иначе устранение идёт по одной за перезапуск.
 */
bool fc_closed_loop_may_enter(const FcClosedLoopConfig *cfg, const FcClosedLoopInputs *in,
                              uint32_t *deny_mask);

const char *fc_closed_loop_deny_name(FcClosedLoopDeny r);
