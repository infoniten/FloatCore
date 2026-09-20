// Теневой наблюдатель команды Refloat (ТЗ v0.9D §6, §7, §11, §12).
//
// ЧТО ЭТО. Место, где команда балансировочного контура записывается и
// анализируется — и НЕ передаётся никуда дальше.
//
//     IMU → Refloat → запрошенный ток → ЗДЕСЬ. Всё.
//
// ЧЕГО ЗДЕСЬ НЕТ. Пути к транспорту. Модуль не знает ни про Motor Gate, ни
// про CAN, ни про координатор; он умеет только писать в кольцо и считать.
// Физическая передача из него невозможна не по правилу, а потому, что
// вызывать нечего.
//
// ПОЧЕМУ ЭТО НЕ ОСЛАБЛЕНИЕ MOTOR GATE. Наблюдение происходит ДО него:
// команду видно там, где она рождается, а не там, где решается её судьба.
// Ослабить Gate ради наблюдения означало бы сломать ровно тот барьер, ради
// которого всё строилось, — и ТЗ прямо называет это признаком неверной
// архитектуры.
#pragma once

#include <stdbool.h>
#include <stdint.h>

// Границы физической мёртвой зоны, измеренные на v0.9B и объяснённые на
// v0.9C (некомпенсированное мёртвое время инвертора, расчётный порог
// 0.381 А). Здесь они НЕ компенсируются — только используются как границы
// для подсчёта, сколько времени команда проводит в неэффективной области.
#define FC_SHADOW_DEADZONE_LOW_A 0.30f
#define FC_SHADOW_DEADZONE_HIGH_A 0.40f

// Кольцо прореживается: статистика считается на КАЖДОЙ итерации 500 Гц, а в
// историю попадает каждая десятая. Без этого 256 записей покрывают полсекунды
// и любое движение рукой в них не помещается; с прореживанием 512 записей —
// это десять секунд, чего хватает на манёвр целиком.
//
// Прореживается только история. Пики, доли мёртвой зоны и поведение
// интегратора считаются по всем семплам: терять их ради удобства чтения
// нельзя.
#define FC_SHADOW_RING 512u
#define FC_SHADOW_DECIMATION 10u

// Границы разбиения по модулю ошибки угла, градусы. Верхняя корзина —
// «всё остальное».
#define FC_SHADOW_ANGLE_BINS 6u
extern const float FC_SHADOW_ANGLE_EDGES[FC_SHADOW_ANGLE_BINS];

/** Предел ESC, по которому считается упор в корзинах. */
void fc_shadow_set_esc_limit(float amps);

typedef struct {
    uint64_t timestamp_us;
    float current;      // то, что запросил Refloat
    float pitch;        // градусы
    float roll;
    float pitch_rate;   // °/с
    float setpoint;
    float pid_p;
    float pid_i;
    float pid_rate_p;
    uint8_t sat;              // SetpointAdjustmentType
    uint8_t refloat_state;
    uint8_t supervisor_state;
    uint8_t imu_permit;       // FcImuPermit
    bool deliverable;         // разрешил бы Motor Gate передачу
} FcShadowSample;

typedef struct {
    uint64_t samples;
    uint64_t dropped;          // кольцо переполнено между чтениями

    // Занятость мёртвой зоны (ТЗ §11).
    uint64_t below_low;        // |I| < 0.30 А — физически неэффективно
    uint64_t in_band;          // 0.30…0.40 А — частичный отклик
    uint64_t above_high;       // > 0.40 А — устойчивый отклик

    float peak_positive;
    float peak_negative;
    float peak_abs_i_term;     // наибольшая по модулю интегральная часть
    float last_i_term;

    uint64_t zero_crossings;
    uint64_t saturation_samples;
    uint64_t deliverable_samples; // сколько раз команда была бы разрешена

    // Интегратор внутри мёртвой зоны (ТЗ §12): накапливается ли он там, где
    // команда ничего не делает.
    float i_term_at_deadzone_entry;
    float max_i_growth_in_deadzone;
    uint64_t deadzone_entries;

    // Взаимодействие мёртвой зоны и экспериментального предела (ТЗ §14).
    // Считается по величине, которая ДОШЛА БЫ до мотора: min(|запрос|,
    // предел опыта) со знаком запроса.
    uint64_t env_ineffective;  // < 0.30 А — физически неэффективно
    uint64_t env_transition;   // 0.30…0.40 А
    uint64_t env_effective;    // 0.40 А … предел, не в упоре
    uint64_t env_saturated;    // в упоре в предел опыта

    // Разбиение по модулю ошибки угла (ТЗ v0.9E §13). Вопрос, на который оно
    // отвечает: при каких углах будущий замкнутый контур просто упрётся в
    // предел, то есть выродится в релейный.
    uint64_t bin_n[FC_SHADOW_ANGLE_BINS];
    float bin_sum_raw[FC_SHADOW_ANGLE_BINS];       // сумма |запроса|
    uint64_t bin_saturated_esc[FC_SHADOW_ANGLE_BINS];  // упор в предел ESC
    uint64_t bin_saturated_env[FC_SHADOW_ANGLE_BINS];  // упор в предел опыта

    // Крайние углы и команда в них (ТЗ §9): при насыщении по всей развёртке
    // именно они показывают, где насыщение начинается.
    float min_pitch, max_pitch;
    float current_at_min_pitch, current_at_max_pitch;
    bool have_pitch;
} FcShadowStats;

// Экспериментальный предел (ТЗ v0.9E §9). НЕ предел ESC и не подменяет его:
// он всегда строже и применяется ТОЛЬКО к теневому разбору. Физически ничего
// не ограничивает, потому что физически ничего и не передаётся.
void fc_shadow_set_envelope(float amps);
float fc_shadow_envelope(void);

void fc_shadow_init(void);
void fc_shadow_reset(void);

/**
 * Записать одну итерацию. Вызывается из пути контура 500 Гц, поэтому только
 * копирует и считает: ни печати, ни аллокаций, ни блокировок.
 */
void fc_shadow_record(const FcShadowSample *s);

FcShadowStats fc_shadow_stats(void);

/** Последние записи для разбора. Возвращает сколько скопировано. */
uint32_t fc_shadow_tail(FcShadowSample *out, uint32_t max);
