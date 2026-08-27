// Тракт «сырой семпл датчика → снимок для Refloat» (ТЗ v0.6D §5, §6, §9).
//
// Платформенно-нейтрален намеренно: здесь живёт вся логика, которая может
// ошибиться (дедупликация, валидация, единицы, AHRS), и именно её надо
// покрывать host-тестами. Всё, что относится к FreeRTOS, I2C и таймерам,
// остаётся снаружи.
//
// Инвариант, ради которого модуль существует:
//
//     ни один семпл не попадает в Refloat, пока не признан валидным,
//     и один физический семпл даёт ровно одну итерацию контура.
#pragma once

#include "../safety/fc_imu_health.h"
#include "fc_ahrs.h"
#include "fc_imu_sample.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    // Транзакция чтения не удалась. В Refloat не идёт.
    FC_IMU_PIPE_READ_FAILED = 0,
    // Датчик отдал те же самые сырые слова, что и в прошлый раз: нового
    // физического семпла ещё нет. В Refloat не идёт, итерации контура нет.
    FC_IMU_PIPE_DUPLICATE,
    // Диагностика отвергла семпл (не конечные числа, немонотонное время,
    // неправдоподобный модуль, залипание, протухание). В Refloat не идёт.
    FC_IMU_PIPE_REJECTED,
    // Семпл принят. Только в этом случае вызывается callback Refloat.
    FC_IMU_PIPE_ACCEPTED,
} FcImuPipeVerdict;

// Состояние стартовой калибровки нулевого смещения гироскопа.
typedef enum {
    FC_IMU_CAL_COLLECTING = 0,  // копим семплы, доска должна быть неподвижна
    FC_IMU_CAL_DONE,            // смещение измерено и применяется
    FC_IMU_CAL_REJECTED,        // измеренное смещение неправдоподобно, не применено
} FcImuCalState;

typedef struct {
    uint64_t polls;              // сколько раз датчик опрашивался
    uint64_t read_failures;      // из них: транзакция не удалась
    uint64_t duplicates;         // из них: тот же физический семпл
    uint64_t rejected;           // из них: отвергнуто диагностикой
    uint64_t accepted;           // из них: принято и отдано в Refloat
    uint32_t consecutive_duplicates;
    uint32_t max_consecutive_duplicates;
    // Подозрение на пропуск физического семпла: интервал между принятыми
    // семплами оказался больше полутора номинальных периодов датчика.
    uint64_t suspected_skips;
    uint64_t max_gap_us;
    uint64_t min_gap_us;
    uint64_t sum_gap_us;
    uint64_t gaps_counted;

    FcImuCalState cal_state;
    uint32_t cal_samples;    // сколько семплов накоплено в текущей попытке
    uint32_t cal_restarts;   // сколько раз накопление сбрасывалось из-за движения
    float gyro_bias_dps[3];  // измеренное смещение, °/с
} FcImuPipelineStats;

typedef struct {
    // Номинальный период выдачи датчика, мкс. Нужен только для того, чтобы
    // отличить нормальный интервал между принятыми семплами от пропуска.
    uint32_t nominal_period_us;
    // Сколько подряд одинаковых сырых семплов считать залипанием датчика.
    // Дубликат сам по себе штатен: контур опрашивает датчик чаще, чем тот
    // выдаёт данные. Но длинная серия дубликатов означает, что датчик замер.
    uint32_t stuck_duplicate_limit;

    // --- стартовая калибровка нулевого смещения гироскопа -------------------
    //
    // Нужна не «для порядка». Платформенный AHRS работает с kp = 0.2, как
    // предписывает Refloat (main.c:210-214), то есть намеренно медленно и с
    // опорой на гироскоп. При такой настройке постоянное смещение даёт
    // установившуюся ошибку угла порядка bias/kp: измеренные 0.5…1.0 °/с
    // превращаются в 2.5…5 градусов, а рыскание уходит без ограничения.
    // Измерено на плате: pitch платформы +9.88 против balance_pitch +7.15.
    //
    // Сколько семплов усреднять. 500 при 500 Гц — одна секунда; этого
    // достаточно, чтобы шум усреднился до сотых °/с, и мало настолько, что
    // старт не затягивается.
    uint32_t cal_samples;
    // Порог неподвижности. Доска, которую держат в руках, даёт десятки °/с;
    // 3 °/с — заведомо выше шума покоя (сотые) и смещения (единицы), но
    // заведомо ниже любого реального движения.
    float cal_still_gyro_dps;
    // Допуск на модуль ускорения: при переносе доски он уходит от 1 g.
    float cal_still_accel_tol_g;
    // Потолок правдоподобия для самого смещения. Больше — это не смещение, а
    // движущаяся доска или неисправный датчик; такое не применяется.
    float cal_max_bias_dps;
} FcImuPipelineConfig;

FcImuPipelineConfig fc_imu_pipeline_default_config(uint32_t nominal_period_us);

void fc_imu_pipeline_init(const FcImuPipelineConfig *cfg, const FcImuHealthConfig *health);

/**
 * Принять результат одного опроса датчика.
 *
 * read_ok    — удалась ли транзакция;
 * raw_words  — 7 сырых слов, как пришли по шине; по ним и только по ним
 *              определяется, новый ли это физический семпл;
 * accel_g    — ускорение в g, оси датчика;
 * gyro_dps   — угловая скорость в °/с, оси датчика;
 * now_us     — момент завершения транзакции.
 *
 * Возвращает вердикт. Снимок обновляется только при ACCEPTED.
 */
FcImuPipeVerdict fc_imu_pipeline_submit(
    bool read_ok,
    const int16_t raw_words[7],
    const float accel_g[3],
    const float gyro_dps[3],
    float temperature_c,
    uint64_t now_us
);

/** Последний принятый снимок. Копия: потребитель не видит частичных записей. */
FcImuSample fc_imu_pipeline_sample(void);
bool fc_imu_pipeline_has_sample(void);
FcImuPipelineStats fc_imu_pipeline_stats(void);
const FcAhrs *fc_imu_pipeline_ahrs(void);
const char *fc_imu_pipe_verdict_name(FcImuPipeVerdict v);
const char *fc_imu_cal_state_name(FcImuCalState s);
/** Завершилась ли стартовая калибровка (успехом или отказом). */
bool fc_imu_pipeline_calibrated(void);
