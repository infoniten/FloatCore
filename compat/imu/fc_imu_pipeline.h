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
#include "fc_imu_calibration.h"
#include "fc_imu_detect.h"
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

// Состояние измерения ОСТАТОЧНОГО смещения гироскопа при старте.
//
// Именно остаточного: с v0.6E основным источником истины является постоянная
// калибровка (fc_imu_calibration.h), как и в прошивке VESC. Стартовое
// измерение больше ничего не вычитает — оно измеряет, что осталось ПОСЛЕ
// применения постоянных смещений, и служит проверкой исправности. Так двойная
// компенсация исключена по построению, а не по внимательности (ТЗ v0.6E §10,
// вариант A).
typedef enum {
    FC_IMU_RESIDUAL_COLLECTING = 0,  // копим семплы, доска должна быть неподвижна
    FC_IMU_RESIDUAL_DONE,            // остаток измерен
    FC_IMU_RESIDUAL_REJECTED,        // остаток неправдоподобно велик
} FcImuResidualState;

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

    FcImuResidualState residual_state;
    uint32_t residual_samples;
    uint32_t residual_restarts;
    float residual_bias_dps[3];  // остаток ПОСЛЕ постоянной калибровки, °/с
} FcImuPipelineStats;

typedef struct {
    // Номинальный период выдачи датчика, мкс. Нужен только для того, чтобы
    // отличить нормальный интервал между принятыми семплами от пропуска.
    uint32_t nominal_period_us;
    // Сколько подряд одинаковых сырых семплов считать залипанием датчика.
    // Дубликат сам по себе штатен: контур опрашивает датчик чаще, чем тот
    // выдаёт данные. Но длинная серия дубликатов означает, что датчик замер.
    uint32_t stuck_duplicate_limit;

    // --- измерение остаточного смещения гироскопа при старте ---------------
    //
    // Не компенсация, а проверка. Постоянная калибровка уже вычла смещение;
    // здесь измеряется, что осталось. Большой остаток означает, что калибровка
    // устарела (смещение гироскопа зависит от температуры) или что доска при
    // включении двигалась.
    //
    // Одновременно это единственная защита от старта на движущейся доске:
    // пока измерение не завершилось, imu_startup_done() возвращает false.
    uint32_t residual_samples;
    // Порог неподвижности. Доска, которую держат в руках, даёт десятки °/с;
    // 3 °/с — заведомо выше шума покоя (сотые) и смещения (единицы), но
    // заведомо ниже любого реального движения.
    float residual_still_gyro_dps;
    // Допуск на модуль ускорения: при переносе доски он уходит от 1 g.
    float residual_still_accel_tol_g;
    // Потолок правдоподобия для самого смещения. Больше — это не смещение, а
    // движущаяся доска или неисправный датчик; такое не применяется.
    float residual_max_bias_dps;
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
const char *fc_imu_residual_state_name(FcImuResidualState s);
/** Завершилось ли измерение остатка (успехом или отказом). */
bool fc_imu_pipeline_residual_ready(void);

// ------------------------------------------------------- постоянная калибровка
/** Применяемая калибровка. Действует со следующего принятого семпла. */
void fc_imu_pipeline_set_calibration(const FcImuCalibration *c);
FcImuCalibration fc_imu_pipeline_calibration(void);
/** Перезапустить фильтр ориентации: нужен после смены калибровки. */
void fc_imu_pipeline_reset_ahrs(void);
