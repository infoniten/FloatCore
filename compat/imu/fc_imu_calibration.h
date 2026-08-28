// Модель калибровки IMU, совместимая с прошивкой VESC (ТЗ v0.6E).
//
// FloatCore играет роль прошивки VESC, а не роль Refloat. Ориентация датчика и
// его монтажный угол принадлежат именно этому слою: в конфигурации Refloat нет
// ни одного поля ориентации, и получить его он не может
// (docs/imu_calibration_architecture.md).
//
// Поля и их смысл повторяют imu_config прошивки (bldc/datatypes.h):
//
//     float rot_roll;          градусы
//     float rot_pitch;         градусы
//     float rot_yaw;           градусы
//     float gyro_offsets[3];   °/с, в УЖЕ ПОВЁРНУТОЙ системе координат
//
// Своих полей здесь нет намеренно: там, где штатная модель уже существует,
// изобретать вторую — значит гарантированно с ней разойтись.
//
// Порядок применения взят из bldc/imu/imu.c и обязателен:
//
//     1. поворот матрицей, построенной из rot_yaw/rot_pitch/rot_roll;
//     2. вычитание смещений (они хранятся в повёрнутой системе!);
//     3. фильтрация;
//     4. AHRS и callback пакета.
//
// Смещения акселерометра (accel_offsets) в модели предусмотрены структурно, но
// на v0.6E не измеряются и не применяются: штатный мастер их калибрует
// отдельным шагом, а у нас для этого ещё нет процедуры. Поле оставлено нулевым,
// чтобы формат хранения не пришлось менять потом.
#pragma once

#include <stdbool.h>
#include <stdint.h>

// Версия формата хранения. Меняется при любом изменении раскладки слов:
// прочитав чужую версию, слой обязан отказаться, а не интерпретировать байты
// по-своему.
#define FC_IMU_CAL_VERSION 1u
#define FC_IMU_CAL_MAGIC 0x464C4331u  // "FLC1"

// Сколько 32-битных слов занимает калибровка в хранилище.
#define FC_IMU_CAL_WORDS 12

typedef enum {
    FC_IMU_CAL_NOT_CALIBRATED = 0,  // записи нет — работаем с единичной
    FC_IMU_CAL_VALID,               // прочитана и прошла проверки
    FC_IMU_CAL_INVALID,             // запись есть, но испорчена или не та версия
} FcImuCalStatus;

typedef struct {
    float rot_roll_deg;
    float rot_pitch_deg;
    float rot_yaw_deg;
    float gyro_offset_dps[3];
    float accel_offset_g[3];  // зарезервировано, на v0.6E всегда 0
} FcImuCalibration;

// Подготовленная калибровка: матрица уже посчитана.
//
// Существует по измеренной причине. Первая версия строила матрицу на каждом
// семпле — это шесть программных sinf/cosf на итерацию контура, и на плате
// это подняло долю опозданий контура с 13.9 % до 34.6 %, а медиану периода с
// 2120 до 2320 мкс. Матрица меняется только при смене калибровки, то есть раз
// в жизни прошивки; считать её 500 раз в секунду незачем.
typedef struct {
    float m[9];
    float gyro_offset_dps[3];
    float accel_offset_g[3];
} FcImuCalibrationPrepared;

void fc_imu_calibration_prepare(const FcImuCalibration *c, FcImuCalibrationPrepared *out);

/** Быстрый путь: применить уже подготовленную калибровку. */
void fc_imu_calibration_apply_prepared(
    const FcImuCalibrationPrepared *p,
    const float accel_in_g[3],
    const float gyro_in_dps[3],
    float accel_out_g[3],
    float gyro_out_dps[3]
);

/** Единичная калибровка: поворота нет, смещений нет. */
FcImuCalibration fc_imu_calibration_identity(void);

/** Отличается ли калибровка от единичной. */
bool fc_imu_calibration_is_identity(const FcImuCalibration *c);

/**
 * Матрица поворота 3x3 по строкам: m[0..2] — первая строка.
 *
 * Формулы посимвольно повторяют bldc/imu/imu.c. Проверять их надо не на
 * «выглядит правильно», а на совпадение с upstream: любая перестановка знака
 * здесь тихо развернёт ось.
 */
void fc_imu_calibration_matrix(const FcImuCalibration *c, float m[9]);

/**
 * Применить калибровку к одному семплу.
 *
 * Сначала поворот, затем вычитание смещений — порядок из upstream. Входные и
 * выходные массивы могут совпадать.
 */
void fc_imu_calibration_apply(
    const FcImuCalibration *c,
    const float accel_in_g[3],
    const float gyro_in_dps[3],
    float accel_out_g[3],
    float gyro_out_dps[3]
);

/**
 * Проверка правдоподобия. Отвергает то, что физически не может быть
 * калибровкой: не конечные числа, повороты вне ±180°, смещения гироскопа
 * больше 20 °/с (у исправного датчика они единицы).
 */
bool fc_imu_calibration_plausible(const FcImuCalibration *c);

/** Сериализация в слова хранилища. Всегда пишет ровно FC_IMU_CAL_WORDS слов. */
void fc_imu_calibration_serialize(const FcImuCalibration *c, uint32_t words[FC_IMU_CAL_WORDS]);

/**
 * Разбор слов хранилища.
 *
 * Возвращает NOT_CALIBRATED, если записи нет (стёртый носитель), INVALID при
 * несовпадении магии, версии, контрольной суммы или при неправдоподобных
 * значениях, и VALID в остальных случаях. При не-VALID *out получает единичную
 * калибровку: слой никогда не отдаёт наружу мусор.
 */
FcImuCalStatus fc_imu_calibration_deserialize(
    const uint32_t words[FC_IMU_CAL_WORDS], FcImuCalibration *out
);

const char *fc_imu_cal_status_name(FcImuCalStatus s);
