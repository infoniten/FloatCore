#include "fc_imu_source.h"

#include "fc_platform.h"

#include "../../../compat/imu/fc_imu_pipeline.h"

#include <string.h>

static FcImuSource g_source = FC_IMU_SOURCE_MOCK;

bool fc_imu_source_start(FcImuSource source) {
    g_source = source;
    if (source == FC_IMU_SOURCE_REAL) {
        return fc_imu_rt_start();
    }
    fc_imu_mock_start();
    return true;
}

FcImuSource fc_imu_source(void) {
    return g_source;
}

const char *fc_imu_source_name_of(FcImuSource s) {
    return s == FC_IMU_SOURCE_REAL ? "REAL ICM-20948" : "MOCK (покой)";
}

const char *fc_imu_source_name(void) {
    return fc_imu_source_name_of(g_source);
}

bool fc_imu_source_available(void) {
    return g_source == FC_IMU_SOURCE_REAL ? fc_imu_rt_available() : true;
}

// ------------------------------------------------------ единый фасад для VESC_IF
//
// Все функции ниже — единственный мост между VESC_IF и источником данных.
// Ветвление по источнику собрано здесь, а не размазано по fc_vesc_if.c:
// иначе однажды одна из шести функций осталась бы смотреть не туда.

int fc_imu_rate_hz(void) {
    return g_source == FC_IMU_SOURCE_REAL ? fc_imu_rt_rate_hz() : fc_imu_mock_rate_hz();
}

bool fc_imu_startup_done(void) {
    return g_source == FC_IMU_SOURCE_REAL ? fc_imu_rt_startup_done() : fc_imu_mock_startup_done();
}

void fc_imu_set_callback(void (*cb)(float *acc, float *gyro, float *mag, float dt)) {
    fc_imu_rt_set_callback(cb);
    fc_imu_mock_set_callback(cb);
}

void fc_imu_get_state(
    float *roll, float *pitch, float *yaw, float accel[3], float gyro[3], float quat[4]
) {
    if (g_source != FC_IMU_SOURCE_REAL) {
        fc_imu_mock_get_state(roll, pitch, yaw, accel, gyro, quat);
        return;
    }

    // Один и тот же снимок для всех выходов: углы, ускорение и угловая
    // скорость обязаны относиться к одному моменту времени (ТЗ v0.6D §5).
    FcImuSample s = fc_imu_pipeline_sample();
    if (roll) {
        *roll = s.roll_rad;
    }
    if (pitch) {
        *pitch = s.pitch_rad;
    }
    if (yaw) {
        *yaw = s.yaw_rad;
    }
    if (accel) {
        memcpy(accel, s.accel_g, sizeof(s.accel_g));
    }
    if (gyro) {
        // ВНИМАНИЕ: здесь °/с, а не рад/с — в отличие от массива, который
        // уходит в callback. Это не описка: две функции VESC_IF имеют разные
        // единицы, обоснование — docs/imu_contract.md, раздел «Две разные
        // единицы гироскопа».
        memcpy(gyro, s.gyro_dps, sizeof(s.gyro_dps));
    }
    if (quat) {
        memcpy(quat, s.quat, sizeof(s.quat));
    }
}

uint32_t fc_imu_stack_watermark(void) {
    return g_source == FC_IMU_SOURCE_REAL ? fc_imu_rt_stack_watermark()
                                          : fc_imu_mock_stack_watermark();
}

void fc_imu_inject_stall(int ms) {
    if (g_source == FC_IMU_SOURCE_REAL) {
        fc_imu_rt_inject_stall(ms);
    } else {
        fc_imu_mock_inject_stall(ms);
    }
}
