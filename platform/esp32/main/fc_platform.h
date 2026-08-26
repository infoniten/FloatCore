// Внутренний интерфейс платформенного слоя FloatCore на ESP32.
//
// ВАЖНО: этот заголовок НЕ включает заголовки Refloat. Причина — в
// refloat-upstream/src/time.h объявлен `typedef uint32_t time_t`, который
// конфликтует с newlib. Правило то же, что и в host-сборке
// (docs/threading_model.md): файлы, видящие ESP-IDF, не видят внутренностей
// Refloat, и наоборот. Единственный мост — compat/refloat_glue/refloat_facade.h.
#pragma once

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// --------------------------------------------------------------- приоритеты
// На VESC контур вытесняет главный поток, главный вытесняет aux.
// Воспроизводим тот же порядок; все задачи Refloat пиннятся на ядро 1
// (docs/threading_model.md §4, пункт 1).
#define FC_CORE_REALTIME 1
#define FC_CORE_HOUSEKEEPING 0

#define FC_PRIO_IMU 14       // источник ритма контура: imu_ref_callback
#define FC_PRIO_REFLOAT 12   // refloat_thd, 500 Гц
#define FC_PRIO_CONSOLE 4    // read-only CLI, ядро 0

// Refloat просит 1536 байт стека — этого мало для Xtensa (docs/vesc_if_contract.md §2).
#define FC_STACK_SCALE 8

// ------------------------------------------------------------------- тайминг
typedef enum {
    FC_TIMING_CONTROL = 0,  // imu_ref_callback: собственно контур управления
    FC_TIMING_MAIN,         // refloat_thd
    FC_TIMING_AUX,          // aux_thd
    FC_TIMING_COUNT
} FcTimingChannel;

typedef struct {
    const char *name;
    uint32_t nominal_period_us;
    uint64_t iterations;
    uint64_t sum_period_us;
    uint32_t min_period_us;
    uint32_t max_period_us;
    uint32_t late;  // период > nominal * (1 + FC_TIMING_LATE_TOLERANCE)
    uint64_t window_start_us;
} FcTimingStats;

void fc_timing_reset(void);
/** Отметить итерацию канала. Вызывается из RT-пути: без аллокаций и печати. */
void fc_timing_tick(FcTimingChannel ch);
void fc_timing_set_nominal(FcTimingChannel ch, uint32_t period_us);
FcTimingStats fc_timing_get(FcTimingChannel ch);

// ------------------------------------------------------------- mock IMU (§8)
/**
 * Запустить mock-IMU: задача на ядре 1, которая с частотой fc_imu_rate_hz()
 * вызывает callback, зарегистрированный Refloat через imu_set_read_callback.
 * Физического датчика нет — отдаётся строго покоящаяся ориентация.
 */
void fc_imu_mock_start(void);
void fc_imu_mock_stop(void);
int fc_imu_rate_hz(void);
bool fc_imu_startup_done(void);
void fc_imu_get_state(float *roll, float *pitch, float *yaw, float accel[3], float gyro[3],
                      float quat[4]);
void fc_imu_set_callback(void (*cb)(float *acc, float *gyro, float *mag, float dt));

// -------------------------------------------------------------- ADC/footpad (§9)
/** Напряжение на «пине» footpad. Возвращает FC_ADC_SAFE_VOLTAGE. */
float fc_adc_read(int vesc_pin);
float fc_adc_safe_voltage(void);

// ------------------------------------------------------------ хранилище (§12)
bool fc_storage_init(void);
bool fc_storage_read(uint32_t *value, int address);
bool fc_storage_write(uint32_t value, int address);
bool fc_storage_commit(void);
/** Число слов, доступных Refloat (эмуляция eeprom_var). */
int fc_storage_capacity(void);

// ------------------------------------------------------------- мотор (§6)
typedef enum {
    FC_MOTOR_CMD_CURRENT = 0,
    FC_MOTOR_CMD_BRAKE,
    FC_MOTOR_CMD_DUTY,
    FC_MOTOR_CMD_RPM,
    FC_MOTOR_CMD_COUNT
} FcMotorCmdKind;

typedef struct {
    uint64_t blocked[FC_MOTOR_CMD_COUNT];
    float last_value[FC_MOTOR_CMD_COUNT];
    uint64_t keepalive_calls;
    uint64_t total_blocked;
} FcMotorStats;

FcMotorStats fc_motor_stats(void);
const char *fc_motor_backend_name(void);
const char *fc_can_backend_name(void);

// ------------------------------------------------------------------ VESC_IF
/** Собрать структуру vesc_c_if и опубликовать её в floatcore_vesc_if. */
void fc_vesc_if_init(void);
void fc_vesc_if_deinit(void);
/** Слот lib_info.arg; вызывается refloat_facade перед refloat_init(). */
void floatcore_set_arg_slot(void **slot);

// ------------------------------------------------------------------- прочее
void fc_console_start(void);
uint64_t fc_uptime_us(void);

// ------------------------------------------- интроспекция для консоли (§15)
size_t fc_thread_count(void);
const char *fc_thread_name(size_t i);
uint32_t fc_thread_stack_watermark(size_t i);
uint32_t fc_imu_stack_watermark(void);
/** Диагностика watchdog: контур не отмечается в TWDT указанное число мс. */
void fc_imu_inject_stall(int ms);
bool fc_config_registered(void);
uint32_t fc_boot_count(void);
const char *fc_reset_reason_name(void);
int fc_config_read(uint8_t *data, bool is_default);
