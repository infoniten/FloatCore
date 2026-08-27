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

#define FC_PRIO_IMU 14        // источник ритма контура: imu_ref_callback
#define FC_PRIO_IMU_HW 13     // чтение физического ICM-20948
#define FC_PRIO_REFLOAT 12    // refloat_thd, 500 Гц
#define FC_PRIO_SUPERVISOR 15 // выше контура: обязан отработать при перегрузке
#define FC_PRIO_CONSOLE 4     // read-only CLI, ядро 0
#define FC_PRIO_IMU_STRESS 5  // стресс-тест шины I2C, ядро 0 (см. fc_imu_stress.c)

// Refloat просит 1536 байт стека — этого мало для Xtensa (docs/vesc_if_contract.md §2).
#define FC_STACK_SCALE 8

// ------------------------------------------------------------------- тайминг
typedef enum {
    FC_TIMING_CONTROL = 0,  // imu_ref_callback: собственно контур управления
    FC_TIMING_MAIN,         // refloat_thd
    FC_TIMING_AUX,          // aux_thd
    FC_TIMING_IMU_READ,     // задача чтения физического ICM-20948
    FC_TIMING_COUNT
} FcTimingChannel;

// Гистограмма периодов. Ширина корзины — nominal/50, диапазон 0…4×nominal,
// плюс корзина переполнения. Такой шаг (40 мкс при номинале 2000) позволяет
// считать p99 с точностью, сравнимой с самим джиттером, и при этом стоит
// 4 канала × 201 × 4 Б ≈ 3.2 КБ ОЗУ.
#define FC_TIMING_BINS 200

typedef struct {
    const char *name;
    uint32_t nominal_period_us;
    uint64_t iterations;
    uint64_t sum_period_us;
    uint32_t min_period_us;
    uint32_t max_period_us;
    uint32_t late;    // период больше номинала более чем на 20 %
    uint32_t missed;  // период больше двух номиналов: итерация пропущена
    uint64_t window_start_us;

    // Перцентили периода, мкс. Считаются по гистограмме, поэтому дают
    // верхнюю границу корзины, а не точное значение.
    uint32_t p50_us;
    uint32_t p95_us;
    uint32_t p99_us;
    uint32_t p999_us;
    uint32_t overflow;  // сколько периодов вышло за диапазон гистограммы

    // Длительность самой итерации, отдельно от периода: это и есть ответ на
    // вопрос «джиттер планировщика или время исполнения».
    uint64_t exec_samples;
    uint64_t exec_sum_us;
    uint32_t exec_min_us;
    uint32_t exec_max_us;
    uint32_t exec_p99_us;
} FcTimingStats;

void fc_timing_reset(void);
/** Отметить итерацию канала. Вызывается из RT-пути: без аллокаций и печати. */
void fc_timing_tick(FcTimingChannel ch);
void fc_timing_set_nominal(FcTimingChannel ch, uint32_t period_us);
FcTimingStats fc_timing_get(FcTimingChannel ch);

/** Границы итерации: между ними меряется время исполнения, а не период. */
void fc_timing_exec_begin(FcTimingChannel ch);
void fc_timing_exec_end(FcTimingChannel ch);

/** Выгрузить гистограмму периодов: bins[i] соответствует [i*w, (i+1)*w). */
uint32_t fc_timing_histogram(FcTimingChannel ch, uint32_t *bins, uint32_t max_bins,
                             uint32_t *bin_width_us);

// ------------------------------------------------- фасад источника IMU (v0.6D)
//
// Единственный мост между VESC_IF и данными датчика. Ветвление по источнику
// (mock или реальный ICM-20948) собрано в fc_imu_source.c — см. fc_imu_source.h.
int fc_imu_rate_hz(void);
bool fc_imu_startup_done(void);
void fc_imu_get_state(float *roll, float *pitch, float *yaw, float accel[3], float gyro[3],
                      float quat[4]);
void fc_imu_set_callback(void (*cb)(float *acc, float *gyro, float *mag, float dt));

// ------------------------------------------------------ mock-источник (v0.5)
/**
 * Задача на ядре 1, которая с фиксированной частотой вызывает callback
 * Refloat и отдаёт строго покоящуюся ориентацию. Физического датчика не
 * касается. Используется host-тестами и как заведомо безопасный источник.
 */
void fc_imu_mock_start(void);
void fc_imu_mock_stop(void);
int fc_imu_mock_rate_hz(void);
bool fc_imu_mock_startup_done(void);
void fc_imu_mock_get_state(float *roll, float *pitch, float *yaw, float accel[3], float gyro[3],
                           float quat[4]);
void fc_imu_mock_set_callback(void (*cb)(float *acc, float *gyro, float *mag, float dt));
uint32_t fc_imu_mock_stack_watermark(void);
void fc_imu_mock_inject_stall(int ms);

// -------------------------------------------------------------- ADC/footpad (§9)
/** Напряжение на «пине» footpad. Возвращает FC_ADC_SAFE_VOLTAGE. */
float fc_adc_read(int vesc_pin);
/** Лабораторная имитация нажатых футпадов (см. fc_adc_safe.c). */
void fc_adc_simulate_footpads(bool on, float voltage);
bool fc_adc_simulation_active(void);
float fc_adc_safe_voltage(void);

// ------------------------------------------------------------ хранилище (§12)
typedef struct {
    uint64_t writes_accepted;
    uint64_t writes_rejected;   // политика супервизора не разрешила
    uint64_t commits_requested;
    uint64_t commits_done;
    uint64_t commits_failed;
    uint32_t last_commit_us;    // длительность последней операции с flash
    uint32_t max_commit_us;     // худшая наблюдённая длительность
    uint64_t last_commit_at_us;
    bool dirty;
} FcStorageStats;

bool fc_storage_init(void);
bool fc_storage_read(uint32_t *value, int address);
bool fc_storage_write(uint32_t value, int address);
/** Синхронный коммит. Разрешён только вне realtime-пути (загрузка, CLI). */
bool fc_storage_commit(void);
/** Асинхронная просьба закоммитить: выполнит задача хранилища (ТЗ §25). */
bool fc_storage_request_commit(void);
/** Учесть запись, отклонённую политикой супервизора. */
void fc_storage_note_rejected_write(void);
FcStorageStats fc_storage_stats(void);
uint32_t fc_storage_stack_watermark(void);
/** Число слов, доступных Refloat (эмуляция eeprom_var). */
int fc_storage_capacity(void);

// ------------------------------------------------------------- мотор (§11)
// Счётчики и политика живут в compat/safety/fc_motor_gate.h — здесь остаётся
// только описание отсутствующей шины.
const char *fc_can_backend_name(void);

// ------------------------------------------------------------------ VESC_IF
/** Собрать структуру vesc_c_if и опубликовать её в floatcore_vesc_if. */
void fc_vesc_if_init(void);
/** Параметры внутреннего AHRS в том виде, в каком их видит Refloat. */
float fc_cfg_imu_mahony_kp(void);
float fc_cfg_imu_accel_confidence_decay(void);
void fc_vesc_if_deinit(void);
/** Слот lib_info.arg; вызывается refloat_facade перед refloat_init(). */
void floatcore_set_arg_slot(void **slot);

// ------------------------------------------------------------------- прочее
void fc_console_start(void);
void fc_supervisor_task_start(void);
uint32_t fc_supervisor_stack_watermark(void);
void fc_print_safety_line(void);

// --------------------------- единая realtime-цепочка от датчика (v0.6D)
// Одна задача: опрос ICM-20948 -> валидация -> AHRS -> callback Refloat.
// Обоснование периода опроса и политики восстановления — fc_imu_rt.c.
bool fc_imu_rt_start(void);
void fc_imu_rt_stop(void);
bool fc_imu_rt_available(void);
bool fc_imu_rt_running(void);
bool fc_imu_rt_startup_done(void);
int fc_imu_rt_rate_hz(void);
void fc_imu_rt_set_callback(void (*cb)(float *acc, float *gyro, float *mag, float dt));
uint64_t fc_imu_rt_iterations(void);
uint32_t fc_imu_rt_max_read_us(void);
uint64_t fc_imu_rt_reinits(void);
uint32_t fc_imu_rt_stack_watermark(void);
void fc_imu_rt_inject_stall(int ms);

uint64_t fc_uptime_us(void);

// ------------------------------------------- стресс-тест шины I2C (v0.6C)
// Диагностика железа: читает ICM-20948 без пауз, замещая штатную задачу.
// К Refloat и к выходу на мотор отношения не имеет — см. fc_imu_stress.c.
// i2c_hz == 0 — оставить штатную частоту шины (400 кГц).
#define FC_STRESS_DEFAULT_RESET_THRESHOLD 3
// reset_threshold — сколько отказов подряд до сброса шины; 0 — не сбрасывать.
bool fc_imu_stress_start(uint32_t seconds, uint32_t i2c_hz, uint32_t reset_threshold);
void fc_imu_stress_stop(void);
bool fc_imu_stress_running(void);
void fc_imu_stress_print_log(void);

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

// ------------------------------------------- идентификация железа (v0.6 §2)
/** I2C на GPIO21/22: скан, WHO_AM_I и сырые семплы. Только диагностика. */
void fc_imu_probe(int stream_seconds);
int fc_config_read(uint8_t *data, bool is_default);
