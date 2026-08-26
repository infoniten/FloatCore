// Управление mock-платформой из тестов.
//
// Этот заголовок НЕ включает заголовки Refloat, поэтому его можно свободно
// использовать вместе с системными заголовками (см. заметку про time_t в
// docs/threading_model.md).
#pragma once

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// ---------------------------------------------------------------- жизненный цикл

void mock_init(void);
void mock_deinit(void);

/**
 * Указать ячейку, которую вернёт VESC_IF->get_arg().
 *
 * На VESC загрузчик хранит `lib_info.arg` у себя, и макрос ARG читает именно её.
 * Refloat присваивает `info->arg = d` в самом начале init() и сразу пользуется ARG,
 * поэтому mock обязан отдавать адрес того же поля.
 */
void floatcore_set_arg_slot(void **slot);

/** Логирование VESC_IF->printf перенаправляется сюда. */
void mock_set_log_sink(void (*sink)(const char *fmt, va_list ap));

// ------------------------------------------------------- детерминированное время

uint64_t mock_now_us(void);

/**
 * Продвинуть виртуальные часы на `us` и дать отработать всем задачам Refloat,
 * которые должны проснуться. Возврат — когда все задачи снова спят.
 */
void mock_advance_us(uint64_t us);

/** Дождаться, пока все запущенные задачи не окажутся в состоянии сна. */
void mock_wait_idle(void);

// ---------------------------------------------------------------------- IMU

/** Сырые данные, которые получит imu_ref_callback. */
void mock_imu_set_raw(const float accel[3], const float gyro[3]);

/** Углы «AHRS прошивки» (imu_get_roll/pitch/yaw), в градусах. */
void mock_imu_set_angles_deg(float roll, float pitch, float yaw);

void mock_imu_set_startup_done(bool done);

/**
 * Сгенерировать один семпл IMU: вызывает зарегистрированный callback Refloat.
 * Вызывается только когда задачи спят — воспроизводит модель «одно ядро,
 * вытеснение без параллелизма».
 */
void mock_imu_tick(float dt);

/** Прекратить генерацию семплов (сценарий IMU timeout). */
void mock_imu_set_stalled(bool stalled);
bool mock_imu_is_stalled(void);

// -------------------------------------------------------------------- футпады

void mock_adc_set(float adc1_volts, float adc2_volts);

/** Текущие напряжения на выводах ADC1/ADC2 — то же, что видит Refloat. */
void mock_adc_get(float *adc1_volts, float *adc2_volts);

// ------------------------------------------------------------------ телеметрия

typedef struct {
    float erpm;
    float duty;
    float motor_current;
    float motor_current_directional;
    float input_current;
    float input_voltage;
    float fet_temp;
    float motor_temp;
    float speed_ms;
    float distance_m;
    int fault_code;
} MockMotorTelemetry;

void mock_motor_set_telemetry(const MockMotorTelemetry *t);

// ------------------------------------------------- захват выходов на мотор

typedef enum {
    MOCK_CMD_CURRENT,
    MOCK_CMD_BRAKE_CURRENT,
    MOCK_CMD_DUTY,
    MOCK_CMD_CURRENT_OFF_DELAY,
    MOCK_CMD_TIMEOUT_RESET,
} MockMotorCmdKind;

typedef struct {
    uint64_t t_us;
    MockMotorCmdKind kind;
    float value;
} MockMotorCmd;

void mock_motor_cmd_clear(void);
size_t mock_motor_cmd_count(void);
MockMotorCmd mock_motor_cmd_at(size_t i);
/** Последняя команда заданного вида; found=false, если такой не было. */
MockMotorCmd mock_motor_cmd_last(MockMotorCmdKind kind, bool *found);
/** Число команд заданного вида. */
size_t mock_motor_cmd_count_of(MockMotorCmdKind kind);

// ------------------------------------------------------------------ хранилище

/** Сбросить эмулируемый EEPROM (все слова = 0xFFFFFFFF, «стёрт»). */
void mock_eeprom_erase(void);
/** Смоделировать отказ EEPROM. */
void mock_eeprom_set_failing(bool failing);

// ------------------------------------------- мост к VESC Tool (COMM_CUSTOM_APP_DATA)

/** Зарегистрирован ли обработчик Refloat (set_app_data_handler). */
bool mock_has_app_data_handler(void);

/** Передать данные в Refloat (эквивалент COMM_CUSTOM_APP_DATA от VESC Tool). */
void mock_app_data_to_firmware(const uint8_t *data, unsigned int len);

/** Куда уходят данные, которые Refloat отправляет через send_app_data(). */
void mock_set_app_data_sink(void (*sink)(void *ctx, const uint8_t *data, unsigned int len),
                            void *ctx);

// ------------------------------------- мост конфигурации (conf_custom_add_config)

/** Зарегистрировал ли Refloat свою конфигурацию. */
bool mock_has_custom_config(void);

/** Текущая или дефолтная конфигурация Refloat. Возвращает длину или -1. */
int mock_custom_config_get(uint8_t *buf, size_t cap, bool is_default);

/** Применить конфигурацию (Refloat сам запишет её в EEPROM). */
bool mock_custom_config_set(const uint8_t *buf);

/** Сжатый XML описания параметров. Возвращает длину. */
int mock_custom_config_get_xml(const uint8_t **data);

// ----------------------------------------------- постоянное хранилище на диске

/** Загрузить эмулируемый EEPROM из файла (если файл есть). */
bool mock_eeprom_load_file(const char *path);

/** Сохранить эмулируемый EEPROM в файл. */
bool mock_eeprom_save_file(const char *path);

/**
 * Автосохранение в файл после каждой успешной записи слова.
 * Благодаря этому конфигурация переживает перезапуск процесса.
 */
void mock_eeprom_set_autosave(const char *path);

// ---------------------------------------------------------- конфигурация VESC

void mock_cfg_set_float(int param, float value);
void mock_cfg_set_int(int param, int value);

/**
 * Брать значения CFG_PARAM_* не из внутренних массивов mock-а, а из
 * FloatCore Config (compat/config/floatcore_limits.h).
 *
 * Тогда Refloat и Virtual mcConfig читают одни и те же пределы — ровно то,
 * ради чего затевался Virtual mcConfig. На ESP32 так же будет устроена
 * настоящая реализация get_cfg_float()/get_cfg_int().
 */
void mock_cfg_use_floatcore_limits(bool enable);

// -------------------------------------------------------------- статистика

/** Число вызовов VESC_IF->mc_set_current с NaN (не должно быть никогда). */
size_t mock_stats_nan_current_requests(void);
