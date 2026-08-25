// Реализация mock-платформы VESC для host-тестов.
//
// Содержит:
//   * детерминированный виртуальный планировщик (см. docs/threading_model.md);
//   * реализацию всех VESC_IF-функций, которые вызывает Refloat;
//   * перехват выходов на мотор в кольцевой буфер (CAN не трогается).

#include "mock_vesc_if.h"
#include "vesc_c_if.h"
#include "../../../compat/config/floatcore_limits.h"
#include "../../../compat/motor/logical_motor.h"

#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ---------------------------------------------------------------- глобальное состояние

#define MAX_THREADS 4
#define MAX_CMDS 4096
#define EEPROM_WORDS 128

typedef struct {
    pthread_t tid;
    void (*fun)(void *);
    void *arg;
    uint64_t wake_at_us;
    bool sleeping;
    bool started;
    bool terminate;
    bool finished;
    int priority;
} MockThread;

static struct {
    pthread_mutex_t lock;
    pthread_cond_t cv_threads;  // будим задачи
    pthread_cond_t cv_test;     // будим тестовый поток

    uint64_t now_us;

    MockThread threads[MAX_THREADS];
    size_t thread_count;

    // IMU
    void (*imu_cb)(float *acc, float *gyro, float *mag, float dt);
    float accel[3], gyro[3];
    float roll_rad, pitch_rad, yaw_rad;
    float quat[4];
    bool imu_startup_done;
    bool imu_stalled;

    // ADC
    float adc[2];

    // телеметрия
    MockMotorTelemetry tele;

    // захват команд
    MockMotorCmd cmds[MAX_CMDS];
    size_t cmd_count;
    size_t nan_current_requests;

    // EEPROM
    uint32_t eeprom[EEPROM_WORDS];
    bool eeprom_failing;

    // конфигурация VESC
    float cfg_float[64];
    int cfg_int[64];

    // прочее
    void (*app_data_handler)(unsigned char *data, unsigned int len);
    void (*app_data_sink)(void *ctx, const uint8_t *data, unsigned int len);
    void *app_data_sink_ctx;

    // конфигурация, зарегистрированная Refloat через conf_custom_add_config
    int (*cfg_get)(uint8_t *data, bool is_default);
    bool (*cfg_set)(uint8_t *data);
    int (*cfg_get_xml)(uint8_t **data);

    char eeprom_autosave[512];
    bool cfg_from_limits;
    void (*log_sink)(const char *fmt, va_list ap);
    bool io_pin_state[16];
} M;

static __thread MockThread *current_thread = NULL;

vesc_c_if *mock_vesc_if = NULL;
static vesc_c_if IF;
static void *refloat_arg = NULL;
static void **arg_slot = &refloat_arg;
volatile int prog_ptr_host = 0;

// ------------------------------------------------------------------- планировщик

/**
 * Задача считается «припаркованной», только если она спит И её срок пробуждения
 * ещё не наступил. Без второго условия тест возобновлялся бы раньше, чем задача,
 * которой уже пора проснуться, успела отработать — источник недетерминированности.
 */
static bool all_parked_locked(void) {
    for (size_t i = 0; i < M.thread_count; ++i) {
        MockThread *t = &M.threads[i];
        if (t->finished) {
            continue;
        }
        if (!t->started) {
            return false;
        }
        if (!t->sleeping || t->wake_at_us <= M.now_us) {
            return false;
        }
    }
    return true;
}

static void *thread_trampoline(void *arg) {
    MockThread *t = (MockThread *) arg;
    current_thread = t;

    pthread_mutex_lock(&M.lock);
    t->started = true;
    pthread_mutex_unlock(&M.lock);

    t->fun(t->arg);

    pthread_mutex_lock(&M.lock);
    t->finished = true;
    t->sleeping = true;
    pthread_cond_broadcast(&M.cv_test);
    pthread_mutex_unlock(&M.lock);
    return NULL;
}

void mock_wait_idle(void) {
    pthread_mutex_lock(&M.lock);
    while (!all_parked_locked()) {
        pthread_cond_wait(&M.cv_test, &M.lock);
    }
    pthread_mutex_unlock(&M.lock);
}

void mock_advance_us(uint64_t us) {
    pthread_mutex_lock(&M.lock);
    M.now_us += us;
    pthread_cond_broadcast(&M.cv_threads);
    while (!all_parked_locked()) {
        pthread_cond_wait(&M.cv_test, &M.lock);
    }
    pthread_mutex_unlock(&M.lock);
}

uint64_t mock_now_us(void) {
    pthread_mutex_lock(&M.lock);
    uint64_t t = M.now_us;
    pthread_mutex_unlock(&M.lock);
    return t;
}

// ------------------------------------------------------------- реализация VESC_IF

static void if_sleep_us(uint32_t us) {
    MockThread *t = current_thread;
    if (!t) {
        return;  // вызов не из задачи Refloat
    }
    pthread_mutex_lock(&M.lock);
    t->wake_at_us = M.now_us + us;
    t->sleeping = true;
    pthread_cond_broadcast(&M.cv_test);
    while (M.now_us < t->wake_at_us && !t->terminate) {
        pthread_cond_wait(&M.cv_threads, &M.lock);
    }
    t->sleeping = false;
    pthread_mutex_unlock(&M.lock);
}

static void if_sleep_ms(uint32_t ms) {
    if_sleep_us(ms * 1000);
}

static float if_system_time(void) {
    return (float) (mock_now_us() * 1e-6);
}

static systime_t if_system_time_ticks(void) {
    return (systime_t) (mock_now_us() / 100);  // SYSTEM_TICK_RATE_HZ = 10000
}

static uint32_t if_timer_time_now(void) {
    return (uint32_t) mock_now_us();
}

static float if_timer_seconds_elapsed_since(uint32_t t) {
    return (float) ((uint32_t) mock_now_us() - t) * 1e-6f;
}

static float if_ts_to_age_s(systime_t ts) {
    return (float) (if_system_time_ticks() - ts) / 10000.0f;
}

static lib_thread if_spawn(void (*fun)(void *), size_t stack, const char *name, void *arg) {
    (void) stack;
    (void) name;
    pthread_mutex_lock(&M.lock);
    if (M.thread_count >= MAX_THREADS) {
        pthread_mutex_unlock(&M.lock);
        return NULL;
    }
    MockThread *t = &M.threads[M.thread_count++];
    memset(t, 0, sizeof(*t));
    t->fun = fun;
    t->arg = arg;
    pthread_mutex_unlock(&M.lock);

    pthread_create(&t->tid, NULL, thread_trampoline, t);

    // Дождаться, пока задача дойдёт до первого сна: иначе она стартовала бы
    // параллельно с тестовым потоком.
    pthread_mutex_lock(&M.lock);
    while (!t->finished && !(t->started && t->sleeping && t->wake_at_us > M.now_us)) {
        pthread_cond_wait(&M.cv_test, &M.lock);
    }
    pthread_mutex_unlock(&M.lock);

    return (lib_thread) t;
}

static void if_request_terminate(lib_thread th) {
    MockThread *t = (MockThread *) th;
    if (!t) {
        return;
    }
    pthread_mutex_lock(&M.lock);
    t->terminate = true;
    pthread_cond_broadcast(&M.cv_threads);
    pthread_mutex_unlock(&M.lock);
    pthread_join(t->tid, NULL);
}

static bool if_should_terminate(void) {
    MockThread *t = current_thread;
    return t ? t->terminate : false;
}

static void if_thread_set_priority(int prio) {
    if (current_thread) {
        current_thread->priority = prio;
    }
}

static void **if_get_arg(uint32_t prog_addr) {
    (void) prog_addr;
    return arg_slot;
}

void mock_set_arg_slot(void **slot) {
    arg_slot = slot ? slot : &refloat_arg;
}

static int if_printf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    if (M.log_sink) {
        M.log_sink(fmt, ap);
    }
    va_end(ap);
    return 0;
}

static void *if_malloc(size_t n) {
    return malloc(n);
}

static void if_free(void *p) {
    free(p);
}

// ------------------------------------------------------------------------ IMU

static bool if_imu_startup_done(void) {
    return M.imu_startup_done;
}

static float if_imu_get_roll(void) {
    return M.roll_rad;
}

static float if_imu_get_pitch(void) {
    return M.pitch_rad;
}

static float if_imu_get_yaw(void) {
    return M.yaw_rad;
}

static void if_imu_get_gyro(float *g) {
    memcpy(g, M.gyro, sizeof(M.gyro));
}

static void if_imu_get_accel(float *a) {
    memcpy(a, M.accel, sizeof(M.accel));
}

static void if_imu_get_quaternions(float *q) {
    memcpy(q, M.quat, sizeof(M.quat));
}

static void if_imu_set_read_callback(void (*cb)(float *, float *, float *, float)) {
    M.imu_cb = cb;
}

// ------------------------------------------------------------------------ IO

static float if_io_read_analog(VESC_PIN pin) {
    if (pin == VESC_PIN_ADC1) {
        return M.adc[0];
    }
    if (pin == VESC_PIN_ADC2) {
        return M.adc[1];
    }
    return -1.0f;
}

static bool if_io_set_mode(VESC_PIN pin, VESC_PIN_MODE mode) {
    (void) pin;
    (void) mode;
    return true;
}

static bool if_io_write(VESC_PIN pin, int state) {
    if ((size_t) pin < 16) {
        M.io_pin_state[pin] = state != 0;
    }
    return true;
}

static bool if_io_read(VESC_PIN pin) {
    return (size_t) pin < 16 ? M.io_pin_state[pin] : false;
}

// ------------------------------------------------------- выход на мотор (перехват)

static void record(MockMotorCmdKind kind, float value) {
    pthread_mutex_lock(&M.lock);
    if (M.cmd_count < MAX_CMDS) {
        M.cmds[M.cmd_count++] = (MockMotorCmd){.t_us = M.now_us, .kind = kind, .value = value};
    }
    pthread_mutex_unlock(&M.lock);
}

static void if_mc_set_current(float current) {
    if (isnan(current) || isinf(current)) {
        M.nan_current_requests++;
    }
    record(MOCK_CMD_CURRENT, current);
    logical_motor_request_current(current);
}

static void if_mc_set_brake_current(float current) {
    record(MOCK_CMD_BRAKE_CURRENT, current);
    logical_motor_request_brake_current(current);
}

static void if_mc_set_duty(float duty) {
    record(MOCK_CMD_DUTY, duty);
    logical_motor_request_duty(duty);
}

static void if_mc_set_current_off_delay(float d) {
    record(MOCK_CMD_CURRENT_OFF_DELAY, d);
    logical_motor_set_current_off_delay(d);
}

static void if_timeout_reset(void) {
    record(MOCK_CMD_TIMEOUT_RESET, 0.0f);
    logical_motor_keepalive();
}

// ------------------------------------------------------------------ телеметрия

static float if_mc_get_rpm(void) {
    return M.tele.erpm;
}

static float if_mc_get_duty_cycle_now(void) {
    return M.tele.duty;
}

static float if_mc_get_tot_current(void) {
    return M.tele.motor_current;
}

static float if_mc_get_tot_current_filtered(void) {
    return M.tele.motor_current;
}

static float if_mc_get_tot_current_directional(void) {
    return M.tele.motor_current_directional;
}

static float if_mc_get_tot_current_directional_filtered(void) {
    return M.tele.motor_current_directional;
}

static float if_mc_get_tot_current_in(void) {
    return M.tele.input_current;
}

static float if_mc_get_tot_current_in_filtered(void) {
    return M.tele.input_current;
}

static float if_mc_get_input_voltage_filtered(void) {
    return M.tele.input_voltage;
}

static float if_mc_temp_fet_filtered(void) {
    return M.tele.fet_temp;
}

static float if_mc_temp_motor_filtered(void) {
    return M.tele.motor_temp;
}

static float if_mc_get_speed(void) {
    return M.tele.speed_ms;
}

static float if_mc_get_distance(void) {
    return M.tele.distance_m;
}

static float if_mc_get_distance_abs(void) {
    return fabsf(M.tele.distance_m);
}

static uint64_t if_mc_get_odometer(void) {
    return (uint64_t) fabsf(M.tele.distance_m);
}

static mc_fault_code if_mc_get_fault(void) {
    return (mc_fault_code) M.tele.fault_code;
}

static const char *if_mc_fault_to_string(mc_fault_code f) {
    (void) f;
    return "MOCK_FAULT";
}

static float if_mc_get_battery_level(float *wh_left) {
    if (wh_left) {
        *wh_left = 0.5f;
    }
    return 0.5f;
}

static float if_zero_bool(bool reset) {
    (void) reset;
    return 0.0f;
}

static float if_zero(void) {
    return 0.0f;
}

static gnss_data gnss_storage;

static volatile gnss_data *if_mc_gnss(void) {
    return &gnss_storage;
}

// ------------------------------------------------------------------ конфигурация

static float if_get_cfg_float(CFG_PARAM p) {
    if (M.cfg_from_limits) {
        switch (p) {
        case CFG_PARAM_l_current_max:
            return fc_effective_current_max();
        case CFG_PARAM_l_current_min:
            return fc_effective_current_min();
        case CFG_PARAM_l_in_current_max:
            return fc_effective_in_current_max();
        case CFG_PARAM_l_in_current_min:
            return fc_effective_in_current_min();
        case CFG_PARAM_l_temp_fet_start:
            return fc_effective_temp_fet_start();
        case CFG_PARAM_l_temp_fet_end:
            return fc_effective_temp_fet_end();
        case CFG_PARAM_l_temp_motor_start:
            return fc_effective_temp_motor_start();
        case CFG_PARAM_l_temp_motor_end:
            return fc_effective_temp_motor_end();
        case CFG_PARAM_l_max_duty:
            return fc_effective_max_duty();
        default:
            break;
        }
    }
    return ((size_t) p < 64) ? M.cfg_float[p] : 0.0f;
}

static int if_get_cfg_int(CFG_PARAM p) {
    if (M.cfg_from_limits && p == CFG_PARAM_si_battery_cells) {
        return fc_battery_cell_count();
    }
    return ((size_t) p < 64) ? M.cfg_int[p] : 0;
}

static bool if_set_cfg_float(CFG_PARAM p, float v) {
    if ((size_t) p < 64) {
        M.cfg_float[p] = v;
    }
    return true;
}

static bool if_set_cfg_int(CFG_PARAM p, int v) {
    if ((size_t) p < 64) {
        M.cfg_int[p] = v;
    }
    return true;
}

static void if_conf_custom_add_config(
    int (*g)(uint8_t *, bool), bool (*s)(uint8_t *), int (*x)(uint8_t **)
) {
    M.cfg_get = g;
    M.cfg_set = s;
    M.cfg_get_xml = x;
}

static void if_conf_custom_clear_configs(void) {
    M.cfg_get = NULL;
    M.cfg_set = NULL;
    M.cfg_get_xml = NULL;
}

// ------------------------------------------------------------------- хранилище

static bool if_read_eeprom_var(eeprom_var *v, int address) {
    if (M.eeprom_failing || address < 0 || address >= EEPROM_WORDS) {
        return false;
    }
    v->as_u32 = M.eeprom[address];
    return true;
}

static bool if_store_eeprom_var(eeprom_var *v, int address) {
    if (M.eeprom_failing || address < 0 || address >= EEPROM_WORDS) {
        return false;
    }
    M.eeprom[address] = v->as_u32;
    if (M.eeprom_autosave[0]) {
        mock_eeprom_save_file(M.eeprom_autosave);
    }
    return true;
}

static bool if_store_backup_data(void) {
    return true;
}

// ------------------------------------------------------------------------ comm

static void if_send_app_data(unsigned char *data, unsigned int len) {
    if (M.app_data_sink) {
        M.app_data_sink(M.app_data_sink_ctx, data, len);
    }
}

static bool if_set_app_data_handler(void (*f)(unsigned char *, unsigned int)) {
    M.app_data_handler = f;
    return true;
}

static bool if_app_is_output_disabled(void) {
    return false;
}

// --------------------------------------------------------------------- пульт

static remote_state if_get_remote_state(void) {
    remote_state s;
    memset(&s, 0, sizeof(s));
    s.age_s = 1000.0f;
    return s;
}

static float if_get_ppm(void) {
    return 0.0f;
}

static float if_get_ppm_age(void) {
    return 1000.0f;
}

// ----------------------------------------------------------------------- LBM

static bool if_lbm_add_extension(char *name, extension_fptr f) {
    (void) name;
    (void) f;
    return true;
}

static float if_lbm_dec_as_float(lbm_value v) {
    float f;
    memcpy(&f, &v, sizeof(f));
    return f;
}

static int32_t if_lbm_dec_as_i32(lbm_value v) {
    return (int32_t) v;
}

// -------------------------------------------------------------------- заглушки

static void if_plot_init(const char *x, const char *y) {
    (void) x;
    (void) y;
}

static void if_plot_add_graph(const char *n) {
    (void) n;
}

static void if_plot_set_graph(int g) {
    (void) g;
}

static void if_plot_send_points(float x, float y) {
    (void) x;
    (void) y;
}

static void if_set_pad_mode(void *gpio, uint32_t pin, uint32_t mode) {
    (void) gpio;
    (void) pin;
    (void) mode;
}

// -------------------------------------------------------------- инициализация

void mock_init(void) {
    memset(&M, 0, sizeof(M));
    pthread_mutex_init(&M.lock, NULL);
    pthread_cond_init(&M.cv_threads, NULL);
    pthread_cond_init(&M.cv_test, NULL);

    memset(&IF, 0, sizeof(IF));

    IF.sleep_us = if_sleep_us;
    IF.sleep_ms = if_sleep_ms;
    IF.system_time = if_system_time;
    IF.system_time_ticks = if_system_time_ticks;
    IF.ts_to_age_s = if_ts_to_age_s;
    IF.timer_time_now = if_timer_time_now;
    IF.timer_seconds_elapsed_since = if_timer_seconds_elapsed_since;
    IF.printf = if_printf;
    IF.malloc = if_malloc;
    IF.free = if_free;
    IF.spawn = if_spawn;
    IF.request_terminate = if_request_terminate;
    IF.should_terminate = if_should_terminate;
    IF.thread_set_priority = if_thread_set_priority;
    IF.get_arg = if_get_arg;

    IF.imu_startup_done = if_imu_startup_done;
    IF.imu_get_roll = if_imu_get_roll;
    IF.imu_get_pitch = if_imu_get_pitch;
    IF.imu_get_yaw = if_imu_get_yaw;
    IF.imu_get_gyro = if_imu_get_gyro;
    IF.imu_get_accel = if_imu_get_accel;
    IF.imu_get_quaternions = if_imu_get_quaternions;
    IF.imu_set_read_callback = if_imu_set_read_callback;

    IF.io_read_analog = if_io_read_analog;
    IF.io_set_mode = if_io_set_mode;
    IF.io_write = if_io_write;
    IF.io_read = if_io_read;
    IF.set_pad_mode = if_set_pad_mode;

    IF.mc_set_current = if_mc_set_current;
    IF.mc_set_brake_current = if_mc_set_brake_current;
    IF.mc_set_duty = if_mc_set_duty;
    IF.mc_set_current_off_delay = if_mc_set_current_off_delay;
    IF.timeout_reset = if_timeout_reset;

    IF.mc_get_rpm = if_mc_get_rpm;
    IF.mc_get_duty_cycle_now = if_mc_get_duty_cycle_now;
    IF.mc_get_tot_current = if_mc_get_tot_current;
    IF.mc_get_tot_current_filtered = if_mc_get_tot_current_filtered;
    IF.mc_get_tot_current_directional = if_mc_get_tot_current_directional;
    IF.mc_get_tot_current_directional_filtered = if_mc_get_tot_current_directional_filtered;
    IF.mc_get_tot_current_in = if_mc_get_tot_current_in;
    IF.mc_get_tot_current_in_filtered = if_mc_get_tot_current_in_filtered;
    IF.mc_get_input_voltage_filtered = if_mc_get_input_voltage_filtered;
    IF.mc_temp_fet_filtered = if_mc_temp_fet_filtered;
    IF.mc_temp_motor_filtered = if_mc_temp_motor_filtered;
    IF.mc_get_speed = if_mc_get_speed;
    IF.mc_get_distance = if_mc_get_distance;
    IF.mc_get_distance_abs = if_mc_get_distance_abs;
    IF.mc_get_odometer = if_mc_get_odometer;
    IF.mc_get_fault = if_mc_get_fault;
    IF.mc_fault_to_string = if_mc_fault_to_string;
    IF.mc_get_battery_level = if_mc_get_battery_level;
    IF.mc_get_amp_hours = if_zero_bool;
    IF.mc_get_amp_hours_charged = if_zero_bool;
    IF.mc_get_watt_hours = if_zero_bool;
    IF.mc_get_watt_hours_charged = if_zero_bool;
    IF.mc_gnss = if_mc_gnss;
    IF.foc_get_id = if_zero;
    IF.foc_get_iq = if_zero;

    IF.get_cfg_float = if_get_cfg_float;
    IF.get_cfg_int = if_get_cfg_int;
    IF.set_cfg_float = if_set_cfg_float;
    IF.set_cfg_int = if_set_cfg_int;
    IF.conf_custom_add_config = if_conf_custom_add_config;
    IF.conf_custom_clear_configs = if_conf_custom_clear_configs;

    IF.read_eeprom_var = if_read_eeprom_var;
    IF.store_eeprom_var = if_store_eeprom_var;
    IF.store_backup_data = if_store_backup_data;

    IF.send_app_data = if_send_app_data;
    IF.set_app_data_handler = if_set_app_data_handler;
    IF.app_is_output_disabled = if_app_is_output_disabled;

    IF.get_remote_state = if_get_remote_state;
    IF.get_ppm = if_get_ppm;
    IF.get_ppm_age = if_get_ppm_age;

    IF.lbm_add_extension = if_lbm_add_extension;
    IF.lbm_dec_as_float = if_lbm_dec_as_float;
    IF.lbm_dec_as_i32 = if_lbm_dec_as_i32;
    IF.lbm_enc_sym_nil = 0;
    IF.lbm_enc_sym_true = 1;

    IF.plot_init = if_plot_init;
    IF.plot_add_graph = if_plot_add_graph;
    IF.plot_set_graph = if_plot_set_graph;
    IF.plot_send_points = if_plot_send_points;

    // Осознанно НЕ реализованы (Refloat проверяет указатель перед вызовом):
    //   foc_play_tone — haptic через FOC, по CAN недоступен

    mock_vesc_if = &IF;
    arg_slot = &refloat_arg;
    refloat_arg = NULL;

    // Разумные значения конфигурации VESC по умолчанию (аналог настроек FSESC)
    M.cfg_float[CFG_PARAM_l_current_max] = 60.0f;
    M.cfg_float[CFG_PARAM_l_current_min] = -60.0f;
    M.cfg_float[CFG_PARAM_l_in_current_max] = 50.0f;
    M.cfg_float[CFG_PARAM_l_in_current_min] = -20.0f;
    M.cfg_float[CFG_PARAM_l_temp_fet_start] = 85.0f;
    M.cfg_float[CFG_PARAM_l_temp_motor_start] = 85.0f;
    M.cfg_float[CFG_PARAM_l_max_duty] = 0.95f;
    M.cfg_float[CFG_PARAM_foc_motor_flux_linkage] = 0.0045f;
    M.cfg_float[CFG_PARAM_IMU_mahony_kp] = 0.2f;
    M.cfg_int[CFG_PARAM_si_motor_poles] = 30;
    M.cfg_int[CFG_PARAM_si_battery_cells] = 20;
    M.cfg_int[CFG_PARAM_IMU_sample_rate] = 500;

    M.quat[0] = 1.0f;
    M.accel[2] = 1.0f;
    M.tele.input_voltage = 75.0f;
    M.tele.fet_temp = 30.0f;
    M.tele.motor_temp = 30.0f;
    M.imu_startup_done = true;

    for (size_t i = 0; i < EEPROM_WORDS; ++i) {
        M.eeprom[i] = 0xFFFFFFFF;
    }
}

void mock_deinit(void) {
    for (size_t i = 0; i < M.thread_count; ++i) {
        MockThread *t = &M.threads[i];
        if (!t->finished) {
            if_request_terminate((lib_thread) t);
        }
    }
    M.thread_count = 0;
    mock_vesc_if = NULL;
}

void mock_set_log_sink(void (*sink)(const char *, va_list)) {
    M.log_sink = sink;
}

// -------------------------------------------------------------- управление IMU

void mock_imu_set_raw(const float accel[3], const float gyro[3]) {
    memcpy(M.accel, accel, sizeof(M.accel));
    memcpy(M.gyro, gyro, sizeof(M.gyro));
}

void mock_imu_set_angles_deg(float roll, float pitch, float yaw) {
    const float d2r = 3.14159265358979f / 180.0f;
    M.roll_rad = roll * d2r;
    M.pitch_rad = pitch * d2r;
    M.yaw_rad = yaw * d2r;
}

void mock_imu_set_startup_done(bool done) {
    M.imu_startup_done = done;
}

void mock_imu_set_stalled(bool stalled) {
    M.imu_stalled = stalled;
}

bool mock_imu_is_stalled(void) {
    return M.imu_stalled;
}

void mock_imu_tick(float dt) {
    if (M.imu_stalled || !M.imu_cb) {
        return;
    }
    float acc[3], gyr[3], mag[3] = {0};
    memcpy(acc, M.accel, sizeof(acc));
    memcpy(gyr, M.gyro, sizeof(gyr));
    M.imu_cb(acc, gyr, mag, dt);
}

// ------------------------------------------------------------------ ADC/телеметрия

void mock_adc_set(float a1, float a2) {
    M.adc[0] = a1;
    M.adc[1] = a2;
}

void mock_motor_set_telemetry(const MockMotorTelemetry *t) {
    M.tele = *t;
}

// ------------------------------------------------------------------ захват команд

void mock_motor_cmd_clear(void) {
    pthread_mutex_lock(&M.lock);
    M.cmd_count = 0;
    pthread_mutex_unlock(&M.lock);
}

size_t mock_motor_cmd_count(void) {
    return M.cmd_count;
}

MockMotorCmd mock_motor_cmd_at(size_t i) {
    return i < M.cmd_count ? M.cmds[i] : (MockMotorCmd){0};
}

MockMotorCmd mock_motor_cmd_last(MockMotorCmdKind kind, bool *found) {
    if (found) {
        *found = false;
    }
    MockMotorCmd res = {0};
    for (size_t i = M.cmd_count; i > 0; --i) {
        if (M.cmds[i - 1].kind == kind) {
            if (found) {
                *found = true;
            }
            return M.cmds[i - 1];
        }
    }
    return res;
}

size_t mock_motor_cmd_count_of(MockMotorCmdKind kind) {
    size_t n = 0;
    for (size_t i = 0; i < M.cmd_count; ++i) {
        if (M.cmds[i].kind == kind) {
            ++n;
        }
    }
    return n;
}

// -------------------------------------------------------------------- хранилище

void mock_eeprom_erase(void) {
    for (size_t i = 0; i < EEPROM_WORDS; ++i) {
        M.eeprom[i] = 0xFFFFFFFF;
    }
}

void mock_eeprom_set_failing(bool failing) {
    M.eeprom_failing = failing;
}

bool mock_has_app_data_handler(void) {
    return M.app_data_handler != NULL;
}

void mock_app_data_to_firmware(const uint8_t *data, unsigned int len) {
    if (!M.app_data_handler) {
        return;
    }
    // Обработчик Refloat принимает неконстантный указатель; копируем, чтобы
    // случайная запись не задела буфер вызывающего.
    static uint8_t buf[600];
    if (len > sizeof(buf)) {
        return;
    }
    memcpy(buf, data, len);
    M.app_data_handler(buf, len);
}

void mock_set_app_data_sink(
    void (*sink)(void *ctx, const uint8_t *data, unsigned int len), void *ctx
) {
    M.app_data_sink = sink;
    M.app_data_sink_ctx = ctx;
}

bool mock_has_custom_config(void) {
    return M.cfg_get != NULL && M.cfg_get_xml != NULL;
}

int mock_custom_config_get(uint8_t *buf, size_t cap, bool is_default) {
    if (!M.cfg_get) {
        return -1;
    }
    // Refloat пишет ровно столько, сколько занимает сериализованная
    // конфигурация (282 байта для v1.3.0). Ориентир — SERIALIZED_CONFIG_LENGTH
    // из main.c, то есть 320 байт.
    if (cap < 320) {
        return -1;
    }
    return M.cfg_get(buf, is_default);
}

bool mock_custom_config_set(const uint8_t *buf) {
    if (!M.cfg_set) {
        return false;
    }
    return M.cfg_set((uint8_t *) buf);
}

int mock_custom_config_get_xml(const uint8_t **data) {
    if (!M.cfg_get_xml) {
        return -1;
    }
    uint8_t *p = NULL;
    int len = M.cfg_get_xml(&p);
    *data = p;
    return len;
}

bool mock_eeprom_load_file(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) {
        return false;
    }
    size_t n = fread(M.eeprom, sizeof(uint32_t), EEPROM_WORDS, f);
    fclose(f);
    return n > 0;
}

bool mock_eeprom_save_file(const char *path) {
    FILE *f = fopen(path, "wb");
    if (!f) {
        return false;
    }
    fwrite(M.eeprom, sizeof(uint32_t), EEPROM_WORDS, f);
    fclose(f);
    return true;
}

void mock_eeprom_set_autosave(const char *path) {
    if (!path) {
        M.eeprom_autosave[0] = 0;
        return;
    }
    snprintf(M.eeprom_autosave, sizeof(M.eeprom_autosave), "%s", path);
}

void mock_cfg_use_floatcore_limits(bool enable) {
    M.cfg_from_limits = enable;
}

void mock_cfg_set_float(int p, float v) {
    if ((size_t) p < 64) {
        M.cfg_float[p] = v;
    }
}

void mock_cfg_set_int(int p, int v) {
    if ((size_t) p < 64) {
        M.cfg_int[p] = v;
    }
}

size_t mock_stats_nan_current_requests(void) {
    return M.nan_current_requests;
}
