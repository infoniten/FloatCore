// Реализация VESC_IF на ESP-IDF/FreeRTOS — платформенный backend FloatCore.
//
// Тот же контракт, что описан в docs/vesc_if_contract.md и реализован
// host-mock-ом; отличается только источник данных. Refloat не меняется и о
// платформе не знает: он видит ровно ту же структуру vesc_c_if.
//
// Ограничение трансляции: этот файл включает заголовки ESP-IDF, поэтому ему
// НЕЛЬЗЯ видеть внутренние заголовки Refloat (src/time.h переопределяет
// time_t). Включается только SDK-заголовок VESC через общий shim
// compat/vesc_api/vesc_c_if.h.

#include "fc_platform.h"

#include "../../../compat/safety/fc_motor_gate.h"
#include "../../../compat/safety/fc_supervisor.h"
#include "vesc_c_if.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "vesc_if";

#define FC_MAX_THREADS 4

typedef struct {
    TaskHandle_t handle;
    void (*fun)(void *);
    void *arg;
    volatile bool terminate;
    volatile bool finished;
    char name[16];
    // Одноразовый аппаратный таймер пробуждения. Нужен, потому что
    // vTaskDelay квантуется тиком в 1 мс, а Refloat просит нецелое число
    // тиков (см. комментарий в if_sleep_us).
    esp_timer_handle_t waker;
} FcThread;

vesc_c_if *floatcore_vesc_if = NULL;

static struct {
    vesc_c_if IF;
    FcThread threads[FC_MAX_THREADS];
    size_t thread_count;

    void **arg_slot;
    void *arg_fallback;

    // Конфигурация «прошивки VESC», которую читает Refloat.
    float cfg_float[64];
    int cfg_int[64];

    // Регистрация Custom Config (транспорт VESC Tool на ESP32 появится позже).
    int (*cfg_get)(uint8_t *data, bool is_default);
    bool (*cfg_set)(uint8_t *data);
    int (*cfg_get_xml)(uint8_t **data);
    void (*app_data_handler)(unsigned char *data, unsigned int len);

    uint64_t log_lines;
} S;

// ------------------------------------------------------------------ время
// SYSTEM_TICK_RATE_HZ = 10000 → один тик = 100 мкс.

static void if_sleep_us(uint32_t us) {
    // Учёт периода главного потока: Refloat вызывает sleep_us ровно один раз
    // за итерацию refloat_thd (main.c:777) и aux_thd (main.c:1143).
    // Меряем здесь, чтобы не трогать upstream.
    FcThread *t = NULL;
    TaskHandle_t self = xTaskGetCurrentTaskHandle();
    for (size_t i = 0; i < S.thread_count; ++i) {
        if (S.threads[i].handle == self) {
            t = &S.threads[i];
            break;
        }
    }
    if (t) {
        FcTimingChannel ch = (t == &S.threads[0]) ? FC_TIMING_MAIN : FC_TIMING_AUX;
        // Итерация потока закончилась ровно здесь: дальше он засыпает.
        fc_timing_exec_end(ch);
        fc_timing_tick(ch);
        // Задача жива — гасим watchdog именно здесь: это единственная точка,
        // которую главный цикл проходит на каждой итерации (ТЗ §11).
        esp_task_wdt_reset();
    }

    if (us == 0) {
        taskYIELD();
        return;
    }
    // Почему здесь аппаратный таймер, а не vTaskDelay.
    //
    // Refloat никогда не просит ровно 2000 мкс: он вычитает время своей
    // итерации (main.c:1058-1059), поэтому реальный запрос — 1800 мкс и
    // подобные. Тик FreeRTOS равен 1000 мкс, то есть такой интервал
    // принципиально не выражается целым числом тиков.
    //
    // Прежняя реализация досыпала остаток циклом taskYIELD. Профилирование
    // показало, что именно она и есть источник знаменитого «джиттера
    // ±0.8 мс»: главный поток Refloat проводил в этом цикле по 800 мкс из
    // каждых 1800, дёргая планировщик ядра 1 сотни раз подряд и мешая
    // контуру (docs/realtime_timing.md §4).
    //
    // esp_timer работает от аппаратного таймера с разрешением 1 мкс и будит
    // задачу уведомлением из обработчика прерывания. Ни квантования тиком,
    // ни активного ожидания.
    if (t && t->waker) {
        esp_timer_start_once(t->waker, us);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    } else {
        // Вызов не из задачи Refloat (например, из инициализации): точность
        // здесь не нужна, тика достаточно.
        vTaskDelay(pdMS_TO_TICKS((us + 999) / 1000));
    }

    if (t) {
        // Проснулись — начинается новая итерация потока Refloat.
        fc_timing_exec_begin((t == &S.threads[0]) ? FC_TIMING_MAIN : FC_TIMING_AUX);
    }
}

static void if_sleep_ms(uint32_t ms) {
    if_sleep_us(ms * 1000);
}

static float if_system_time(void) {
    return (float) (esp_timer_get_time() * 1e-6);
}

static systime_t if_system_time_ticks(void) {
    return (systime_t) (esp_timer_get_time() / 100);
}

static uint32_t if_timer_time_now(void) {
    return (uint32_t) esp_timer_get_time();
}

static float if_timer_seconds_elapsed_since(uint32_t t) {
    // Разностная арифметика в uint32: корректно переживает переполнение
    // раз в ~71 минуту (docs/vesc_if_contract.md §1).
    return (float) ((uint32_t) esp_timer_get_time() - t) * 1e-6f;
}

static float if_ts_to_age_s(systime_t ts) {
    return (float) ((systime_t) (esp_timer_get_time() / 100) - ts) / (float) SYSTEM_TICK_RATE_HZ;
}

// ------------------------------------------------------------------ потоки

// Обработчик таймера пробуждения. Выполняется в задаче esp_timer, поэтому
// уведомление отправляется обычной, а не ISR-версией функции.
static void waker_cb(void *arg) {
    TaskHandle_t task = (TaskHandle_t) arg;
    if (task) {
        xTaskNotifyGive(task);
    }
}

static void thread_trampoline(void *arg) {
    FcThread *t = (FcThread *) arg;

    // Таймер пробуждения создаётся внутри самой задачи: ему нужен её
    // дескриптор, который до старта ещё неизвестен.
    esp_timer_create_args_t targs = {
        .callback = waker_cb,
        .arg = xTaskGetCurrentTaskHandle(),
        .dispatch_method = ESP_TIMER_TASK,
        .name = "fc_wake",
    };
    if (esp_timer_create(&targs, &t->waker) != ESP_OK) {
        t->waker = NULL;
        ESP_LOGW(TAG, "%s: таймер пробуждения не создан, ожидание пойдёт по тикам", t->name);
    }

    // Задачи Refloat подписаны на TWDT: зависание контура должно быть видно.
    esp_task_wdt_add(NULL);
    t->fun(t->arg);
    esp_task_wdt_delete(NULL);
    if (t->waker) {
        esp_timer_stop(t->waker);
        esp_timer_delete(t->waker);
        t->waker = NULL;
    }
    t->finished = true;
    t->handle = NULL;
    vTaskDelete(NULL);
}

static lib_thread if_spawn(void (*fun)(void *), size_t stack, const char *name, void *arg) {
    if (S.thread_count >= FC_MAX_THREADS) {
        ESP_LOGE(TAG, "spawn(%s): превышен лимит задач", name ? name : "?");
        return NULL;
    }
    FcThread *t = &S.threads[S.thread_count];
    memset(t, 0, sizeof(*t));
    t->fun = fun;
    t->arg = arg;
    snprintf(t->name, sizeof(t->name), "%s", name ? name : "refloat");

    // Refloat просит 1536 байт — этого мало для Xtensa с его оконными
    // регистрами и float-ами на стеке (docs/threading_model.md §4, пункт 5).
    size_t stack_bytes = stack * FC_STACK_SCALE;
    if (stack_bytes < 4096) {
        stack_bytes = 4096;
    }

    // Все задачи Refloat — на ядро 1. Это воспроизводит однопроцессорную
    // семантику STM32, на которую Refloat рассчитан (в нём нет ни одного
    // мьютекса; docs/threading_model.md §3).
    BaseType_t ok = xTaskCreatePinnedToCore(
        thread_trampoline, t->name, stack_bytes, t, FC_PRIO_REFLOAT, &t->handle,
        FC_CORE_REALTIME
    );
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "spawn(%s): не удалось создать задачу", t->name);
        return NULL;
    }
    ESP_LOGI(TAG, "spawn: %s, стек %u Б (запрошено %u), ядро %d, приоритет %d",
             t->name, (unsigned) stack_bytes, (unsigned) stack, FC_CORE_REALTIME, FC_PRIO_REFLOAT);
    ++S.thread_count;
    return (lib_thread) t;
}

static void if_request_terminate(lib_thread th) {
    FcThread *t = (FcThread *) th;
    if (!t) {
        return;
    }
    t->terminate = true;
    for (int i = 0; i < 200 && !t->finished; ++i) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static bool if_should_terminate(void) {
    TaskHandle_t self = xTaskGetCurrentTaskHandle();
    for (size_t i = 0; i < S.thread_count; ++i) {
        if (S.threads[i].handle == self) {
            return S.threads[i].terminate;
        }
    }
    return false;
}

static void if_thread_set_priority(int priority) {
    // Контракт VESC: -5…5, 0 — норма. aux_thd просит -1.
    if (priority < -5) {
        priority = -5;
    }
    if (priority > 5) {
        priority = 5;
    }
    int prio = FC_PRIO_REFLOAT + priority * 2;
    if (prio < 1) {
        prio = 1;
    }
    if (prio > FC_PRIO_IMU - 1) {
        prio = FC_PRIO_IMU - 1;
    }
    vTaskPrioritySet(NULL, prio);
}

static void **if_get_arg(uint32_t prog_addr) {
    (void) prog_addr;
    return S.arg_slot ? S.arg_slot : &S.arg_fallback;
}

void floatcore_set_arg_slot(void **slot) {
    S.arg_slot = slot;
}

// ------------------------------------------------------------------- прочее

static int if_printf(const char *fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    ++S.log_lines;
    ESP_LOGI("refloat", "%s", buf);
    return n;
}

static void *if_malloc(size_t n) {
    return heap_caps_malloc(n, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
}

static void if_free(void *p) {
    heap_caps_free(p);
}

// --------------------------------------------------------------------- IMU

static bool if_imu_startup_done(void) {
    return fc_imu_startup_done();
}

static float if_imu_get_roll(void) {
    float v;
    fc_imu_get_state(&v, NULL, NULL, NULL, NULL, NULL);
    return v;
}

static float if_imu_get_pitch(void) {
    float v;
    fc_imu_get_state(NULL, &v, NULL, NULL, NULL, NULL);
    return v;
}

static float if_imu_get_yaw(void) {
    float v;
    fc_imu_get_state(NULL, NULL, &v, NULL, NULL, NULL);
    return v;
}

static void if_imu_get_gyro(float *g) {
    fc_imu_get_state(NULL, NULL, NULL, NULL, g, NULL);
}

static void if_imu_get_accel(float *a) {
    fc_imu_get_state(NULL, NULL, NULL, a, NULL, NULL);
}

static void if_imu_get_quaternions(float *q) {
    fc_imu_get_state(NULL, NULL, NULL, NULL, NULL, q);
}

static void if_imu_set_read_callback(void (*cb)(float *, float *, float *, float)) {
    fc_imu_set_callback(cb);
}

// ---------------------------------------------------------------------- IO

static float if_io_read_analog(VESC_PIN pin) {
    return fc_adc_read((int) pin);
}

static bool if_io_set_mode(VESC_PIN pin, VESC_PIN_MODE mode) {
    (void) pin;
    (void) mode;
    return false;  // GPIO платы на v0.5 не заводятся
}

static bool if_io_write(VESC_PIN pin, int state) {
    (void) pin;
    (void) state;
    return false;  // пищалка не подключена
}

static bool if_io_read(VESC_PIN pin) {
    (void) pin;
    return false;
}

static void if_set_pad_mode(void *gpio, uint32_t pin, uint32_t mode) {
    (void) gpio;
    (void) pin;
    (void) mode;
}

// ------------------------------------------------------------------- мотор
//
// Единственный путь наружу — fc_motor_gate_request(). Заполнены ВСЕ указатели
// SDK, способные подать что-либо на мотор (vesc_c_if.h:436-447, 476, 653), а
// не только те, которыми пользуется Refloat: оставленный NULL — это место,
// куда однажды впишут прямой вызов backend-а мимо политики.

static void gate(FcMotorRequestKind kind, float value) {
    fc_motor_gate_request(kind, value, (uint64_t) esp_timer_get_time());
}

static void if_mc_set_current(float current) {
    gate(FC_MOTOR_REQ_CURRENT, current);
}

static void if_mc_set_brake_current(float current) {
    gate(FC_MOTOR_REQ_BRAKE_CURRENT, current);
}

static void if_mc_set_current_rel(float v) {
    gate(FC_MOTOR_REQ_CURRENT_REL, v);
}

static void if_mc_set_brake_current_rel(float v) {
    gate(FC_MOTOR_REQ_BRAKE_CURRENT_REL, v);
}

static void if_mc_set_duty(float duty) {
    gate(FC_MOTOR_REQ_DUTY, duty);
}

static void if_mc_set_duty_noramp(float duty) {
    gate(FC_MOTOR_REQ_DUTY_NORAMP, duty);
}

static void if_mc_set_pid_speed(float rpm) {
    gate(FC_MOTOR_REQ_PID_SPEED, rpm);
}

static void if_mc_set_pid_pos(float pos) {
    gate(FC_MOTOR_REQ_PID_POS, pos);
}

static void if_mc_set_handbrake(float current) {
    gate(FC_MOTOR_REQ_HANDBRAKE, current);
}

static void if_mc_set_handbrake_rel(float v) {
    gate(FC_MOTOR_REQ_HANDBRAKE_REL, v);
}

static void if_mc_release_motor(void) {
    gate(FC_MOTOR_REQ_RELEASE, 0.0f);
}

static bool if_foc_play_tone(int channel, float freq, float voltage) {
    // Звук на VESC издаётся подачей напряжения на обмотки, поэтому это тоже
    // выход на мотор и он идёт через ту же политику.
    (void) channel;
    (void) freq;
    gate(FC_MOTOR_REQ_TONE, voltage);
    return false;
}

static void if_mc_set_current_off_delay(float d) {
    // Не выход: параметр следующей команды тока, которой не будет.
    (void) d;
}

static void if_timeout_reset(void) {
    fc_motor_gate_keepalive();
}

// -------------------------------------------------------------- телеметрия
// ESC не подключены. Отдаём нули — Refloat увидит неподвижный мотор.
// Напряжение подставляем номинальным, иначе Refloat немедленно объявит
// low-voltage tiltback и это замаскирует настоящее поведение контура.

#define FC_STUB_VOLTAGE 60.0f
#define FC_STUB_TEMP 25.0f

static float if_zero(void) {
    return 0.0f;
}

static float if_zero_bool(bool reset) {
    (void) reset;
    return 0.0f;
}

static float if_stub_voltage(void) {
    return FC_STUB_VOLTAGE;
}

static float if_stub_temp(void) {
    return FC_STUB_TEMP;
}

static uint64_t if_zero_u64(void) {
    return 0;
}

static mc_fault_code if_mc_get_fault(void) {
    return FAULT_CODE_NONE;
}

static const char *if_mc_fault_to_string(mc_fault_code f) {
    return f == FAULT_CODE_NONE ? "FAULT_CODE_NONE" : "FAULT_CODE_UNKNOWN";
}

static float if_mc_get_battery_level(float *wh_left) {
    if (wh_left) {
        *wh_left = 0.0f;
    }
    return 0.0f;
}

static gnss_data g_gnss;

static volatile gnss_data *if_mc_gnss(void) {
    return &g_gnss;
}

// ------------------------------------------------------------ конфигурация

static float if_get_cfg_float(CFG_PARAM p) {
    return ((unsigned) p < 64) ? S.cfg_float[p] : 0.0f;
}

static int if_get_cfg_int(CFG_PARAM p) {
    return ((unsigned) p < 64) ? S.cfg_int[p] : 0;
}

static bool if_set_cfg_float(CFG_PARAM p, float v) {
    if ((unsigned) p >= 64) {
        return false;
    }
    S.cfg_float[p] = v;
    return true;
}

static bool if_set_cfg_int(CFG_PARAM p, int v) {
    if ((unsigned) p >= 64) {
        return false;
    }
    S.cfg_int[p] = v;
    return true;
}

static void if_conf_custom_add_config(
    int (*get_cfg)(uint8_t *data, bool is_default), bool (*set_cfg)(uint8_t *data),
    int (*get_cfg_xml)(uint8_t **data)
) {
    S.cfg_get = get_cfg;
    S.cfg_set = set_cfg;
    S.cfg_get_xml = get_cfg_xml;
    ESP_LOGI(TAG, "config registered: get=%p set=%p xml=%p", (void *) get_cfg, (void *) set_cfg,
             (void *) get_cfg_xml);
}

static void if_conf_custom_clear_configs(void) {
    S.cfg_get = NULL;
    S.cfg_set = NULL;
    S.cfg_get_xml = NULL;
}

bool fc_config_registered(void) {
    return S.cfg_get && S.cfg_set && S.cfg_get_xml;
}

int fc_config_read(uint8_t *data, bool is_default) {
    return S.cfg_get ? S.cfg_get(data, is_default) : 0;
}

// ---------------------------------------------------------------- хранилище

static bool if_read_eeprom_var(eeprom_var *v, int address) {
    uint32_t w = 0;
    if (!fc_storage_read(&w, address)) {
        return false;
    }
    v->as_u32 = w;
    return true;
}

static bool if_store_eeprom_var(eeprom_var *v, int address) {
    // Политика записи проверяется здесь, до всякого обращения к носителю
    // (ТЗ v0.6A §24). Refloat получает честный false и печатает свою ошибку.
    if (!fc_supervisor_config_write_allowed()) {
        fc_storage_note_rejected_write();
        return false;
    }
    return fc_storage_write(v->as_u32, address);
}

static bool if_store_backup_data(void) {
    if (!fc_supervisor_config_write_allowed()) {
        fc_storage_note_rejected_write();
        return false;
    }
    return fc_storage_request_commit();
}

// ---------------------------------------------------------------- обмен с UI

static void if_send_app_data(unsigned char *data, unsigned int len) {
    (void) data;
    (void) len;
    // Транспорт VESC Tool на ESP32 — следующий этап (ТЗ §18).
}

static bool if_set_app_data_handler(void (*func)(unsigned char *data, unsigned int len)) {
    S.app_data_handler = func;
    return true;
}

static bool if_app_is_output_disabled(void) {
    return false;
}

// ------------------------------------------------------------------- пульт

static remote_state if_get_remote_state(void) {
    remote_state r;
    memset(&r, 0, sizeof(r));
    return r;
}

// --------------------------------------------------------------------- LBM

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

// --------------------------------------------------------------- сборка IF

void fc_vesc_if_init(void) {
    memset(&S, 0, sizeof(S));

    vesc_c_if *IF = &S.IF;

    IF->sleep_us = if_sleep_us;
    IF->sleep_ms = if_sleep_ms;
    IF->system_time = if_system_time;
    IF->system_time_ticks = if_system_time_ticks;
    IF->ts_to_age_s = if_ts_to_age_s;
    IF->timer_time_now = if_timer_time_now;
    IF->timer_seconds_elapsed_since = if_timer_seconds_elapsed_since;
    IF->printf = if_printf;
    IF->malloc = if_malloc;
    IF->free = if_free;
    IF->spawn = if_spawn;
    IF->request_terminate = if_request_terminate;
    IF->should_terminate = if_should_terminate;
    IF->thread_set_priority = if_thread_set_priority;
    IF->get_arg = if_get_arg;

    IF->imu_startup_done = if_imu_startup_done;
    IF->imu_get_roll = if_imu_get_roll;
    IF->imu_get_pitch = if_imu_get_pitch;
    IF->imu_get_yaw = if_imu_get_yaw;
    IF->imu_get_gyro = if_imu_get_gyro;
    IF->imu_get_accel = if_imu_get_accel;
    IF->imu_get_quaternions = if_imu_get_quaternions;
    IF->imu_set_read_callback = if_imu_set_read_callback;

    IF->io_read_analog = if_io_read_analog;
    IF->io_set_mode = if_io_set_mode;
    IF->io_write = if_io_write;
    IF->io_read = if_io_read;
    IF->set_pad_mode = if_set_pad_mode;

    // Все выходы на мотор — через Motor Gate, включая те, которых Refloat не
    // использует: оставленный NULL стал бы дырой в политике.
    IF->mc_set_current = if_mc_set_current;
    IF->mc_set_brake_current = if_mc_set_brake_current;
    IF->mc_set_current_rel = if_mc_set_current_rel;
    IF->mc_set_brake_current_rel = if_mc_set_brake_current_rel;
    IF->mc_set_duty = if_mc_set_duty;
    IF->mc_set_duty_noramp = if_mc_set_duty_noramp;
    IF->mc_set_pid_speed = if_mc_set_pid_speed;
    IF->mc_set_pid_pos = if_mc_set_pid_pos;
    IF->mc_set_handbrake = if_mc_set_handbrake;
    IF->mc_set_handbrake_rel = if_mc_set_handbrake_rel;
    IF->mc_release_motor = if_mc_release_motor;
    IF->foc_play_tone = if_foc_play_tone;
    IF->mc_set_current_off_delay = if_mc_set_current_off_delay;
    IF->timeout_reset = if_timeout_reset;

    IF->mc_get_rpm = if_zero;
    IF->mc_get_duty_cycle_now = if_zero;
    IF->mc_get_tot_current = if_zero;
    IF->mc_get_tot_current_filtered = if_zero;
    IF->mc_get_tot_current_directional = if_zero;
    IF->mc_get_tot_current_directional_filtered = if_zero;
    IF->mc_get_tot_current_in = if_zero;
    IF->mc_get_tot_current_in_filtered = if_zero;
    IF->mc_get_input_voltage_filtered = if_stub_voltage;
    IF->mc_temp_fet_filtered = if_stub_temp;
    IF->mc_temp_motor_filtered = if_stub_temp;
    IF->mc_get_speed = if_zero;
    IF->mc_get_distance = if_zero;
    IF->mc_get_distance_abs = if_zero;
    IF->mc_get_odometer = if_zero_u64;
    IF->mc_get_fault = if_mc_get_fault;
    IF->mc_fault_to_string = if_mc_fault_to_string;
    IF->mc_get_battery_level = if_mc_get_battery_level;
    IF->mc_get_amp_hours = if_zero_bool;
    IF->mc_get_amp_hours_charged = if_zero_bool;
    IF->mc_get_watt_hours = if_zero_bool;
    IF->mc_get_watt_hours_charged = if_zero_bool;
    IF->mc_gnss = if_mc_gnss;
    IF->foc_get_id = if_zero;
    IF->foc_get_iq = if_zero;

    IF->get_cfg_float = if_get_cfg_float;
    IF->get_cfg_int = if_get_cfg_int;
    IF->set_cfg_float = if_set_cfg_float;
    IF->set_cfg_int = if_set_cfg_int;
    IF->conf_custom_add_config = if_conf_custom_add_config;
    IF->conf_custom_clear_configs = if_conf_custom_clear_configs;

    IF->read_eeprom_var = if_read_eeprom_var;
    IF->store_eeprom_var = if_store_eeprom_var;
    IF->store_backup_data = if_store_backup_data;

    IF->send_app_data = if_send_app_data;
    IF->set_app_data_handler = if_set_app_data_handler;
    IF->app_is_output_disabled = if_app_is_output_disabled;

    IF->get_remote_state = if_get_remote_state;
    IF->get_ppm = if_zero;
    IF->get_ppm_age = if_zero;

    IF->lbm_add_extension = if_lbm_add_extension;
    IF->lbm_dec_as_float = if_lbm_dec_as_float;
    IF->lbm_dec_as_i32 = if_lbm_dec_as_i32;
    IF->lbm_enc_sym_nil = 0;
    IF->lbm_enc_sym_true = 1;

    IF->plot_init = if_plot_init;
    IF->plot_add_graph = if_plot_add_graph;
    IF->plot_set_graph = if_plot_set_graph;
    IF->plot_send_points = if_plot_send_points;


    // Значения «конфигурации ESC». ESC нет, поэтому это заведомо консервативные
    // числа; реальные придут по CAN на следующем этапе. Ток намеренно мал:
    // даже если бы выход существовал, Refloat не запросил бы много.
    S.cfg_float[CFG_PARAM_l_current_max] = 10.0f;
    S.cfg_float[CFG_PARAM_l_current_min] = -10.0f;
    S.cfg_float[CFG_PARAM_l_in_current_max] = 10.0f;
    S.cfg_float[CFG_PARAM_l_in_current_min] = -5.0f;
    S.cfg_float[CFG_PARAM_l_temp_fet_start] = 85.0f;
    S.cfg_float[CFG_PARAM_l_temp_motor_start] = 85.0f;
    S.cfg_float[CFG_PARAM_l_max_duty] = 0.95f;
    S.cfg_float[CFG_PARAM_foc_motor_flux_linkage] = 0.0045f;
    S.cfg_float[CFG_PARAM_IMU_mahony_kp] = 0.2f;
    S.cfg_int[CFG_PARAM_si_motor_poles] = 30;
    S.cfg_int[CFG_PARAM_si_battery_cells] = 15;
    // Реальная частота mock-IMU: ноль Refloat трактует как FW 6.02.
    S.cfg_int[CFG_PARAM_IMU_sample_rate] = fc_imu_rate_hz();

    floatcore_vesc_if = IF;
}

void fc_vesc_if_deinit(void) {
    floatcore_vesc_if = NULL;
}

const char *fc_thread_name(size_t i) {
    return i < S.thread_count ? S.threads[i].name : NULL;
}

size_t fc_thread_count(void) {
    return S.thread_count;
}

uint32_t fc_thread_stack_watermark(size_t i) {
    if (i >= S.thread_count || !S.threads[i].handle) {
        return 0;
    }
    return (uint32_t) uxTaskGetStackHighWaterMark(S.threads[i].handle);
}

// Вызов зарегистрированного Refloat set_cfg — общий контракт платформы,
// используется refloat_facade_config_save_test() (ТЗ §12).
bool floatcore_config_apply(uint8_t *data) {
    return S.cfg_set ? S.cfg_set(data) : false;
}
