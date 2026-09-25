// Тонкая обёртка над Refloat для тестов.
//
// Изолирует заголовки Refloat от системных: файлы Refloat объявляют
// `typedef uint32_t time_t`, что конфликтует с newlib/libc. Поэтому весь код,
// включающий заголовки Refloat, живёт в refloat_facade.c, а тесты работают
// только с этим заголовком (stdint/stdbool/stddef).
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int state;            // RunState: 0 DISABLED, 1 STARTUP, 2 READY, 3 RUNNING
    int mode;             // Mode
    int sat;              // SetpointAdjustmentType
    int stop_condition;   // StopCondition
    int footpad_state;    // FootpadSensorState: 0 NONE, 1 LEFT, 2 RIGHT, 3 BOTH
    float adc_left, adc_right;
    // Пороги срабатывания footpad из конфигурации: нужны, чтобы доказать, что
    // отданное платформой напряжение действительно означает disengaged
    // (footpad_sensor.c:31 — порог 0 означает «датчик всегда нажат»).
    float fault_adc1, fault_adc2;
    float pitch, balance_pitch, roll, pitch_rate;
    float setpoint, setpoint_target;
    float balance_current;
    float motor_erpm, motor_duty, motor_current;
    // Пределы, которые Refloat прочитал через VESC_IF->get_cfg_float().
    // Нужны, чтобы проверить: Virtual mcConfig показывает UI ровно их.
    float motor_current_max, motor_current_min;
    float motor_batt_current_max, motor_batt_current_min;
    float mosfet_temp_max, motor_temp_max;
    float lv_threshold, hv_threshold;
    bool darkride;
    bool traction_control;
    float imu_frequency, main_frequency;
} RefloatSnapshot;

/** Инициализация Refloat (вызов его init()). Возвращает false при ошибке. */
bool refloat_facade_start(void);

/** Останов Refloat (его stop_fun). */
void refloat_facade_stop(void);

RefloatSnapshot refloat_facade_snapshot(void);

/** Состояние датчиков ног (FootpadState), 0 — не нажаты. Для опроса супервизором. */
int refloat_facade_footpad_state(void);

// Лёгкий срез состояния для теневого наблюдателя (ТЗ v0.9D §7).
//
// Отдельно от RefloatSnapshot намеренно: тот копирует три десятка полей и
// читается человеком по запросу, а это снимается 500 раз в секунду из пути
// контура. Копировать туда лишнее — значит тратить время контура на то, что
// никто не читает.
//
// Составляющие ПИД берутся из pid.h (поля p, i, rate_p). Upstream ради этого
// НЕ менялся: структура уже публична, просто до неё не доходили.
typedef struct {
    float pitch, roll, pitch_rate;
    // Регулятор работает от balance_pitch, а не от pitch: это отфильтрованный
    // и смещённый угол (imu.c). Считать ошибку по pitch — значит считать не ту
    // величину, что и делает контур.
    float balance_pitch;
    float setpoint;
    float balance_current;
    float pid_p, pid_i, pid_rate_p;
    int sat;
    int state;
} RefloatShadowFields;

void refloat_facade_shadow(RefloatShadowFields *out);

// Действующие коэффициенты балансировки. Нужны, чтобы формулу можно было
// проверить численно, а не принять на веру по умолчаниям.
typedef struct {
    float kp, kp2, ki, ki_limit;
    float kp_brake, kp2_brake;
    float mahony_kp;
    float booster_current, brkbooster_current;
    float torque_constant_compat;  // постоянная эталонного мотора, Н·м/А
    float speed_constant;          // 1/Kt нашего, А/Н·м
} RefloatGains;

void refloat_facade_gains(RefloatGains *out);

// Условия входа и всё, что меняет поведение контура в первые миллисекунды
// после engage (ТЗ v0.9K §2, §5). Только чтение.
typedef struct {
    float startup_pitch_tolerance, startup_roll_tolerance, startup_speed;
    float startup_click_current;
    bool startup_simplestart_enabled, startup_pushstart_enabled;
    float fault_pitch, fault_roll, fault_adc1, fault_adc2;
    bool fault_is_dual_switch;
    uint16_t fault_delay_pitch;
    int parking_brake_mode;
    float booster_angle, booster_current, brkbooster_angle, brkbooster_current;
    float torquetilt_strength, atr_strength_up, turntilt_strength;
    float motor_current_max, motor_current_min;  // как их видит Refloat
} RefloatStartupConf;

void refloat_facade_startup_conf(RefloatStartupConf *out);

// Временная правка коэффициентов для ТЕНЕВЫХ опытов (ТЗ v0.9F §11, §12).
//
// Меняется только конфигурация в памяти Refloat, upstream не трогается.
// Исходные значения сохраняются при первом вызове и восстанавливаются
// refloat_facade_restore_gains(). Оставить стенд в изменённой конфигурации
// без явного восстановления — верный способ потом гадать, что измеряли.
//
// Масштаб применяется к kp, kp2 и ki ОДНОВРЕМЕННО: пока не понятен общий
// масштаб, разделять вклады рано (ТЗ §11).
bool refloat_facade_scale_gains(float scale);

/** Задать составляющие по отдельности. Отрицательное значение — не менять. */
bool refloat_facade_set_gains(float kp, float kp2, float ki);

/**
 * Отключить интегральную часть для изоляции P и D (ТЗ v0.9F §12, §13).
 *
 * В теневом режиме контур разомкнут, ошибка никогда не устраняется, и
 * интегратор упирается в свой предел за секунды. Мерить по нему масштаб
 * бессмысленно: он покажет ki_limit, а не отклик регулятора.
 */
bool refloat_facade_disable_integral(void);

bool refloat_facade_restore_gains(void);
bool refloat_facade_gains_modified(void);

const char *refloat_facade_state_name(int state);
const char *refloat_facade_stop_name(int stop_condition);
const char *refloat_facade_footpad_name(int fs);

// --------------------------------------------------- проверка persistence
//
// Гоняет конфигурацию Refloat через его же путь сохранения — тот самый, что
// использует VESC Tool: зарегистрированный через conf_custom_add_config
// set_cfg() десериализует данные и вызывает write_cfg_to_eeprom().
//
// Меняется только `leds.status.brightness_headlights_off` — яркость фар в
// статус-баре. Параметр выбран потому, что он не участвует ни в одном
// вычислении контура, а лента к плате не подключена вовсе.
//
// Платформа обязана предоставить floatcore_config_apply(): вызов сохранённого
// указателя set_cfg.
/**
 * Пороги отката по напряжению, В на ячейку (ТЗ v0.7D.1).
 *
 * Отвергает значения вне окна 2…5 В на ячейку и случай hv <= lv: опечатка в
 * пороге предупреждения водителю — тот случай, когда отказать лучше, чем
 * записать.
 */
bool refloat_facade_set_voltage_tiltback(float lv_per_cell, float hv_per_cell);
void refloat_facade_get_voltage_tiltback(float *lv_per_cell, float *hv_per_cell);

bool refloat_facade_config_save_test(float value);
float refloat_facade_config_test_value(void);
