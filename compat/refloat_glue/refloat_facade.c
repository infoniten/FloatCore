// Единственная единица трансляции теста, включающая заголовки Refloat.
// Системные заголовки здесь запрещены (см. refloat_facade.h).

#include "refloat_facade.h"

#include "data.h"
#include "conf/confparser.h"
#include "lib/utils.h"
#include "vesc_c_if.h"

// Реализуется backend-ом платформы (host-mock или platform/esp32). Объявляется здесь,
// потому что заголовки backend-ов нельзя включать вместе с заголовками Refloat.
void floatcore_set_arg_slot(void **slot);

// init() Refloat, переименованный макросом INIT_FUN в нашем shim-заголовке
bool refloat_init(lib_info *info);

static lib_info info;
static bool started = false;

bool refloat_facade_start(void) {
    // ARG раскрывается в *VESC_IF->get_arg(); на VESC это ячейка lib_info.arg,
    // которой владеет загрузчик. Воспроизводим это.
    floatcore_set_arg_slot(&info.arg);
    started = refloat_init(&info);
    return started;
}

void refloat_facade_stop(void) {
    if (started && info.stop_fun) {
        info.stop_fun(info.arg);
        started = false;
    }
}

void refloat_facade_shadow(RefloatShadowFields *out) {
    if (!out) {
        return;
    }
    if (!started) {
        RefloatShadowFields z = {0};
        *out = z;
        return;
    }
    const Data *d = (const Data *) info.arg;
    out->pitch = d->imu.pitch;
    out->balance_pitch = d->imu.balance_pitch;
    out->roll = d->imu.roll;
    out->pitch_rate = d->imu.pitch_rate;
    out->setpoint = d->setpoint;
    out->balance_current = d->balance_current.value;
    out->pid_p = d->pid.p;
    out->pid_i = d->pid.i;
    out->pid_rate_p = d->pid.rate_p;
    out->sat = (int) d->state.sat;
    out->state = (int) d->state.state;
}

void refloat_facade_gains(RefloatGains *out) {
    if (!out) {
        return;
    }
    RefloatGains z = {0};
    *out = z;
    if (!started) {
        return;
    }
    const Data *d = (const Data *) info.arg;
    out->kp = d->float_conf.kp;
    out->kp2 = d->float_conf.kp2;
    out->ki = d->float_conf.ki;
    out->ki_limit = d->float_conf.ki_limit;
    out->kp_brake = d->float_conf.kp_brake;
    out->kp2_brake = d->float_conf.kp2_brake;
    out->mahony_kp = d->float_conf.mahony_kp;
    out->booster_current = d->float_conf.booster_current;
    out->brkbooster_current = d->float_conf.brkbooster_current;
    out->torque_constant_compat = TORQUE_CONSTANT_COMPAT;
    out->speed_constant = d->motor.speed_constant;
}

void refloat_facade_startup_conf(RefloatStartupConf *out) {
    if (!out) {
        return;
    }
    RefloatStartupConf z = {0};
    *out = z;
    if (!started) {
        return;
    }
    const Data *d = (const Data *) info.arg;
    const RefloatConfig *c = &d->float_conf;
    out->startup_pitch_tolerance = c->startup_pitch_tolerance;
    out->startup_roll_tolerance = c->startup_roll_tolerance;
    out->startup_speed = c->startup_speed;
    out->startup_click_current = c->startup_click_current;
    out->startup_simplestart_enabled = c->startup_simplestart_enabled;
    out->startup_pushstart_enabled = c->startup_pushstart_enabled;
    out->fault_pitch = c->fault_pitch;
    out->fault_roll = c->fault_roll;
    out->fault_adc1 = c->fault_adc1;
    out->fault_adc2 = c->fault_adc2;
    out->fault_is_dual_switch = c->fault_is_dual_switch;
    out->fault_delay_pitch = c->fault_delay_pitch;
    out->parking_brake_mode = (int) c->parking_brake_mode;
    out->booster_angle = c->booster_angle;
    out->booster_current = c->booster_current;
    out->brkbooster_angle = c->brkbooster_angle;
    out->brkbooster_current = c->brkbooster_current;
    out->torquetilt_strength = c->torquetilt_strength;
    out->atr_strength_up = c->atr_strength_up;
    out->turntilt_strength = c->turntilt_strength;
    out->motor_current_max = d->motor.current_max;
    out->motor_current_min = d->motor.current_min;
}

// Исходные коэффициенты. Сохраняются при первой правке, чтобы стенд всегда
// можно было вернуть в состояние, в котором снимались прежние измерения.
static struct {
    bool saved;
    float kp, kp2, ki, ki_limit;
} BASELINE;

static void reset_integral(Data *d) {
    // Накопленный интеграл принадлежит прежним коэффициентам. Оставить его
    // при смене масштаба — значит мерить отклик, к которому примешано
    // состояние, набранное при другой настройке.
    //
    // Отдельно важно для отключения интеграла: обнуление ki останавливает
    // РОСТ, но само значение остаётся в pid->i навсегда, а ki_limit = 0
    // вдобавок снимает ограничитель целиком (pid.c:64 зажимает только при
    // ki_limit > 0). Без этого сброса «интеграл отключён» означает «интеграл
    // заморожен на последнем значении».
    d->pid.i = 0.0f;
}

static Data *mutable_data(void) {
    return started ? (Data *) info.arg : NULL;
}

static void save_baseline(const Data *d) {
    if (!BASELINE.saved) {
        BASELINE.kp = d->float_conf.kp;
        BASELINE.kp2 = d->float_conf.kp2;
        BASELINE.ki = d->float_conf.ki;
        BASELINE.ki_limit = d->float_conf.ki_limit;
        BASELINE.saved = true;
    }
}

bool refloat_facade_scale_gains(float scale) {
    Data *d = mutable_data();
    if (!d || !(scale > 0.0f) || scale > 10.0f) {
        return false;
    }
    save_baseline(d);
    d->float_conf.kp = BASELINE.kp * scale;
    d->float_conf.kp2 = BASELINE.kp2 * scale;
    d->float_conf.ki = BASELINE.ki * scale;
    // Предел интегратора масштабируется ВМЕСТЕ с ki. Без этого интеграл
    // накручивается до прежнего потолка независимо от масштаба, и вся
    // развёртка показывает один и тот же упор.
    d->float_conf.ki_limit = BASELINE.ki_limit * scale;
    reset_integral(d);
    return true;
}

bool refloat_facade_set_gains(float kp, float kp2, float ki) {
    Data *d = mutable_data();
    if (!d) {
        return false;
    }
    save_baseline(d);
    if (kp >= 0.0f) {
        d->float_conf.kp = kp;
    }
    if (kp2 >= 0.0f) {
        d->float_conf.kp2 = kp2;
    }
    if (ki >= 0.0f) {
        d->float_conf.ki = ki;
    }
    reset_integral(d);
    return true;
}

bool refloat_facade_disable_integral(void) {
    Data *d = mutable_data();
    if (!d) {
        return false;
    }
    save_baseline(d);
    d->float_conf.ki = 0.0f;
    d->float_conf.ki_limit = 0.0f;
    reset_integral(d);
    return true;
}

bool refloat_facade_restore_gains(void) {
    Data *d = mutable_data();
    if (!d || !BASELINE.saved) {
        return false;
    }
    d->float_conf.kp = BASELINE.kp;
    d->float_conf.kp2 = BASELINE.kp2;
    d->float_conf.ki = BASELINE.ki;
    d->float_conf.ki_limit = BASELINE.ki_limit;
    reset_integral(d);
    BASELINE.saved = false;
    return true;
}

bool refloat_facade_gains_modified(void) {
    return BASELINE.saved;
}

int refloat_facade_footpad_state(void) {
    if (!started) {
        return 0;
    }
    return (int) ((const Data *) info.arg)->footpad.state;
}

RefloatSnapshot refloat_facade_snapshot(void) {
    RefloatSnapshot s = {0};
    if (!started) {
        return s;
    }
    const Data *d = (const Data *) info.arg;

    s.state = d->state.state;
    s.mode = d->state.mode;
    s.sat = d->state.sat;
    s.stop_condition = d->state.stop_condition;
    s.darkride = d->state.darkride;
    s.traction_control = d->traction_control;

    s.footpad_state = d->footpad.state;
    s.adc_left = d->footpad.adc_left;
    s.adc_right = d->footpad.adc_right;
    s.fault_adc1 = d->float_conf.fault_adc1;
    s.fault_adc2 = d->float_conf.fault_adc2;

    s.pitch = d->imu.pitch;
    s.balance_pitch = d->imu.balance_pitch;
    s.roll = d->imu.roll;
    s.pitch_rate = d->imu.pitch_rate;

    s.setpoint = d->setpoint;
    s.setpoint_target = d->setpoint_target;
    s.balance_current = d->balance_current.value;

    s.motor_erpm = d->motor.erpm;
    s.motor_duty = d->motor.duty_cycle.value;
    s.motor_current = d->motor.current;

    s.motor_current_max = d->motor.current_max;
    s.motor_current_min = d->motor.current_min;
    s.motor_batt_current_max = d->motor.battery_current_max;
    s.motor_batt_current_min = d->motor.battery_current_min;
    // Refloat вычитает 3 °C из порога прошивки, храним как есть
    s.mosfet_temp_max = d->motor.mosfet_temp_max;
    s.motor_temp_max = d->motor.motor_temp_max;
    s.lv_threshold = d->motor.lv_threshold;
    s.hv_threshold = d->motor.hv_threshold;

    s.imu_frequency = d->imu_freq_tracker.filter_frequency;
    s.main_frequency = d->main_freq_tracker.filter_frequency;
    return s;
}

const char *refloat_facade_state_name(int state) {
    switch (state) {
    case STATE_DISABLED:
        return "DISABLED";
    case STATE_STARTUP:
        return "STARTUP";
    case STATE_READY:
        return "READY";
    case STATE_RUNNING:
        return "RUNNING";
    default:
        return "?";
    }
}

const char *refloat_facade_stop_name(int sc) {
    switch (sc) {
    case STOP_NONE:
        return "NONE";
    case STOP_PITCH:
        return "PITCH";
    case STOP_ROLL:
        return "ROLL";
    case STOP_SWITCH_HALF:
        return "SWITCH_HALF";
    case STOP_SWITCH_FULL:
        return "SWITCH_FULL";
    case STOP_REVERSE_STOP:
        return "REVERSE_STOP";
    case STOP_QUICKSTOP:
        return "QUICKSTOP";
    default:
        return "?";
    }
}

const char *refloat_facade_footpad_name(int fs) {
    switch (fs) {
    case FS_NONE:
        return "NONE";
    case FS_LEFT:
        return "LEFT";
    case FS_RIGHT:
        return "RIGHT";
    case FS_BOTH:
        return "BOTH";
    default:
        return "?";
    }
}

// Реализуется backend-ом платформы: вызов зарегистрированного Refloat set_cfg.
bool floatcore_config_apply(uint8_t *data);

float refloat_facade_config_test_value(void) {
    if (!started) {
        return -1.0f;
    }
    const Data *d = (const Data *) info.arg;
    return d->float_conf.leds.status.brightness_headlights_off;
}

// Пороги отката по напряжению (ТЗ v0.7D.1 §3, §4).
//
// Это конфигурация REFLOAT, а не VESC: она живёт в нашей NVS и меняется тем же
// путём, что и любая другая настройка Refloat — сериализация, разбор самим
// Refloat, применение и запись. Никакого отдельного канала записи не заводится.
//
// Почему понадобилась отдельная функция, а не VESC Tool: на ESP32 транспорта
// протокола VESC нет, Tool сюда не подключается. Единственный доступ к
// конфигурации Refloat — через нас.
//
// Значения задаются НА ЯЧЕЙКУ. Refloat сам умножит их на число ячеек, если
// они меньше 10 (motor_data.c:82-87), поэтому 3.6 и 4.0 — это 36.0 и 40.0 В
// для батареи 10S.
bool refloat_facade_set_voltage_tiltback(float lv_per_cell, float hv_per_cell) {
    if (!started) {
        return false;
    }
    // Значения вне разумного окна не принимаются: опечатка здесь означает
    // неверный порог предупреждения водителю, а это тот случай, когда лучше
    // отказать, чем записать.
    if (!(lv_per_cell > 2.0f && lv_per_cell < 5.0f)) {
        return false;
    }
    if (!(hv_per_cell > 2.0f && hv_per_cell < 5.0f)) {
        return false;
    }
    if (!(hv_per_cell > lv_per_cell)) {
        return false;
    }

    Data *d = (Data *) info.arg;
    d->float_conf.tiltback_lv = lv_per_cell;
    d->float_conf.tiltback_hv = hv_per_cell;

    static uint8_t buffer[512];
    uint32_t written = confparser_serialize_refloatconfig(buffer, &d->float_conf);
    if (written == 0 || written > sizeof(buffer)) {
        return false;
    }
    return floatcore_config_apply(buffer);
}

void refloat_facade_get_voltage_tiltback(float *lv_per_cell, float *hv_per_cell) {
    if (!started) {
        if (lv_per_cell) {
            *lv_per_cell = 0.0f;
        }
        if (hv_per_cell) {
            *hv_per_cell = 0.0f;
        }
        return;
    }
    const Data *d = (const Data *) info.arg;
    if (lv_per_cell) {
        *lv_per_cell = d->float_conf.tiltback_lv;
    }
    if (hv_per_cell) {
        *hv_per_cell = d->float_conf.tiltback_hv;
    }
}

bool refloat_facade_config_save_test(float value) {
    if (!started) {
        return false;
    }
    Data *d = (Data *) info.arg;
    d->float_conf.leds.status.brightness_headlights_off = value;

    // Буфер с запасом: main.c рассчитывает на SERIALIZED_CONFIG_LENGTH = 320 Б.
    static uint8_t buffer[512];
    uint32_t written = confparser_serialize_refloatconfig(buffer, &d->float_conf);
    if (written == 0 || written > sizeof(buffer)) {
        return false;
    }
    // Дальше — путь самого Refloat: десериализация, применение и запись в eeprom.
    return floatcore_config_apply(buffer);
}
