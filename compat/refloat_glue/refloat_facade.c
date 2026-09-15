// Единственная единица трансляции теста, включающая заголовки Refloat.
// Системные заголовки здесь запрещены (см. refloat_facade.h).

#include "refloat_facade.h"

#include "data.h"
#include "conf/confparser.h"
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
