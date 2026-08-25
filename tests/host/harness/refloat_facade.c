// Единственная единица трансляции теста, включающая заголовки Refloat.
// Системные заголовки здесь запрещены (см. refloat_facade.h).

#include "refloat_facade.h"

#include "data.h"
#include "vesc_c_if.h"

// объявлено в mock_vesc_if.h, который нельзя включать вместе с заголовками Refloat
void mock_set_arg_slot(void **slot);

// init() Refloat, переименованный макросом INIT_FUN в нашем shim-заголовке
bool refloat_init(lib_info *info);

static lib_info info;
static bool started = false;

bool refloat_facade_start(void) {
    // ARG раскрывается в *VESC_IF->get_arg(); на VESC это ячейка lib_info.arg,
    // которой владеет загрузчик. Воспроизводим это.
    mock_set_arg_slot(&info.arg);
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
