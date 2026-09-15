#include "fc_motor_permit.h"

#include "fc_build_profile.h"

#include <string.h>

// Имена условий в том же порядке, в каком они проверяются. Порядок не
// случаен: сначала то, без чего остальное не имеет смысла (загрузка, контур,
// датчик), потом машина безопасности, потом транспорт, и только в конце —
// человек. Так первая строка списка отказов обычно и есть корневая причина.
static const char *const NAMES[] = {
    "фаза загрузки не завершена",
    "контур управления нездоров",
    "IMU нездоров",
    "IMU не откалиброван",
    "шина I2C нездорова",
    "Supervisor не в READY",
    "есть залипший отказ",
    "шина CAN не в RUNNING",
    "половина ESC A недоступна",
    "половина ESC B недоступна",
    "поток команд не в норме",
    "realtime-квалификация не пройдена",
    "модель батареи не проверена",
    "политика сторожевого таймера не проверена",
    "параметры мотора не подтверждены на моторе",
    "конфигурация мотора не подтверждена",
    "конфигурация батареи не подтверждена",
    "конфигурация приложения ESC не подтверждена",
    "оператор не разрешил движение",
    "площадки не нажаты",
};

#define NAME_COUNT (sizeof(NAMES) / sizeof(NAMES[0]))

void fc_motor_permit_evaluate(const FcMotorPermitInputs *in, FcMotorPermitResult *out) {
    memset(out, 0, sizeof(*out));

    const bool ok[NAME_COUNT] = {
        in->boot_phase_done,
        in->control_loop_healthy,
        in->imu_healthy,
        in->imu_calibrated,
        in->i2c_healthy,
        in->supervisor_ready,
        in->no_latched_fault,
        in->can_bus_running,
        in->esc_a_healthy,
        in->esc_b_healthy,
        in->command_stream_ok,
        in->realtime_qualified,
        in->battery_model_valid,
        in->watchdog_policy_valid,
        in->motor_model_verified,
        in->motor_config_validated,
        in->battery_config_validated,
        in->app_config_validated,
        in->operator_armed,
        in->footpads_engaged,
    };

    for (uint32_t i = 0; i < NAME_COUNT; ++i) {
        if (!ok[i] && out->reason_count < FC_MOTOR_PERMIT_MAX_REASONS) {
            out->reasons[out->reason_count++] = NAMES[i];
        }
    }
    out->permitted = (out->reason_count == 0);
}

bool fc_motor_permit_available_in_this_build(void) {
    // Не «проверка», а отражение свойства сборки. В профилях LAB_SAFE и
    // ACTIVE_DIAG backend-а вывода нет как кода, поэтому никакая комбинация
    // входов не может дать тягу — и это должно быть видно тесту, а не только
    // читателю комментариев.
    return FC_MOTOR_BACKEND_AVAILABLE != 0;
}

const char *const *fc_motor_permit_condition_names(uint32_t *n) {
    if (n) {
        *n = NAME_COUNT;
    }
    return NAMES;
}
