// Диагностическая консоль FloatCore (ТЗ v0.6A §16, §17).
//
// Полный перечень команд с классификацией — docs/esp32_safety.md, раздел
// «Инвентарь диагностических команд». Здесь та же классификация выражена
// структурой файла:
//
//   SAFE_READONLY   — ничего не меняют. Компилируются всегда.
//   STATE_CHANGING  — меняют состояние системы, но физически безопасны.
//                     Только при FC_LAB_DIAGNOSTICS.
//   LAB_DIAGNOSTICS — намеренно ломают что-то, чтобы доказать, что механизм
//                     безопасности работает. Только при FC_LAB_DIAGNOSTICS.
//   SAFETY_BYPASS   — таких команд не существует.
//   MOTOR_OUTPUT    — таких команд не существует.
//
// В профиле MOTOR_CAPABLE FC_LAB_DIAGNOSTICS равен нулю, поэтому две
// последние категории кода просто не попадают в двоичный файл. Это не флаг,
// который можно переключить: соответствующих функций там нет.

#include "fc_platform.h"

#include "../../../compat/refloat_glue/refloat_facade.h"
#include "../../../compat/safety/fc_build_profile.h"
#include "../../../compat/safety/fc_imu_health.h"
#include "../../../compat/safety/fc_motor_gate.h"
#include "../../../compat/safety/fc_supervisor.h"
#include "../../../compat/imu/fc_imu_pipeline.h"
#include "../drivers/icm20948.h"
#include "fc_rt_clock.h"
#include "../../../compat/safety/fc_flash_policy.h"
#include "../../../compat/diag/fc_i2c_fit.h"
#include "../../../compat/can/fc_vesc_values.h"
#include "fc_imu_source.h"
#include "fc_imu_cal_store.h"
#include "fc_can_bus.h"
#include "../../../compat/config/floatcore_limits.h"
#include "fc_gap_port.h"
#include "fc_limits_sync.h"
#include "../../../compat/config/floatcore_limits.h"
#include "fc_log_port.h"
#include "fc_motor_experiment.h"
#include "../../../compat/diag/fc_gap_trace.h"
#include "../../../compat/diag/fc_shadow.h"
#include "../../../compat/motor/fc_dual_motor.h"
#include "../../../compat/safety/fc_imu_policy.h"
#include "fc_sched.h"
#include "../../../compat/can/fc_vesc_can.h"
#include "../../../compat/vesc_protocol/packet.h"

#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_intr_alloc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <fcntl.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// ============================================================ SAFE_READONLY

static void print_task_table(void);

static void cmd_status(void) {
    RefloatSnapshot s = refloat_facade_snapshot();
    printf("uptime            %.1f s\n", (double) fc_uptime_us() * 1e-6);
    printf("profile           %s\n", FC_PROFILE_NAME);
    printf("reset reason      %s\n", fc_reset_reason_name());
    printf("boot #            %u (счётчик в NVS)\n", (unsigned) fc_boot_count());
    printf("supervisor        %s\n", fc_supervisor_state_name(fc_supervisor_state()));
    printf("refloat state     %s\n", refloat_facade_state_name(s.state));
    printf("stop condition    %s\n", refloat_facade_stop_name(s.stop_condition));
    printf("footpad           %s (adc %.2f / %.2f V)\n",
           refloat_facade_footpad_name(s.footpad_state), (double) s.adc_left,
           (double) s.adc_right);
    printf("pitch / roll      %.2f / %.2f deg (источник %s)\n", (double) s.pitch,
           (double) s.roll, fc_imu_source_name());
    printf("imu / main freq   %.1f / %.1f Hz (по счётчикам Refloat)\n", (double) s.imu_frequency,
           (double) s.main_frequency);
    printf("motor backend     %s\n", fc_motor_gate_backend_name());
    printf("can backend       %s\n", fc_can_backend_name());
}

static void cmd_supervisor(void) {
    FcSupervisorStatus st = fc_supervisor_status();
    uint64_t now = fc_uptime_us();
    printf("state             %s (в этом состоянии %.1f s)\n", fc_supervisor_state_name(st.state),
           (double) (now - st.state_since_us) * 1e-6);
    printf("faults            %s (0x%08" PRIx32 "), latched 0x%08" PRIx32 "\n",
           fc_supervisor_fault_name(st.faults), st.faults, st.faults_latched);
    printf("переходов         %" PRIu32 ", входов в FAULT %" PRIu32 "\n", st.transitions,
           st.fault_entries);
    printf("возраст IMU       худший наблюдавшийся %" PRIu32 " мкс; в момент отказа %" PRIu32
           " мкс (порог %d)\n",
           st.imu_worst_age_us, st.imu_stale_age_us, (int) FC_SUP_IMU_TIMEOUT_US);
    if (st.imu_fault.cause != FC_IMU_FAULT_CAUSE_NONE) {
        const FcImuFaultSnapshot *f = &st.imu_fault;
        printf("снимок отказа IMU (первого, не последнего):\n");
        printf("  причина         %s\n", fc_imu_fault_cause_name(f->cause));
        printf("  здоровье        %s\n",
               fc_imu_health_state_name((FcImuHealthState) f->health_state));
        printf("  политика        %s, причины 0x%02" PRIx32 "\n",
               fc_imu_permit_name((FcImuPermit) f->policy_permit), f->policy_reasons);
        for (int b_ = 0; b_ < FC_IMU_POLICY_REASON_COUNT; ++b_) {
            if (f->policy_reasons & (1u << b_)) {
                printf("    - %s\n",
                       fc_imu_policy_reason_name((FcImuPolicyReason) (1u << b_)));
            }
        }
        printf("  now             %llu мкс\n", (unsigned long long) f->now_us);
        printf("  посл. принятый  %llu мкс, предыдущий %llu\n",
               (unsigned long long) f->time.last_valid_us,
               (unsigned long long) f->time.prev_valid_us);
        printf("  возраст         %" PRIu32 " мкс, зазор %" PRIu32 " мкс\n",
               f->computed_age_us, f->time.last_gap_us);
        printf("  номер семпла    %" PRIu32 ", вердикт опроса %" PRIu32 ", опрос в %llu\n",
               f->time.sequence, f->time.last_verdict,
               (unsigned long long) f->time.last_poll_us);
    } else {
        printf("снимок отказа IMU отказа не было\n");
    }
    printf("motor output      %s\n",
           fc_supervisor_motor_output_permitted() ? "РАЗРЕШЁН" : "запрещён");
    printf("config write      %s\n",
           fc_flash_write_allowed() ? "разрешена" : "запрещена");
    printf("входы:\n");
    printf("  platform_init   %d\n", st.inputs.platform_initialized);
    printf("  config_valid    %d\n", st.inputs.config_valid);
    printf("  loop_alive      %d (последний тик %.1f ms назад)\n", st.inputs.loop_alive,
           st.last_loop_tick_us ? (double) (now - st.last_loop_tick_us) / 1000.0 : -1.0);
    printf("  imu_healthy     %d (последний семпл %.1f ms назад)\n", st.inputs.imu_healthy,
           st.last_imu_sample_us ? (double) (now - st.last_imu_sample_us) / 1000.0 : -1.0);
    printf("  watchdog        %d\n", st.inputs.watchdog_healthy);
    printf("  footpad_engaged %d\n", st.inputs.footpad_engaged);
    printf("  calibration     %d (%s)\n", st.inputs.calibration_valid,
           fc_imu_cal_status_name((FcImuCalStatus) fc_imu_rt_cal_status()));
#if FC_CAN_RX_AVAILABLE
    // Модель здоровья узлов CAN существует с v0.7B, но эти входы Supervisor
    // из неё НАМЕРЕННО не заполняются: подключение «оба VESC живы» к
    // разрешающей логике создало бы второй путь к моторной команде помимо
    // Motor Gate. Отсутствие VESC обязано только запрещать, и запрещает оно
    // уже сейчас — тем, что backend-а вывода нет как кода.
    printf("входы ESC (наблюдение CAN есть, к разрешению не подключено):\n");
#else
    printf("входы под будущие этапы (CAN ещё нет):\n");
#endif
    printf("  esc_a/esc_b     %d / %d\n", st.inputs.esc_a_alive, st.inputs.esc_b_alive);
    printf("  can_fresh       %d\n", st.inputs.can_fresh);
    printf("  battery/thermal %d / %d\n", st.inputs.battery_ok, st.inputs.thermal_ok);
}

static void cmd_imu(void) {
    const icm20948_config_t *cfg = icm20948_active_config();
    icm20948_stats_t st = icm20948_stats();
    FcImuHealthStatus h = fc_imu_health_status();
    FcImuPipelineStats ps = fc_imu_pipeline_stats();
    FcImuSample sm = fc_imu_pipeline_sample();
    RefloatSnapshot rs = refloat_facade_snapshot();
    FcMotorGateStats mg = fc_motor_gate_stats();
    bool real = fc_imu_source() == FC_IMU_SOURCE_REAL;
    uint64_t now = fc_uptime_us();

    printf("IMU source        %s%s\n", fc_imu_source_name(),
           fc_imu_source_available() ? "" : "  (НЕ ПОДНЯТ)");
    printf("драйвер           %s\n", fc_imu_rt_available() ? "ICM-20948 работает" : "не поднят");
    if (!fc_imu_rt_available()) {
        printf("последний шаг     %s\n", icm20948_last_stage());
    }
    printf("инициализация     повторов записи %" PRIu32 ", ожидание после сброса %" PRIu32
           " мс, реинициализаций %llu\n",
           icm20948_init_retries(), icm20948_reset_wait_us() / 1000,
           (unsigned long long) fc_imu_rt_reinits());
    printf("шина              SDA=GPIO%d SCL=GPIO%d, %" PRIu32 " Гц, адрес 0x%02x\n",
           cfg->sda_gpio, cfg->scl_gpio, cfg->i2c_hz, cfg->i2c_addr);
    printf("шкалы             accel ±%.0f g (%.0f LSB/g), gyro ±%.0f °/с (%.1f LSB/(°/с))\n",
           (double) icm20948_accel_fs_g(cfg->accel_fs),
           (double) icm20948_accel_lsb_per_g(cfg->accel_fs),
           (double) icm20948_gyro_fs_dps(cfg->gyro_fs),
           (double) icm20948_gyro_lsb_per_dps(cfg->gyro_fs));
    printf("регистры / контур %.0f Гц обновления регистров (измерено) / %d Гц контур,"
           " startup_done %d\n",
           1125.0, fc_imu_rate_hz(), fc_imu_startup_done());
    printf("транзакции        ok=%llu failed=%llu, средняя %.1f мкс, худшая %" PRIu32 " мкс\n",
           (unsigned long long) st.reads_ok, (unsigned long long) st.reads_failed,
           st.reads_ok ? (double) st.sum_transaction_us / (double) st.reads_ok : 0.0,
           st.max_transaction_us);
    printf("health            %s (семплов %llu, ошибок %llu, stuck %llu, stale %llu, timeout %llu)\n",
           fc_imu_health_state_name(h.state), (unsigned long long) h.samples_total,
           (unsigned long long) h.read_errors, (unsigned long long) h.stuck_events,
           (unsigned long long) h.stale_events, (unsigned long long) h.timeout_events);
    printf("  отвергнуто      %llu: |a| мало %llu, |a| велико %llu, гиро велико %llu"
           " (последний |a| = %.3f g)\n",
           (unsigned long long) h.invalid_samples, (unsigned long long) h.invalid_accel_low,
           (unsigned long long) h.invalid_accel_high, (unsigned long long) h.invalid_gyro_high,
           (double) h.last_invalid_accel_mag);

    // Тракт: сколько опросов во что превратилось. Именно здесь видно
    // соотношение «одна физическая выборка — одна итерация контура».
    printf("тракт             опросов %llu -> принято %llu, дубликатов %llu, отвергнуто %llu,\n",
           (unsigned long long) ps.polls, (unsigned long long) ps.accepted,
           (unsigned long long) ps.duplicates, (unsigned long long) ps.rejected);
    printf("                  сбоев чтения %llu, подозрений на пропуск %llu, дубликатов подряд max %"
           PRIu32 "\n",
           (unsigned long long) ps.read_failures, (unsigned long long) ps.suspected_skips,
           ps.max_consecutive_duplicates);
    printf("остаток смещения  %s: %+.3f %+.3f %+.3f °/с (семплов %" PRIu32 ", перезапусков %"
           PRIu32 ")\n",
           fc_imu_residual_state_name(ps.residual_state), (double) ps.residual_bias_dps[0],
           (double) ps.residual_bias_dps[1], (double) ps.residual_bias_dps[2],
           ps.residual_samples, ps.residual_restarts);
    if (ps.gaps_counted) {
        printf("                  интервал между принятыми: mean %.0f, min %llu, max %llu мкс\n",
               (double) ps.sum_gap_us / (double) ps.gaps_counted,
               (unsigned long long) ps.min_gap_us, (unsigned long long) ps.max_gap_us);
    }

    if (real && fc_imu_pipeline_has_sample()) {
        float mag = sqrtf(sm.accel_g[0] * sm.accel_g[0] + sm.accel_g[1] * sm.accel_g[1] +
                          sm.accel_g[2] * sm.accel_g[2]);
        printf("семпл #%" PRIu32 "        возраст %.1f мс, dt %.0f мкс\n", sm.sample_counter,
               (double) (now - sm.timestamp_us) / 1000.0, (double) sm.dt_s * 1e6);
        float rmag = sqrtf(sm.accel_raw_g[0] * sm.accel_raw_g[0] +
                           sm.accel_raw_g[1] * sm.accel_raw_g[1] +
                           sm.accel_raw_g[2] * sm.accel_raw_g[2]);
        printf("  raw accel       %+7.3f %+7.3f %+7.3f g  |a|=%.3f\n", (double) sm.accel_raw_g[0],
               (double) sm.accel_raw_g[1], (double) sm.accel_raw_g[2], (double) rmag);
        printf("  raw gyro        %+8.2f %+8.2f %+8.2f °/с\n", (double) sm.gyro_raw_dps[0],
               (double) sm.gyro_raw_dps[1], (double) sm.gyro_raw_dps[2]);
        printf("  cal accel       %+7.3f %+7.3f %+7.3f g  |a|=%.3f\n", (double) sm.accel_g[0],
               (double) sm.accel_g[1], (double) sm.accel_g[2], (double) mag);
        printf("  cal gyro        %+8.2f %+8.2f %+8.2f °/с   (%+7.4f %+7.4f %+7.4f рад/с)\n",
               (double) sm.gyro_dps[0], (double) sm.gyro_dps[1], (double) sm.gyro_dps[2],
               (double) sm.gyro_rad_s[0], (double) sm.gyro_rad_s[1], (double) sm.gyro_rad_s[2]);
        printf("  температура     %.1f °C\n", (double) sm.temperature_c);
        printf("  AHRS платформы  pitch %+7.3f  roll %+7.3f  yaw %+8.3f град\n",
               (double) (sm.pitch_rad * 57.29578f), (double) (sm.roll_rad * 57.29578f),
               (double) (sm.yaw_rad * 57.29578f));
    } else if (!real) {
        printf("семпл             mock: покой, acc (0,0,+1) g, gyro 0\n");
    } else {
        printf("семпл             ЕЩЁ НЕ ПОЛУЧЕН\n");
    }

    // То, что из этих данных сделал сам Refloat, и куда это ушло.
    printf("Refloat           state %s, pitch %+7.3f, balance_pitch %+7.3f, roll %+7.3f град\n",
           refloat_facade_state_name(rs.state), (double) rs.pitch, (double) rs.balance_pitch,
           (double) rs.roll);
    printf("                  pitch_rate %+8.3f, setpoint %+6.3f, imu %.1f Гц, main %.1f Гц\n",
           (double) rs.pitch_rate, (double) rs.setpoint, (double) rs.imu_frequency,
           (double) rs.main_frequency);
    // Два разных числа, и их легко перепутать.
    //
    // balance_current — внутреннее сглаженное значение PID. В состоянии READY
    // Refloat его не обновляет, поэтому там оно ПРОСРОЧЕНО и показывает
    // последнее значение из RUNNING. Правда о том, что реально запрошено —
    // последнее значение, дошедшее до Motor Gate.
    printf("запрошенный ток   %+8.3f A (PID balance_current%s)\n", (double) rs.balance_current,
           rs.state == 3 ? "" : ", НЕ обновляется вне RUNNING");
    printf("последний запрос  %+8.3f A через Motor Gate (%llu запросов set_current)\n",
           (double) mg.last_value[FC_MOTOR_REQ_CURRENT],
           (unsigned long long) mg.by_kind[FC_MOTOR_REQ_CURRENT]);
    printf("supervisor        %s\n", fc_supervisor_state_name(fc_supervisor_state()));
    printf("Motor Gate        requested=%llu allowed=%llu sent=%llu backend=%s\n",
           (unsigned long long) mg.requests_total, (unsigned long long) mg.allowed_by_policy,
           (unsigned long long) mg.physically_sent, fc_motor_gate_backend_name());
    printf("оси               СЫРЫЕ, в системе координат датчика: преобразование к осям доски\n");
    printf("                  тождественно (docs/imu_orientation_mapping.md §8)\n");
}

// Скан шины I2C. SAFE_READONLY, но формулировка требует точности: команда
// опрашивает адреса и читает WHO_AM_I, а чтобы прочитать его, выбирает нулевой
// банк записью в BANK_SEL. Это переключение окна регистров, а не изменение
// конфигурации: тот же select_bank драйвер делает при каждом обычном чтении, и
// кэш банка после опроса инвалидируется. Ни один параметр датчика команда не
// меняет и к мотору отношения не имеет. Если шина ещё не поднята (датчик не
// инициализировался), команда поднимает её — именно для этого случая она и
// нужна.
static void cmd_i2cscan(void) {
    if (fc_imu_rt_running()) {
        printf("i2cscan: штатная задача чтения активна — у драйвера один читатель.\n");
        printf("  остановите её: imu-stop-confirm, затем повторите скан\n");
        return;
    }
    icm20948_config_t cfg = icm20948_default_config();
    uint8_t found[16];
    size_t n = 0;
    esp_err_t err = icm20948_scan(&cfg, found, sizeof(found), &n);
    if (err != ESP_OK) {
        printf("i2cscan: шина не поднялась: %s\n", esp_err_to_name(err));
        printf("  это уже не про датчик: проверьте GPIO%d/GPIO%d и питание\n", cfg.sda_gpio,
               cfg.scl_gpio);
        return;
    }
    printf("i2cscan: SDA=GPIO%d SCL=GPIO%d, %" PRIu32 " Гц, диапазон 0x08…0x77\n", cfg.sda_gpio,
           cfg.scl_gpio, cfg.i2c_hz);
    if (!n) {
        printf("  НИ ОДНОГО устройства не ответило\n");
        printf("  вероятные причины: нет питания на модуле, перепутаны SDA/SCL,\n");
        printf("  обрыв в жгуте, отсутствуют подтяжки на линиях\n");
        return;
    }
    printf("  ответили: %u\n", (unsigned) n);
    for (size_t i = 0; i < n; ++i) {
        uint8_t who = 0;
        esp_err_t we = icm20948_probe_addr(&cfg, found[i], &who);
        const char *verdict = "неизвестное устройство";
        if (we != ESP_OK) {
            verdict = "WHO_AM_I не прочитался";
        } else if (who == ICM20948_WHO_AM_I_VALUE) {
            verdict = "ICM-20948 (WHO_AM_I совпал)";
        }
        printf("    0x%02x  WHO_AM_I=0x%02x  %s\n", found[i], who, verdict);
        if (we == ESP_OK && who == ICM20948_WHO_AM_I_VALUE && found[i] != cfg.i2c_addr) {
            printf("      ВНИМАНИЕ: драйвер настроен на 0x%02x. Датчик на 0x%02x означает,\n",
                   cfg.i2c_addr, found[i]);
            printf("      что вывод AD0 подтянут к питанию, а не к земле\n");
        }
    }
}

// Показать текущую калибровку. SAFE_READONLY: только чтение состояния.
static void cmd_imu_cal_show(void) {
    FcImuCalibration c = fc_imu_pipeline_calibration();
    FcDetectStatus d = fc_imu_detect_status();
    printf("статус            %s\n", fc_imu_cal_status_name((FcImuCalStatus) fc_imu_rt_cal_status()));
    printf("поворот           roll %+8.3f  pitch %+8.3f  yaw %+8.3f град\n",
           (double) c.rot_roll_deg, (double) c.rot_pitch_deg, (double) c.rot_yaw_deg);
    printf("смещения gyro     %+8.3f %+8.3f %+8.3f °/с (в повёрнутой системе)\n",
           (double) c.gyro_offset_dps[0], (double) c.gyro_offset_dps[1],
           (double) c.gyro_offset_dps[2]);
    printf("смещения accel    %+8.3f %+8.3f %+8.3f g (на v0.6E не измеряются)\n",
           (double) c.accel_offset_g[0], (double) c.accel_offset_g[1],
           (double) c.accel_offset_g[2]);
    printf("порядок           поворот -> вычитание смещений -> AHRS и Refloat\n");
    printf("измерение         %s", fc_imu_detect_state_name(d.state));
    if (d.state == FC_DETECT_COLLECTING) {
        printf(", накоплено %" PRIu32 " из %" PRIu32 ", сбросов %" PRIu32, d.samples, d.needed,
               d.restarts);
    }
    if (d.failure != FC_DETECT_FAIL_NONE) {
        printf(", последняя помеха: %s", fc_imu_detect_failure_name(d.failure));
    }
    printf("\n");
    if (d.state == FC_DETECT_DONE) {
        printf("найдено (НЕ сохранено): roll %+8.3f  pitch %+8.3f  yaw %+8.3f град\n",
               (double) d.result.rot_roll_deg, (double) d.result.rot_pitch_deg,
               (double) d.result.rot_yaw_deg);
        printf("                        смещения %+8.3f %+8.3f %+8.3f °/с\n",
               (double) d.result.gyro_offset_dps[0], (double) d.result.gyro_offset_dps[1],
               (double) d.result.gyro_offset_dps[2]);
        printf("качество                СКО gyro %.3f %.3f %.3f °/с, accel %.4f %.4f %.4f g\n",
               (double) d.gyro_std_dps[0], (double) d.gyro_std_dps[1], (double) d.gyro_std_dps[2],
               (double) d.accel_std_g[0], (double) d.accel_std_g[1], (double) d.accel_std_g[2]);
        printf("                        |a| = %.4f g\n", (double) d.accel_mag_g);
        printf("сохранить:              imu-cal-save\n");
    }
}

#if FC_CAN_RX_AVAILABLE
// Сводка по шине CAN. SAFE_READONLY: только чтение счётчиков.
//
// Каждый кадр в UART намеренно НЕ печатается: при 100 кадрах/с это сломало бы
// тайминг вывода и ничего бы не дало. Здесь агрегаты, а последние кадры — в
// отдельной команде can-frames.
static void cmd_can(void) {
    FcCanStats s = fc_can_bus_stats();
    uint64_t now = fc_uptime_us();
    double secs = s.started_us ? (double) (now - s.started_us) * 1e-6 : 0.0;

    printf("профиль CAN       %s\n", FC_CAN_PROFILE_NAME);
    printf("режим TWAI        %s\n", fc_can_bus_mode_name());
    printf("транспорт мотору  %s\n", fc_can_backend_name());
    printf("контроллер        %s, приём %s\n", fc_can_bus_state_name(s.bus_state),
           fc_can_bus_running() ? "запущен" : "не запущен");
    printf("кадров            всего %llu за %.1f с (%.1f/с)\n",
           (unsigned long long) s.frames_total, secs,
           secs > 0 ? (double) s.frames_total / secs : 0.0);
    printf("                  стандартных %llu, расширенных %llu, RTR %llu\n",
           (unsigned long long) s.frames_std, (unsigned long long) s.frames_ext,
           (unsigned long long) s.frames_rtr);
    printf("очередь/ошибки    в очереди %" PRIu32 ", потеряно %" PRIu32 ", переполнений %" PRIu32
           "\n", s.msgs_to_rx, s.rx_missed, s.rx_overrun);
    printf("                  ошибок шины %" PRIu32 ", потерь арбитража %" PRIu32
           ", неудач передачи %" PRIu32 ", ошибок приёма %llu\n",
           s.bus_error_count, s.arb_lost_count, s.tx_failed_count,
           (unsigned long long) s.receive_errors);
    printf("                  счётчик ошибок TX %" PRIu32 ", RX %" PRIu32 ", BUS_OFF %" PRIu32
           ", восстановлений %" PRIu32 "\n",
           s.tx_error_counter, s.rx_error_counter, s.bus_off_count, s.recoveries);
    if (s.last_frame_us) {
        printf("последний кадр    %.1f мс назад\n", (double) (now - s.last_frame_us) / 1000.0);
    } else {
        printf("последний кадр    НЕ ПОЛУЧЕН НИ ОДНОГО\n");
    }

    printf("DLC:             ");
    for (int i = 0; i <= 8; ++i) {
        if (s.dlc_hist[i]) {
            printf(" %d:%llu", i, (unsigned long long) s.dlc_hist[i]);
        }
    }
    printf("\n");

    printf("идентификаторы (%" PRIu32 "%s):\n", s.id_count,
           s.ids_overflow ? ", таблица переполнена" : "");
    for (uint32_t i = 0; i < s.id_count; ++i) {
        const FcCanIdStat *d = &s.ids[i];
        FcVescCanId v = fc_vesc_can_decode(d->id, d->extended);
        printf("  0x%08" PRIx32 " %s n=%-7llu dlc=%" PRIu32 " %6.1f/с  ", d->id,
               d->extended ? "ext" : "std", (unsigned long long) d->count, d->last_dlc,
               secs > 0 ? (double) d->count / secs : 0.0);
        if (!v.vesc_format) {
            printf("НЕ формат VESC");
        } else if (v.known_type) {
            printf("VESC id=%-3u %-24s%s", v.controller_id, v.name,
                   v.is_motor_command ? " <-- КОМАНДА МОТОРУ" : "");
        } else {
            printf("VESC id=%-3u тип %-3" PRIu32 " UNKNOWN        ", v.controller_id,
                   v.packet_type);
        }
        printf("  ");
        for (uint32_t b = 0; b < d->last_dlc && b < 8; ++b) {
            printf("%02x ", d->last_data[b]);
        }
        printf("\n");
    }
}

// Здоровье узлов. Наблюдение: ни одно значение отсюда никуда не подключено.
// Состояние отложенного журнала (ТЗ v0.7C §6).
static void cmd_log(void) {
    FcLogPortStats s = fc_log_port_stats();
    printf("журнал            фаза загрузки %s",
           fc_boot_phase_done() ? "закрыта" : "ИДЁТ");
    if (fc_boot_phase_done()) {
        printf(" на %.2f с", (double) fc_boot_phase_end_us() / 1e6);
    }
    printf("\n");
    printf("записей           принято %" PRIu32 ", напечатано %" PRIu32 ", в очереди %" PRIu32
           "\n", s.emitted, s.printed, s.pending);
    printf("потери            отброшено %" PRIu32 ", обрезано %" PRIu32 "\n", s.dropped,
           s.truncated);
    printf("кольцо            пик заполнения %" PRIu32 " из %d\n", s.high_water, FC_LOG_SLOTS);
    printf("стоимость вызова  худшая %" PRIu32 " мкс, тег %s (UART той же строкой — тысячи)\n",
           s.max_format_us, s.max_format_tag[0] ? s.max_format_tag : "-");
    if (s.dropped) {
        printf("ВНИМАНИЕ: записи терялись — кольцо мало или сливальщик не успевает\n");
    }
}

static void cmd_can_health(void) {
    FcCanHealth snap = fc_can_bus_health();
    const FcCanHealth *h = &snap;
    uint64_t now = fc_uptime_us();
    printf("узлы CAN (порог молчания %.0f мс):\n", (double) h->stale_us / 1000.0);
    for (uint32_t i = 0; i < h->count; ++i) {
        const FcCanNode *n = &h->nodes[i];
        printf("  id=%-3u %-10s %-9s статусов %-8" PRIu32 " ответов %-4" PRIu32
               " таймаутов %-3" PRIu32 " падений %-3" PRIu32 " молчит ",
               n->id, n->expected ? "ожидаемый" : "посторонний",
               n->healthy ? "доступен" : "НЕДОСТУПЕН", n->statuses, n->diag_responses,
               n->diag_timeouts, n->health_drops);
        if (n->ever_seen) {
            // Задача приёма обновляет отметку параллельно, поэтому она может
            // оказаться свежее now: беззнаковая разность дала бы астрономическое
            // число вместо нуля.
            uint64_t age = now > n->last_seen_us ? now - n->last_seen_us : 0;
            printf("%.0f мс\n", (double) age / 1000.0);
        } else {
            printf("никогда не отвечал\n");
        }
    }
    printf("все ожидаемые доступны: %s\n",
           fc_can_health_all_expected_healthy(h) ? "да" : "НЕТ");
    printf("ВАЖНО: это наблюдение. Motor Gate этих значений не видит.\n");
}

#if FC_CAN_DIAG_TX_AVAILABLE
static void print_diag_stats(void) {
    FcCanDiagStats d = fc_can_bus_diag_stats();
    printf("диагностика TX    запросов %llu, ответов %llu, таймаутов %llu\n",
           (unsigned long long) d.requests, (unsigned long long) d.responses,
           (unsigned long long) d.timeouts);
    printf("                  отказов передачи %llu, ошибок CRC %llu, отклонено списком %llu,"
           " придержано %llu\n",
           (unsigned long long) d.tx_failures, (unsigned long long) d.crc_errors,
           (unsigned long long) d.build_rejected, (unsigned long long) d.throttled);
    printf("                  RTT последний %" PRIu32 " мкс, максимум %" PRIu32 " мкс\n",
           d.last_rtt_us, d.max_rtt_us);
}

static void print_fw_payload(const uint8_t *p, uint16_t n) {
    // Раскладка из bldc release_6_06, comm/commands.c:231-252.
    if (n < 4) {
        return;
    }
    printf("  версия прошивки  %u.%u\n", p[1], p[2]);
    printf("  железо           %s\n", (const char *) (p + 3));
    uint16_t i = (uint16_t) (3 + strlen((const char *) (p + 3)) + 1);
    if (i + 12 <= n) {
        printf("  UUID             ");
        for (int b = 0; b < 12; ++b) {
            printf("%02x", p[i + b]);
        }
        printf("\n");
    }
}

static bool parse_diag_target(const char *arg, uint8_t *id_out) {
    char *end = NULL;
    long v = strtol(arg, &end, 10);
    if (end == arg || v < 0 || v > 255) {
        printf("нужен номер контроллера 0…254\n");
        return false;
    }
    if (v == 255) {
        printf("широковещательный адрес 255 запрещён: ответят обе половины сразу\n");
        return false;
    }
    *id_out = (uint8_t) v;
    return true;
}

// Один read-only запрос. SAFE_READONLY: белый список не даёт собрать ничего,
// что меняло бы состояние ESC (см. compat/can/fc_can_diag.h).
static void cmd_can_diag(const char *args, bool hex) {
    FcCanDiagRequest req;
    const char *rest;
    if (!strncmp(args, "ping ", 5)) {
        req = FC_CAN_DIAG_PING;
        rest = args + 5;
    } else if (!strncmp(args, "fw ", 3)) {
        req = FC_CAN_DIAG_FW_VERSION;
        rest = args + 3;
    } else if (!strncmp(args, "values ", 7)) {
        req = FC_CAN_DIAG_VALUES;
        rest = args + 7;
    } else if (!strncmp(args, "mcconf ", 7)) {
        req = FC_CAN_DIAG_MCCONF;
        rest = args + 7;
    } else if (!strncmp(args, "appconf ", 8)) {
        req = FC_CAN_DIAG_APPCONF;
        rest = args + 8;
    } else {
        printf("can-diag <ping|fw|values|mcconf|appconf> <id>\n");
        return;
    }

    uint8_t target;
    if (!parse_diag_target(rest, &target)) {
        return;
    }

    static uint8_t buf[FC_CAN_DIAG_RX_MAX];
    uint16_t n = 0;
    uint8_t comm = fc_can_diag_request_comm_id(req);
    printf("запрос %s -> контроллер %u (тип пакета %" PRIu32 ", ", fc_can_diag_request_name(req),
           target, fc_can_diag_request_packet_type(req));
    if (comm == 0xFFu) {
        printf("COMM-уровень не используется)\n");
    } else {
        printf("COMM %u = %s)\n", comm, fc_vesc_comm_name(comm));
    }

    if (!fc_can_bus_diag_request(req, target, 500, buf, sizeof(buf), &n)) {
        printf("ОТВЕТА НЕТ\n");
        print_diag_stats();
        return;
    }

    FcCanDiagStats d = fc_can_bus_diag_stats();
    printf("ответ %u байт за %" PRIu32 " мкс, CRC16 %04x\n", n, d.last_rtt_us,
           vesc_crc16(buf, n));

    if (req == FC_CAN_DIAG_PING) {
        printf("  ответил id=%u, тип железа %u\n", buf[0], n > 1 ? buf[1] : 0);
    } else if (req == FC_CAN_DIAG_FW_VERSION) {
        print_fw_payload(buf, n);
    }

    if (hex) {
        for (uint16_t i = 0; i < n; ++i) {
            if (i % 32 == 0) {
                printf("\n  %03u  ", i);
            }
            printf("%02x", buf[i]);
        }
        printf("\n");
    }
}
#endif  // FC_CAN_DIAG_TX_AVAILABLE

// ------------------------------------------- телеметрия половин (ТЗ v0.9J §9, §11)
//
// Только чтение: COMM_GET_VALUES и COMM_GET_APPCONF из белого списка
// диагностики. Между запросами выдерживается пауза: у диагностики ограничитель
// частоты 200 мс, и запросы подряд на v0.9E упирались в него.
#if FC_CAN_DIAG_TX_AVAILABLE
typedef struct {
    bool values_ok;
    FcVescValues v;
    bool timeout_ok;
    FcVescAppTimeout t;
} HalfTelemetry;

static bool diag_with_retry(FcCanDiagRequest req, uint8_t id, uint8_t *buf, uint16_t cap,
                            uint16_t *n) {
    for (int attempt = 0; attempt < 4; ++attempt) {
        if (fc_can_bus_diag_request(req, id, 500, buf, cap, n)) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    return false;
}

static HalfTelemetry read_half(uint8_t id, bool with_timeout) {
    static uint8_t buf[FC_CAN_DIAG_RX_MAX];
    HalfTelemetry h;
    memset(&h, 0, sizeof(h));
    uint16_t n = 0;
    if (diag_with_retry(FC_CAN_DIAG_VALUES, id, buf, sizeof(buf), &n)) {
        h.v = fc_vesc_values_decode(buf, n);
        h.values_ok = h.v.valid;
    }
    if (with_timeout) {
        vTaskDelay(pdMS_TO_TICKS(250));
        if (diag_with_retry(FC_CAN_DIAG_APPCONF, id, buf, sizeof(buf), &n)) {
            h.t = fc_vesc_appconf_timeout(buf, n, id);
            h.timeout_ok = h.t.valid;
        }
    }
    return h;
}

static void print_half(uint8_t id, const HalfTelemetry *h) {
    if (!h->values_ok) {
        printf("  %u: телеметрия НЕ получена\n", id);
    } else {
        printf("  %u: %.1f В  ток %+.2f А  ERPM %+ld  скважность %+.3f  тахометр %ld  "
               "отказ %s\n", id, (double) h->v.v_in, (double) h->v.current_motor_a,
               (long) h->v.erpm, (double) h->v.duty, (long) h->v.tachometer,
               h->v.has_fault ? fc_vesc_fault_name(h->v.fault_code) : "?");
    }
    if (h->timeout_ok) {
        printf("       таймаут команды %lu мс, тормозной ток по таймауту %.2f А%s\n",
               (unsigned long) h->t.timeout_ms, (double) h->t.timeout_brake_current_a,
               h->t.timeout_brake_current_a == 0.0f ? " (выбег)" : "");
    }
}

static void cmd_vesc_values(void) {
    printf("телеметрия половин (только чтение):\n");
    HalfTelemetry a = read_half(118, true);
    vTaskDelay(pdMS_TO_TICKS(250));
    HalfTelemetry b = read_half(100, true);
    print_half(118, &a);
    print_half(100, &b);
}
#endif

// Низкотемповая диагностическая сессия (ТЗ v0.7B §10, §20 шаг 10).
//
// Ни одного печатного символа внутри сессии: измерять тайминг и одновременно
// печатать в UART бессмысленно — известно, что печать сама даёт опоздание.
// Сводка выводится один раз в конце.
//
// Темп задан жёстко и не настраивается: один запрос в секунду. Периодический
// STATUS остаётся основным источником телеметрии, диагностика — редкое
// событие.
#if FC_CAN_DIAG_TX_AVAILABLE
static void cmd_can_diag_poll(const char *arg) {
    char *end = NULL;
    long secs = strtol(arg, &end, 10);
    if (end == arg || secs < 1 || secs > 600) {
        printf("can-diag-poll <секунды 1…600>\n");
        return;
    }

    static const struct {
        FcCanDiagRequest req;
        uint8_t target;
    } ROTATION[] = {
        {FC_CAN_DIAG_PING, 118},
        {FC_CAN_DIAG_PING, 100},
        {FC_CAN_DIAG_FW_VERSION, 118},
        {FC_CAN_DIAG_FW_VERSION, 100},
    };
    const int n_rot = (int) (sizeof(ROTATION) / sizeof(ROTATION[0]));

    printf("сессия %ld с, 1 запрос/с, вывод только в конце\n", secs);

    uint32_t ok = 0, fail = 0;
    for (long i = 0; i < secs; ++i) {
        int k = (int) (i % n_rot);
        if (fc_can_bus_diag_request(ROTATION[k].req, ROTATION[k].target, 500, NULL, 0, NULL)) {
            ++ok;
        } else {
            ++fail;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    printf("сессия завершена: успешно %" PRIu32 ", без ответа %" PRIu32 "\n", ok, fail);
    print_diag_stats();
}
#endif

static void cmd_can_frames(void) {
    static FcCanFrame f[FC_CAN_RING];
    uint32_t n = fc_can_bus_ring(f, FC_CAN_RING);
    printf("последние %" PRIu32 " кадров (новейший внизу):\n", n);
    for (uint32_t i = 0; i < n; ++i) {
        printf("  %10llu us  0x%08" PRIx32 " %s%s dlc=%u ", (unsigned long long) f[i].t_us,
               f[i].id, f[i].extended ? "ext" : "std", f[i].rtr ? " RTR" : "", f[i].dlc);
        for (uint32_t b = 0; b < f[i].dlc; ++b) {
            printf("%02x ", f[i].data[b]);
        }
        printf("\n");
    }
}
#endif

// Качество пробуждения потоков Refloat (ТЗ v0.7D §11).
static void cmd_sched(void) {
    printf("точность пробуждения потоков Refloat:\n");
    for (size_t i = 0; i < fc_thread_count() && i < FC_SCHED_MAX_THREADS; ++i) {
        FcSchedStats s = fc_sched_stats(i);
        if (!s.iterations) {
            continue;
        }
        printf("  %-14s n=%-8llu просил %6.1f мкс, спал %6.1f мкс, задержка mean %5.1f "
               "min %" PRIu32 " max %" PRIu32 "\n",
               fc_thread_name(i), (unsigned long long) s.iterations,
               (double) s.requested_sum_us / (double) s.iterations,
               (double) s.actual_sum_us / (double) s.iterations,
               (double) s.latency_sum_us / (double) s.iterations, s.latency_min_us,
               s.latency_max_us);
        printf("  %-14s гистограмма задержки, мкс:", "");
        for (int b = 0; b < 10; ++b) {
            if (s.latency_hist[b]) {
                if (b == 9) {
                    printf("  900+:%" PRIu32, s.latency_hist[b]);
                } else {
                    printf("  %d-%d:%" PRIu32, b * 100, b * 100 + 99, s.latency_hist[b]);
                }
            }
        }
        printf("\n");
    }
    printf("задержка пробуждения — это НЕ время исполнения: поток в этот момент спит\n");
}

static void cmd_tasks(void) {
    printf("задачи FloatCore:\n");
    // Размер берётся из того же определения, что и создание задачи: зашитое
    // число здесь однажды уже разошлось с действительностью (ТЗ v0.9H §3).
    {
        unsigned free_b = (unsigned) fc_imu_rt_stack_watermark();
        printf("  %-14s свободно минимум %u B из %u (%.0f %%) %s\n", "fc_imu_rt", free_b,
               (unsigned) FC_IMU_RT_STACK_BYTES,
               100.0 * (double) free_b / (double) FC_IMU_RT_STACK_BYTES,
               free_b < FC_IMU_RT_STACK_WARN_BYTES ? "<- ЗАПАС ИСЧЕРПАН" : "");
    }
    printf("  %-14s свободно минимум %u B из 4096 (supervisor)\n", "fc_super",
           (unsigned) fc_supervisor_stack_watermark());
    printf("  %-14s свободно минимум %u B из 3072 (хранилище)\n", "fc_nvs",
           (unsigned) fc_storage_stack_watermark());
    for (size_t i = 0; i < fc_thread_count(); ++i) {
        printf("  %-14s свободно минимум %u B из 12288 (задача Refloat)\n", fc_thread_name(i),
               (unsigned) fc_thread_stack_watermark(i));

        {
            uint64_t age = 0;
            const char *st = fc_thread_stage(i, &age);
            printf("  %-14s в вызове %s, %llu мкс назад%s\n", "", st ? st : "?",
                   (unsigned long long) age,
                   age > 1000000ull ? "   <- ИЗ ВЫЗОВА НЕ ВЕРНУЛСЯ" : "");
            printf("  %-14s период сна защёлкнут на %" PRIu32 " тиков, отметка пробуждения %" PRIu32
                   ", тик сейчас %" PRIu32 "\n",
                   "", fc_thread_period_ticks(i), fc_thread_last_wake(i),
                   (uint32_t) xTaskGetTickCount());
        }    }
    // Без vTaskList (ТЗ v0.9I §19): измерено, что десять её вызовов дают пять
    // пропущенных периодов контура с провалами до 6.8 мс. См. print_task_table.
    print_task_table();
}

static void print_timing(FcTimingChannel ch) {
    FcTimingStats t = fc_timing_get(ch);
    if (t.iterations == 0 && t.exec_samples) {
        // Канал без отметок периода: он меряет не цикл задачи, а отдельный
        // участок внутри неё (например, транзакцию I²C). Пропускать такой
        // канал нельзя — именно в нём и оказалась неучтённая работа.
        printf("  %-26s участок внутри задачи, n=%llu\n", t.name,
               (unsigned long long) t.exec_samples);
        printf("      исполнение mean %6.1f  p99 %6" PRIu32 "  min %6" PRIu32 "  max %6" PRIu32
               " us  (НАСТЕННОЕ: вытеснение включено)\n",
               (double) t.exec_sum_us / (double) t.exec_samples, t.exec_p99_us, t.exec_min_us,
               t.exec_max_us);
        printf("      вытеснение mean %6.1f  max %6" PRIu32 " us   собственное CPU mean %6.1f  "
               "max %6" PRIu32 " us\n",
               (double) t.preempt_sum_us / (double) t.exec_samples, t.preempt_max_us,
               (double) t.net_sum_us / (double) t.exec_samples, t.net_max_us);
        return;
    }
    if (t.iterations == 0) {
        printf("  %-26s нет итераций\n", t.name);
        return;
    }
    double mean = (double) t.sum_period_us / (double) t.iterations;
    printf("  %-26s n=%llu номинал %" PRIu32 " us\n", t.name, (unsigned long long) t.iterations,
           t.nominal_period_us);
    printf("      период  mean %8.1f  p50 %6" PRIu32 "  p95 %6" PRIu32 "  p99 %6" PRIu32
           "  p99.9 %6" PRIu32 "  min %6" PRIu32 "  max %6" PRIu32 "\n",
           mean, t.p50_us, t.p95_us, t.p99_us, t.p999_us, t.min_period_us, t.max_period_us);
    printf("      дедлайны late %" PRIu32 " (%.2f %%)  missed %" PRIu32 "  вне гистограммы %"
           PRIu32 "\n",
           t.late, 100.0 * (double) t.late / (double) t.iterations, t.missed, t.overflow);
    if (t.exec_samples) {
        printf("      исполнение mean %6.1f  p99 %6" PRIu32 "  min %6" PRIu32 "  max %6" PRIu32
               " us  (НАСТЕННОЕ: вытеснение включено)\n",
               (double) t.exec_sum_us / (double) t.exec_samples, t.exec_p99_us, t.exec_min_us,
               t.exec_max_us);
        printf("      вытеснение mean %6.1f  max %6" PRIu32 " us   собственное CPU mean %6.1f  "
               "max %6" PRIu32 " us\n",
               (double) t.preempt_sum_us / (double) t.exec_samples, t.preempt_max_us,
               (double) t.net_sum_us / (double) t.exec_samples, t.net_max_us);
    }
}

static void print_timing_table(void) {
    printf("периодичность (esp_timer, отметка в момент пробуждения задачи):\n");
    for (int i = 0; i < FC_TIMING_COUNT; ++i) {
        print_timing((FcTimingChannel) i);
    }
}

static void cmd_timing(void) {
    // Две фазы печатаются раздельно (ТЗ v0.7C §7). Загрузка печатает в UART
    // килобайты, и опоздания контура в этот момент неизбежны и ожидаемы.
    // Смешивать их с установившимся режимом нельзя: один пропуск со старта
    // иначе навсегда осел бы в цифрах и маскировал бы настоящие.
    if (fc_timing_is_steady()) {
        printf("\nфаза загрузки (счётчики закрыты на %.2f с, печать баннера внутри окна):\n",
               (double) fc_boot_phase_end_us() / 1e6);
        for (int i = 0; i < FC_TIMING_COUNT; ++i) {
            FcTimingStats t = fc_timing_get_boot((FcTimingChannel) i);
            if (!t.iterations) {
                continue;
            }
            printf("  %-26s n=%-7llu late %" PRIu32 " missed %" PRIu32 " max %" PRIu32 "\n",
                   t.name, (unsigned long long) t.iterations, t.late, t.missed, t.max_period_us);
        }
        printf("\nустановившийся режим:\n");
    } else {
        printf("\nфаза загрузки ЕЩЁ ИДЁТ — цифры ниже включают печать баннера:\n");
    }
    print_timing_table();
}


static void cmd_timing_hist(void) {
    static uint32_t bins[FC_TIMING_BINS + 1];
    uint32_t width = 0;
    uint32_t n = fc_timing_histogram(FC_TIMING_CONTROL, bins, FC_TIMING_BINS + 1, &width);
    FcTimingStats t = fc_timing_get(FC_TIMING_CONTROL);
    printf("гистограмма периодов контура, ширина корзины %" PRIu32 " мкс, всего %llu\n", width,
           (unsigned long long) t.iterations);
    uint32_t peak = 1;
    for (uint32_t i = 0; i < n; ++i) {
        if (bins[i] > peak) {
            peak = bins[i];
        }
    }
    for (uint32_t i = 0; i < n; ++i) {
        if (!bins[i]) {
            continue;
        }
        int len = (int) ((uint64_t) bins[i] * 50 / peak);
        printf("  %6" PRIu32 "…%6" PRIu32 " us  %8" PRIu32 " ", i * width, (i + 1) * width,
               bins[i]);
        for (int k = 0; k < len; ++k) {
            putchar('#');
        }
        printf("\n");
    }
}

static void cmd_heap(void) {
    printf("free heap         %u B\n", (unsigned) esp_get_free_heap_size());
    printf("min free heap     %u B\n", (unsigned) esp_get_minimum_free_heap_size());
    printf("largest block     %u B\n",
           (unsigned) heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    printf("internal free     %u B\n", (unsigned) heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
}

static void cmd_config(void) {
    RefloatSnapshot s = refloat_facade_snapshot();
    FcStorageStats st = fc_storage_stats();
    printf("custom config     %s\n", fc_config_registered() ? "зарегистрирован" : "НЕТ");
    printf("storage слов      %d (NVS)\n", fc_storage_capacity());
    printf("запись сейчас     %s (supervisor %s)\n",
           fc_flash_write_allowed() ? "разрешена" : "ЗАПРЕЩЕНА",
           fc_supervisor_state_name(fc_supervisor_state()));
    printf("статистика        принято=%llu отклонено=%llu коммитов=%llu (ошибок %llu)\n",
           (unsigned long long) st.writes_accepted, (unsigned long long) st.writes_rejected,
           (unsigned long long) st.commits_done, (unsigned long long) st.commits_failed);
    printf("длительность      последний коммит %" PRIu32 " мкс, худший %" PRIu32 " мкс\n",
           st.last_commit_us, st.max_commit_us);
    printf("fault_adc1/2      %.2f / %.2f V\n", (double) s.fault_adc1, (double) s.fault_adc2);
    printf("test param        %.3f (leds.status.brightness_headlights_off)\n",
           (double) refloat_facade_config_test_value());
}

static void cmd_safety(void) {
    fc_print_safety_line();
    FcMotorGateStats g = fc_motor_gate_stats();
    printf("  по видам запросов:\n");
    for (int i = 0; i < FC_MOTOR_REQ_KIND_COUNT; ++i) {
        if (!g.by_kind[i]) {
            continue;
        }
        printf("    %-22s %llu (последнее значение %.3f)\n",
               fc_motor_gate_kind_name((FcMotorRequestKind) i), (unsigned long long) g.by_kind[i],
               (double) g.last_value[i]);
    }
    printf("  timeout_reset  %llu (продление watchdog, тяги не запрашивает)\n",
           (unsigned long long) g.keepalive_calls);
}

// ========================================================== STATE_CHANGING
#if FC_LAB_DIAGNOSTICS

static void cmd_ready(void) {
    bool ok = fc_supervisor_request_ready(fc_uptime_us());
    printf("supervisor: переход в READY %s, состояние %s\n", ok ? "выполнен" : "ОТКЛОНЁН",
           fc_supervisor_state_name(fc_supervisor_state()));
    if (!ok) {
        printf("  причина: не выполнены условия или активен отказ — см. `supervisor`\n");
    }
    printf("  запись конфигурации теперь %s\n",
           fc_flash_write_allowed() ? "разрешена" : "запрещена");
    printf("  выход на мотор: %s (в LAB_SAFE не разрешает ни одно состояние)\n",
           fc_supervisor_motor_output_permitted() ? "РАЗРЕШЁН" : "запрещён");
}

static void cmd_disarm(void) {
    fc_supervisor_disarm(fc_uptime_us());
    printf("supervisor: состояние %s, запись конфигурации %s\n",
           fc_supervisor_state_name(fc_supervisor_state()),
           fc_flash_write_allowed() ? "разрешена" : "запрещена");
}

static void cmd_fault_clear(void) {
    bool ok = fc_supervisor_clear_fault(fc_uptime_us());
    printf("supervisor: снятие отказа %s, состояние %s, активные причины %s\n",
           ok ? "выполнено" : "ОТКЛОНЕНО (причина всё ещё активна)",
           fc_supervisor_state_name(fc_supervisor_state()),
           fc_supervisor_fault_name(fc_supervisor_status().faults));
}

// Пороги отката по напряжению. SAFE: меняет конфигурацию Refloat, не ESC.
// Запись проходит ту же политику супервизора, что и любая другая.
static void cmd_tiltback(const char *args) {
    float lv = 0.0f, hv = 0.0f;
    refloat_facade_get_voltage_tiltback(&lv, &hv);
    uint8_t cells = fc_battery_cell_count();

    if (!args || !*args) {
        printf("пороги отката по напряжению (конфигурация Refloat):\n");
        printf("  tiltback_lv  %.2f В/ячейку -> %.1f В при %u ячейках\n", (double) lv,
               (double) (lv < 10.0f ? lv * cells : lv), cells);
        printf("  tiltback_hv  %.2f В/ячейку -> %.1f В при %u ячейках\n", (double) hv,
               (double) (hv < 10.0f ? hv * cells : hv), cells);
        printf("изменить: tiltback <lv> <hv>   (В на ячейку)\n");
        return;
    }

    char *end = NULL;
    float new_lv = strtof(args, &end);
    if (end == args) {
        printf("tiltback <lv> <hv>\n");
        return;
    }
    char *end2 = NULL;
    float new_hv = strtof(end, &end2);
    if (end2 == end) {
        printf("tiltback <lv> <hv>\n");
        return;
    }

    printf("tiltback: %.2f -> %.2f В/яч (низкое), %.2f -> %.2f В/яч (высокое)\n", (double) lv,
           (double) new_lv, (double) hv, (double) new_hv);
    bool ok = refloat_facade_set_voltage_tiltback(new_lv, new_hv);
    printf("tiltback: запись %s\n", ok ? "принята" : "ОТКЛОНЕНА");
    refloat_facade_get_voltage_tiltback(&lv, &hv);
    printf("tiltback: сейчас %.2f / %.2f В/яч = %.1f / %.1f В при %u ячейках\n", (double) lv,
           (double) hv, (double) (lv * cells), (double) (hv * cells), cells);
    printf("tiltback: коммит в NVS выполняет задача хранилища, не контур\n");
}

static void cmd_persist(void) {
    float before = refloat_facade_config_test_value();
    float next = (before >= 0.9f) ? 0.25f : before + 0.1f;
    printf("persist: leds.status.brightness_headlights_off %.3f -> %.3f\n", (double) before,
           (double) next);
    printf("persist: политика записи сейчас %s (supervisor %s)\n",
           fc_flash_write_allowed() ? "разрешает" : "ЗАПРЕЩАЕТ",
           fc_supervisor_state_name(fc_supervisor_state()));
    bool ok = refloat_facade_config_save_test(next);
    printf("persist: set_cfg вернул %s\n", ok ? "true" : "false");
    FcStorageStats st = fc_storage_stats();
    printf("persist: записей принято %llu, отклонено %llu\n",
           (unsigned long long) st.writes_accepted, (unsigned long long) st.writes_rejected);
    printf("persist: значение в конфигурации сейчас %.3f\n",
           (double) refloat_facade_config_test_value());
    printf("persist: коммит выполняет отдельная задача хранилища, не контур\n");
    FcStorageEvent ev[FC_STORAGE_EVENTS];
    uint32_t n = fc_storage_events(ev, FC_STORAGE_EVENTS);
    uint64_t since = 0;
    printf("flash сейчас      %s\n", fc_storage_busy(&since) ? "ЗАНЯТ" : "свободен");
    if (n == 0) {
        printf("операций с flash  ни одной после загрузки\n");
        printf("  Это и есть ответ на вопрос «виновата ли flash»: если провалы\n");
        printf("  планирования есть, а операций нет, — версия не подтверждается.\n");
    } else {
        uint64_t now = fc_uptime_us();
        printf("операций с flash  %" PRIu32 " (от свежей к старым; кэш на это время отключён,\n", n);
        printf("                  встают ОБА ядра)\n");
        for (uint32_t k = 0; k < n; ++k) {
            printf("  #%-2" PRIu32 " %8.3f с назад, длительность %6" PRIu32 " мкс, %s\n", k,
                   (double) (now - ev[k].start_us) / 1e6, ev[k].duration_us,
                   ev[k].ok ? "успех" : "ОШИБКА");
        }
    }
    FcFlashPolicyStats fp = fc_flash_policy_stats();
    printf("отложенная запись (ТЗ v0.9I §12):\n");
    printf("  ожидает         %s%s\n", fp.pending ? "ДА" : "нет",
           fp.pending ? " — выполнится при первом DISARMED" : "");
    printf("  просьб %llu, отложено эпизодов %llu, выполнено %llu, ошибок %llu\n",
           (unsigned long long) fp.requested, (unsigned long long) fp.deferred,
           (unsigned long long) fp.executed, (unsigned long long) fp.failed);
    printf("  последняя запись %" PRIu32 " мкс, худшая %" PRIu32 " мкс\n", fp.last_duration_us,
           fp.max_duration_us);
}

static void cmd_timing_reset(void) {
    fc_timing_reset();
    printf("статистика тайминга обнулена\n");
}

static void cmd_restart(void) {
    printf("restart: esp_restart(), причина следующей загрузки должна быть SW\n");
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
}

// ========================================================= LAB_DIAGNOSTICS
// Намеренно ломают подсистему, чтобы доказать, что защита срабатывает.
// Ни одна из них не способна подать что-либо на мотор: единственный путь к
// мотору — Motor Gate, а backend физической отправки в этой сборке не
// существует как код.

static void cmd_wdtest(void) {
    printf("wdtest: контур перестаёт отмечаться на 8 с при таймауте TWDT %d с\n",
           CONFIG_ESP_TASK_WDT_TIMEOUT_S);
    printf("  ожидается: сначала FAULT супервизора (таймаут контура %d мс), затем TWDT\n",
           FC_SUP_LOOP_TIMEOUT_US / 1000);
    fc_imu_inject_stall(8000);
}

static void cmd_crashtest(void) {
    printf("crashtest: намеренное разыменование нулевого указателя\n");
    fflush(stdout);
    volatile int *p = (volatile int *) 0;
    *p = 1;
}

// Остановка/запуск задачи чтения датчика. Нужна, чтобы снять прогоны A и B
// (ТЗ v0.6A §20) на одной и той же прошивке: иначе сравнивались бы разные
// двоичные файлы, и разницу нельзя было бы приписать именно IMU.
static void cmd_imu_stop(void) {
    fc_imu_rt_stop();
    printf("imu-stop: задача чтения ICM-20948 останавливается\n");
    printf("  ВНИМАНИЕ: с v0.6D контур питается от этого же датчика — он остановится,\n");
    printf("  и супервизор уйдёт в FAULT по таймауту тика. Это ожидаемо.\n");
}

static void cmd_imu_start(void) {
    bool ok = fc_imu_rt_start();
    printf("imu-start: %s\n", ok ? "задача чтения запущена" : "датчик не отвечает");
}

// Стресс-тест шины I2C (ТЗ v0.6C). Категория STATE_CHANGING: замещает
// штатную задачу чтения датчика, но физически безопасен — путей к мотору в
// нём нет. Но контур на время теста остаётся без данных: датчик один, и
// владение шиной исключительное (ТЗ v0.6D §27).
static void cmd_imu_stress(const char *arg) {
    uint32_t seconds = 150;  // > 2 минут по ТЗ, с запасом на разгон
    uint32_t khz = 0;        // 0 — штатные 400 кГц
    uint32_t threshold = FC_STRESS_DEFAULT_RESET_THRESHOLD;
    if (arg && *arg) {
        char *end = NULL;
        long v = strtol(arg, &end, 10);
        if (v >= 5 && v <= 3600) {
            seconds = (uint32_t) v;
        } else {
            printf("imu_stress: длительность вне диапазона 5…3600 s, беру %" PRIu32 "\n", seconds);
        }
        if (end) {
            char *end2 = NULL;
            long k = strtol(end, &end2, 10);
            if (k >= 10 && k <= 400) {
                khz = (uint32_t) k;
            } else if (k != 0) {
                printf("imu_stress: частота вне диапазона 10…400 кГц, беру штатную\n");
            }
            if (end2 && *end2) {
                long r = strtol(end2, NULL, 10);
                if (r >= 0 && r <= 100000) {
                    threshold = (uint32_t) r;
                }
            }
        }
    }
    if (fc_imu_stress_running()) {
        printf("imu_stress: тест уже идёт, остановить — imu_stress-stop\n");
        return;
    }
    printf("imu_stress: запуск на %" PRIu32 " s. Воздействуйте на жгут не менее двух минут.\n",
           seconds);
    printf("  досрочная остановка: imu_stress-stop\n");
    printf("  предыстория первого отказа: imu_stress-log\n");
    if (khz) {
        printf("  частота шины на время теста: %" PRIu32 " кГц (штатная 400)\n", khz);
    }
    if (threshold) {
        printf("  сброс шины после %" PRIu32 " отказов подряд\n", threshold);
    } else {
        printf("  сброс шины ОТКЛЮЧЁН (диагностика: восстановится ли шина сама)\n");
    }
    if (!fc_imu_stress_start(seconds, khz * 1000, threshold)) {
        printf("imu_stress: НЕ ЗАПУЩЕН (датчик не поднят или штатная задача не остановилась)\n");
    }
}

static void cmd_imu_stress_stop(void) {
    if (!fc_imu_stress_running()) {
        printf("imu_stress: тест не запущен\n");
        return;
    }
    fc_imu_stress_stop();
    printf("imu_stress: остановка запрошена, итоговый отчёт напечатает сама задача\n");
}

// Имитация нажатых футпадов (ТЗ v0.6D §16). LAB_DIAGNOSTICS: меняет только
// напряжение, которое видит Refloat, и ни на шаг не приближает к мотору.
static void cmd_footpad_sim(bool on) {
    fc_adc_simulate_footpads(on, 3.0f);
    printf("footpad-sim: имитация %s (%.1f В на обоих каналах, порог 2.0 В)\n",
           on ? "ВКЛЮЧЕНА" : "выключена", on ? 3.0 : 0.0);
    if (on) {
        printf("  Refloat сможет войти в RUNNING и начнёт запрашивать балансировочный ток.\n");
        printf("  Выход на мотор при этом не появляется: backend физической отправки в\n");
        printf("  профиле LAB_SAFE отсутствует как код. Следите за sent= в `safety`.\n");
    }
}

// Detect / Save / Clear калибровки (ТЗ v0.6E §12).
//
// Detect и Save разделены намеренно: измерение ничего не записывает во flash,
// и человек видит результат до того, как он станет постоянным.
#if FC_CAN_RX_AVAILABLE
// Остановка и запуск приёма CAN. STATE_CHANGING, но физически безопасна:
// останавливается только приём, а передачи в этой сборке не существует вовсе.
// Нужна для того, чтобы приписать изменение тайминга именно приёму CAN, а не
// предполагать это: один и тот же двоичный файл измеряется с приёмом и без.
static void cmd_can_stop(void) {
    if (!fc_can_bus_running()) {
        printf("can-stop: приём и так не запущен\n");
        return;
    }
    fc_can_bus_stop();
    printf("can-stop: приём остановлен, драйвер TWAI выгружен\n");
}

static void cmd_can_start(void) {
    if (fc_can_bus_running()) {
        printf("can-start: приём уже идёт\n");
        return;
    }
    printf("can-start: %s\n", fc_can_bus_start() ? "приём запущен" : "TWAI не поднялся");
}
#endif

static void cmd_imu_cal_detect(const char *arg) {
    float yaw = 0.0f;
    if (arg && *arg) {
        yaw = strtof(arg, NULL);
    }
    if (!fc_imu_rt_available()) {
        printf("imu-cal-detect: датчик не поднят\n");
        return;
    }
    FcDetectConfig cfg = fc_imu_detect_default_config();
    fc_imu_detect_start(&cfg, yaw);
    printf("imu-cal-detect: измерение начато, %" PRIu32 " семплов (около %.1f с при 500 Гц)\n",
           cfg.samples, (double) cfg.samples / 500.0);
    printf("  Держите доску НЕПОДВИЖНО в том положении, которое считается ровным.\n");
    printf("  Любое движение сбрасывает накопление — измерение просто не завершится.\n");
    printf("  rot_yaw = %+.1f град (задаётся аргументом, акселерометром не измеряется)\n",
           (double) yaw);
    printf("  Результат посмотреть: imu-cal-show. Записать: imu-cal-save\n");
}

static void cmd_imu_cal_save(void) {
    FcDetectStatus d = fc_imu_detect_status();
    if (d.state != FC_DETECT_DONE) {
        printf("imu-cal-save: нечего сохранять, измерение в состоянии %s\n",
               fc_imu_detect_state_name(d.state));
        return;
    }
    if (!fc_flash_write_allowed()) {
        printf("imu-cal-save: ОТКЛОНЕНО, запись разрешена только в DISARMED (сейчас %s)\n",
               fc_supervisor_state_name(fc_supervisor_state()));
        return;
    }
    if (!fc_imu_cal_store_save(&d.result)) {
        printf("imu-cal-save: запись не удалась\n");
        return;
    }
    // Применяем сразу: иначе человек увидит эффект только после перезагрузки и
    // не сможет проверить результат тем же прогоном.
    fc_imu_pipeline_set_calibration(&d.result);
    fc_imu_pipeline_reset_ahrs();
    fc_imu_rt_set_cal_status((int) FC_IMU_CAL_VALID);
    printf("imu-cal-save: записано и применено. Фильтр ориентации перезапущен.\n");
    printf("  roll %+8.3f  pitch %+8.3f  yaw %+8.3f град\n", (double) d.result.rot_roll_deg,
           (double) d.result.rot_pitch_deg, (double) d.result.rot_yaw_deg);
}

static void cmd_imu_cal_clear(void) {
    if (!fc_flash_write_allowed()) {
        printf("imu-cal-clear: ОТКЛОНЕНО, запись разрешена только в DISARMED (сейчас %s)\n",
               fc_supervisor_state_name(fc_supervisor_state()));
        return;
    }
    if (!fc_imu_cal_store_clear()) {
        printf("imu-cal-clear: стирание не удалось\n");
        return;
    }
    FcImuCalibration id = fc_imu_calibration_identity();
    fc_imu_pipeline_set_calibration(&id);
    fc_imu_pipeline_reset_ahrs();
    fc_imu_rt_set_cal_status((int) FC_IMU_CAL_NOT_CALIBRATED);
    fc_imu_detect_abort();
    printf("imu-cal-clear: калибровка стёрта, применена единичная.\n");
    printf("  Статус NOT_CALIBRATED — READY теперь недостижим до новой калибровки.\n");
}

static void cmd_imu_fail(void) {
    printf("imu-fail: следующие 500 чтений ICM-20948 завершатся ошибкой\n");
    printf("  ожидается: health READ_ERROR -> supervisor FAULT (IMU_UNHEALTHY)\n");
    icm20948_inject_read_failures(500);
}

static void cmd_imu_freeze(void) {
    printf("imu-freeze: следующие 500 чтений вернут один и тот же семпл\n");
    printf("  ожидается: health STUCK -> supervisor FAULT (IMU_UNHEALTHY)\n");
    icm20948_inject_frozen(500);
}

#endif  // FC_LAB_DIAGNOSTICS

static void cmd_help(void) {
    printf("диагностика (read-only): status | supervisor | imu | i2cscan | timing | timing-hist |\n");
    printf("                         cpu [мс] |\n");
    printf("                         tasks | heap | config | safety | imu-cal-show | help\n");
#if FC_CAN_RX_AVAILABLE
    printf("пороги напряжения:       tiltback | tiltback <lv> <hv>\n");
    printf("журнал:                  log\n");
    printf("планировщик:             sched | sched-reset\n");
    printf("шина CAN:                can | can-frames | can-reset | can-health\n");
#if FC_CAN_DIAG_TX_AVAILABLE
    printf("диагностика CAN (r/o):   can-diag <ping|fw|values|mcconf|appconf> <id>\n");
    printf("                         can-diag-hex <...> <id> | can-diag-poll <сек>\n");
    printf("                         can-diag-stats | can-diag-reset\n");
#endif
#endif
#if FC_LAB_DIAGNOSTICS
    printf("меняют состояние:        ready | disarm | fault-clear | persist | timing-reset |\n");
    printf("                         restart\n");
    printf("проверка защит:          wdtest-confirm | crashtest-confirm | imu-fail-confirm |\n");
    printf("                         imu-freeze-confirm | imu-stop-confirm | imu-start\n");
    printf("                         footpad-sim-confirm | footpad-sim-off\n");
    printf("калибровка IMU:          imu-cal-detect [yaw] | imu-cal-save | imu-cal-clear\n");
#if FC_CAN_RX_AVAILABLE
    printf("приём CAN:               can-stop | can-start\n");
#endif
    printf("стресс-тест шины I2C:    imu_stress [сек] [кГц] [порог_сброса] | imu_stress-stop |\n");
    printf("                         imu_stress-log\n");
#endif
    printf("зазоры IMU:              gaps [порог_мкс] | gaps-reset | imu-stall-confirm <мс>\n");
    printf("политика IMU:            imu-policy | imu-accel-low-confirm <N>\n");
    printf("теневая команда:         shadow | shadow-tail [N] | shadow-reset\n");
    printf("пределы тока:            limits | limits-sync\n");
    printf("коэффициенты:            gains | gain-scale <k> | gain-no-integral |\n");
    printf("                         gain-restore\n");
#if FC_MOTOR_BACKEND_AVAILABLE
    printf("МОТОРНЫЙ СТЕНД (колесо вывешено!):\n");
    printf("                         motor-status | motor-arm | motor-disarm\n");
    printf("                         motor-run <А> <мс> | motor-stop\n");
    printf("                         motor-oneshot-prep | motor-oneshot-fire\n");
    printf("                         motor-clear-latch | motor-inject <маска> |\n");
    printf("                         motor-reset-stats\n");
#endif
    printf("профиль сборки:          %s\n", FC_PROFILE_NAME);
    printf("команд управления мотором нет: в этой сборке нет кода, способного что-либо\n");
    printf("отправить — см. docs/esp32_safety.md\n");
}

// ------------------------------------------- зазоры IMU и моторный стенд

static void cmd_gaps(const char *arg) {
    if (arg && *arg) {
        uint32_t th = (uint32_t) strtoul(arg, NULL, 10);
        if (th) {
            fc_gap_trace_set_threshold(th);
            printf("порог трассировки: %" PRIu32 " мкс\n", th);
        }
    }
    fc_gap_port_print();
}

static void cmd_gaps_reset(void) {
    fc_gap_trace_reset();
    printf("трассировка зазоров: обнулена\n");
}

#if FC_MOTOR_BACKEND_AVAILABLE

static void print_arm_deny(uint32_t mask) {
    for (int b = 0; b < FC_DUAL_ARM_DENY_COUNT; ++b) {
        if (mask & (1u << b)) {
            printf("    - %s\n", fc_dual_motor_arm_deny_name((FcDualArmDeny) (1u << b)));
        }
    }
}

static void cmd_motor_status(void) {
    FcMotorExpStats e = fc_motor_experiment_stats();
    FcDualMotorStats d = fc_dual_motor_stats();
    FcMotorGateStats g = fc_motor_gate_stats();
    FcCanMotorStats m = fc_can_bus_motor_stats();

    printf("профиль сборки    %s\n", FC_PROFILE_NAME);
    printf("профиль CAN       %s\n", FC_CAN_PROFILE_NAME);
    printf("вооружение        %s\n", d.armed ? "ВООРУЖЕНО" : "обезоружено");
    printf("защёлка           %s\n", d.latched ? "ЗАЩЁЛКНУТА" : "нет");
    printf("источник          %s, запрошено %.3f А, осталось %" PRIu32 " мс\n",
           e.running ? "РАБОТАЕТ" : "остановлен", (double) e.requested_a, e.remaining_ms);
    printf("программный предел %.2f А (предел ESC 5 А не трогаем)\n",
           (double) FC_MOTOR_EXP_MAX_CURRENT_A);
    printf("Motor Gate        запросов %llu, разрешено %llu, доставлено %llu, физически %llu\n",
           (unsigned long long) g.requests_total, (unsigned long long) g.allowed_by_policy,
           (unsigned long long) g.delivered_to_backend,
           (unsigned long long) g.physically_sent);
    printf("  отказы          origin %llu, disarmed %llu, fault %llu, invalid %llu, no_backend %llu\n",
           (unsigned long long) g.rejected_origin, (unsigned long long) g.rejected_disarmed,
           (unsigned long long) g.rejected_fault, (unsigned long long) g.rejected_invalid,
           (unsigned long long) g.rejected_no_backend);
    printf("  по источникам   refloat %llu, эксперимент %llu\n",
           (unsigned long long) g.by_origin[FC_MOTOR_ORIGIN_REFLOAT],
           (unsigned long long) g.by_origin[FC_MOTOR_ORIGIN_EXPERIMENT]);
    printf("координатор       разрешено %llu, отказано %llu, частичных передач %llu\n",
           (unsigned long long) d.permits_granted, (unsigned long long) d.permits_denied,
           (unsigned long long) d.partial_sends);
    if (d.last_deny_mask) {
        printf("  последний отказ:\n");
        for (int b = 0; b < FC_DUAL_DENY_REASON_COUNT; ++b) {
            if (d.last_deny_mask & (1u << b)) {
                printf("    - %s\n", fc_dual_motor_reason_name((FcDualDenyReason) (1u << b)));
            }
        }
    }
    printf("пары              целиком %llu, частичных %llu, запрещено %llu\n",
           (unsigned long long) e.pairs_sent, (unsigned long long) e.pairs_partial,
           (unsigned long long) e.pairs_denied);
    printf("разбег пары       последний %" PRIu32 ", p50 %" PRIu32 ", p99 %" PRIu32
           ", максимум %" PRIu32 " мкс (граница %u)\n",
           e.skew_last_us, e.skew_p50_us, e.skew_p99_us, e.skew_max_us,
           (unsigned) FC_DUAL_MAX_SKEW_US);
    if (e.path_samples) {
        printf("стоимость пути    mean %.1f, p50 %" PRIu32 ", p99 %" PRIu32 ", p99.9 %" PRIu32
               ", min %" PRIu32 ", max %" PRIu32 " мкс  (n=%llu)\n",
               (double) e.path_sum_us / (double) e.path_samples, e.path_p50_us, e.path_p99_us,
               e.path_p999_us, e.path_min_us, e.path_max_us,
               (unsigned long long) e.path_samples);
        printf("                  Motor Gate -> координатор -> сериализация 118 -> TX -> "
               "сериализация 100 -> TX\n");
        printf("                  доля периода 2000 мкс: mean %.2f %%, p99 %.2f %%\n",
               100.0 * (double) e.path_sum_us / (double) e.path_samples / 2000.0,
               100.0 * (double) e.path_p99_us / 2000.0);
    }
    printf("кадры мотору      попыток %llu, ушло %llu, неудач %llu, отвергнуто сборкой %llu\n",
           (unsigned long long) m.attempts, (unsigned long long) m.sent,
           (unsigned long long) m.failed, (unsigned long long) m.build_rejected);
    printf("последняя команда %llu мкс, последняя передача %llu мкс\n",
           (unsigned long long) e.last_command_us, (unsigned long long) e.last_tx_us);
    printf("впрыск отказов    0x%03" PRIx32 "%s\n", e.inject_mask,
           e.inject_mask ? "  (ВКЛЮЧЁН)" : "");
}

static void cmd_motor_arm(void) {
    uint32_t deny = 0;
    if (fc_motor_experiment_arm(&deny)) {
        printf("ВООРУЖЕНО. Колесо обязано быть вывешено.\n");
        printf("Разоружить немедленно: motor-disarm\n");
        return;
    }
    printf("вооружение ОТКЛОНЕНО, не выполнены условия:\n");
    print_arm_deny(deny);
}

static void cmd_motor_disarm(void) {
    fc_motor_experiment_disarm();
    printf("обезоружено, источник остановлен\n");
}

static void cmd_motor_clear(void) {
    fc_motor_experiment_clear_latch();
    printf("защёлка снята. Вооружение НЕ восстановлено: motor-arm отдельно\n");
}

static void cmd_motor_run(const char *arg) {
    if (!arg || !*arg) {
        printf("motor-run <ампер> <мс>\n");
        return;
    }
    char *end = NULL;
    float a = strtof(arg, &end);
    uint32_t ms = end ? (uint32_t) strtoul(end, NULL, 10) : 0;
    if (ms == 0) {
        ms = 1000;
    }
    if (!fc_motor_experiment_run(a, ms)) {
        printf("ОТКЛОНЕНО: ток вне предела %.2f А, либо источник уже работает\n",
               (double) FC_MOTOR_EXP_MAX_CURRENT_A);
        return;
    }
    printf("источник запущен: %.3f А на %" PRIu32 " мс\n", (double) a, ms);
}

static void cmd_motor_stop(void) {
    fc_motor_experiment_stop();
    printf("остановка источника запрошена\n");
}

static void cmd_motor_inject(const char *arg) {
    if (!arg || !*arg) {
        printf("motor-inject <маска hex>  (0 — снять)\n");
        printf("  0x001 IMU stale   0x002 узел A молчит   0x004 узел B молчит\n");
        printf("  0x008 fault A     0x010 fault B         0x020 BUS_OFF\n");
        printf("  0x040 CAN degraded 0x080 отказ TX A     0x100 отказ TX B\n");
        printf("  0x200 остановка источника\n");
        printf("текущая маска: 0x%03" PRIx32 "\n", fc_motor_experiment_injected());
        return;
    }
    uint32_t m = (uint32_t) strtoul(arg, NULL, 0);
    fc_motor_experiment_inject(m);
    printf("маска впрыска: 0x%03" PRIx32 "\n", m);
}

static void cmd_motor_reset(void) {
    fc_motor_experiment_reset_stats();
    printf("статистика моторного стенда обнулена\n");
}


// ------------------------------------------- одношаговая команда (ТЗ v0.9J)
//
// Одна пара кадров, 118 затем 100, без обновления. Путь ровно тот же, что у
// контура: Motor Gate -> координатор -> транспорт -> обе половины. Источник —
// EXPERIMENT, а не REFLOAT: контур балансировки через гейт по-прежнему не
// проходит (бит REFLOAT в маске не ставится, сборка без
// FLOATCORE_REFLOAT_REAL_MOTOR_LOOP). Величина и ЗНАК берутся из команды
// Refloat, поэтому проверяется сквозной путь Refloat -> мотор, но непрерывного
// контура нет: одна команда по явному действию оператора.
//
// Два шага намеренно. Подготовка проверяет всё и печатает чекпойнт §9;
// отправка без действительного токена отвергается.

#define ONESHOT_TTL_US 60000000ull
#define ONESHOT_ENVELOPE_A 0.5f
#define ONESHOT_MIN_ERR_DEG 0.2f   // ниже — ток Refloat около мёртвой зоны 0.38 А
#define ONESHOT_MAX_ERR_DEG 2.0f   // выше — «большой наклон», §6

static struct {
    bool valid;
    uint64_t prepared_us;
    float amps;
    float raw_a;
    float err_deg;
    HalfTelemetry pre[2];
    uint64_t pairs_before, partial_before;
    uint64_t tx_before[2];
    uint64_t gate_sent_before;
    // результат отправки
    bool fired;
    uint64_t t_request, t_done;
    uint32_t verdict;
} ONESHOT;

static bool check(bool ok, const char *what, int *fails) {
    printf("  [%s] %s\n", ok ? "да " : "НЕТ", what);
    if (!ok) {
        ++*fails;
    }
    return ok;
}

static void cmd_oneshot_prep(void) {
    ONESHOT.valid = false;
    int fails = 0;
    // Статические: консоль однопоточна, а на стеке эти структуры вместе с
    // printf уже однажды переполнили стек задачи (v0.9J, отчёт одношага).
    static RefloatShadowFields sh;
    refloat_facade_shadow(&sh);
    static RefloatGains g;
    refloat_facade_gains(&g);
    static FcSupervisorStatus ss;
    ss = fc_supervisor_status();
    static FcMotorExpStats es;
    es = fc_motor_experiment_stats();
    static FcLimitsSyncStatus ls;
    ls = fc_limits_sync_status();
    static FcFlashPolicyStats fp;
    fp = fc_flash_policy_stats();
    static FcTimingStats tc, tm, tw;
    tc = fc_timing_get(FC_TIMING_CONTROL);
    tm = fc_timing_get(FC_TIMING_MAIN);
    tw = fc_timing_get(FC_TIMING_IMU_WAKE);

    float err = sh.setpoint - sh.balance_pitch;
    float raw = sh.balance_current;
    // Корректирующий знак: нос вниз (тангаж < setpoint) -> ошибка > 0 ->
    // положительный ток -> верх колеса к носу. Никакой инверсии в этом пути нет.
    int expected = err > 0.0f ? 1 : (err < 0.0f ? -1 : 0);
    int got = raw > 0.0f ? 1 : (raw < 0.0f ? -1 : 0);
    float amps = got > 0 ? ONESHOT_ENVELOPE_A : -ONESHOT_ENVELOPE_A;

    printf("=== ПРОВЕРКИ ОДНОШАГА =========================================\n");
    check(fabsf(g.kp - 4.0f) < 1e-4f && fabsf(g.kp2 - 0.12f) < 1e-4f,
          "коэффициенты кандидата A: kp 4.000, kp2 0.120", &fails);
    check(g.ki == 0.0f && g.ki_limit == 0.0f,
          "интеграл как при теневой квалификации: ki 0, ki_limit 0", &fails);
    check(sh.state == 3, "Refloat в RUNNING (иначе его команда не обновляется)", &fails);
    check(ss.state != FC_SUP_FAULT && ss.faults == 0, "супервизор здоров, отказов нет", &fails);
    check(fc_supervisor_last_imu_permit() == FC_IMU_PERMIT_OK, "политика IMU: OK", &fails);
    check(fc_imu_rt_cal_status() == FC_IMU_CAL_VALID, "калибровка ориентации действительна", &fails);
    check(fc_motor_experiment_armed(), "координатор вооружён оператором", &fails);
    check(!es.running, "источник 500 Гц НЕ работает", &fails);
    check(ls.applied && ls.common.current_max > 0.0f, "пределы ESC синхронизированы", &fails);
    check(!fp.pending && !fc_storage_busy(NULL), "отложенных записей во flash нет", &fails);
    check(tc.missed == 0 && tm.missed == 0 && tw.missed == 0,
          "пропусков дедлайнов с последнего timing-reset нет", &fails);
    check(fabsf(err) >= ONESHOT_MIN_ERR_DEG && fabsf(err) <= ONESHOT_MAX_ERR_DEG,
          "ошибка тангажа в окне 0.2…2.0° (не большой наклон)", &fails);
    check(fabsf(raw) >= ONESHOT_ENVELOPE_A, "команда Refloat не меньше 0.5 А (выше мёртвой зоны)",
          &fails);
    check(expected != 0 && got == expected, "ЗНАК команды Refloat корректирующий", &fails);

    // Телеметрия и отказы обеих половин — последней, она занимает шину.
    ONESHOT.pre[0] = read_half(118, true);
    vTaskDelay(pdMS_TO_TICKS(250));
    ONESHOT.pre[1] = read_half(100, true);
    for (int k = 0; k < 2; ++k) {
        const HalfTelemetry *h = &ONESHOT.pre[k];
        char what[160];
        snprintf(what, sizeof(what), "половина %u: телеметрия есть, отказ NONE, таймаут прочитан",
                 k ? 100 : 118);
        check(h->values_ok && h->v.has_fault && h->v.fault_code == 0 && h->timeout_ok, what,
              &fails);
    }

    printf("\n=== ЧЕКПОЙНТ §9 ===============================================\n");
    printf("BUILD      профиль %s\n", FC_PROFILE_NAME);
    printf("STATE      супервизор %s, координатор %s, Refloat %s\n",
           fc_supervisor_state_name(ss.state), fc_motor_experiment_armed() ? "ВООРУЖЁН" : "не вооружён",
           refloat_facade_state_name(sh.state));
    printf("           замкнутый контур: %s; запросы Refloat в гейт отвергаются (разрешено %llu)\n",
           FC_CLOSED_LOOP_AVAILABLE ? "собран, выключен" : "ОТСУТСТВУЕТ в сборке",
           (unsigned long long) fc_motor_gate_stats().allowed_by_policy);
    printf("BATTERY    118: %.1f В, 100: %.1f В  — сверить с мультиметром\n",
           (double) ONESHOT.pre[0].v.v_in, (double) ONESHOT.pre[1].v.v_in);
    printf("TACH       опорный тахометр прямо перед отправкой: 118 = %ld, 100 = %ld\n",
           (long) ONESHOT.pre[0].v.tachometer, (long) ONESHOT.pre[1].v.tachometer);
    printf("ANGLE      тангаж %+.3f°, balance_pitch %+.3f°, setpoint %+.3f°, ошибка %+.3f°\n",
           (double) sh.pitch, (double) sh.balance_pitch, (double) sh.setpoint, (double) err);
    printf("           угловая скорость %+.2f °/с\n", (double) sh.pitch_rate);
    printf("COMMAND    Refloat %+.3f А;  предел ESC %+.2f/%+.2f А;  предел одношага ±%.2f А\n",
           (double) raw, (double) ls.common.current_max, (double) ls.common.current_min,
           (double) ONESHOT_ENVELOPE_A);
    printf("           УЙДЁТ: 118 = %+.3f А, 100 = %+.3f А  (одна пара, одинаковая величина)\n",
           (double) amps, (double) amps);
    printf("DIRECTION  %s\n", amps > 0.0f ? "положительный ток: верх колеса К НОСУ (край без фанеры)"
                                          : "отрицательный ток: верх колеса К ХВОСТУ (край с фанерой)");
    printf("           наклон %s -> корректирующее направление %s\n",
           err > 0.0f ? "нос ВНИЗ" : "нос ВВЕРХ", amps > 0.0f ? "к носу" : "к хвосту");
    printf("TIMING     порядок 118 -> 100; разбег пары по v0.9G: p99 0 мкс, граница 1000\n");
    printf("           таймаут команды ESC: 118 %lu мс, 100 %lu мс\n",
           (unsigned long) ONESHOT.pre[0].t.timeout_ms, (unsigned long) ONESHOT.pre[1].t.timeout_ms);
    printf("STOP       после пары НИЧЕГО не обновляется: каждая половина снимет ток сама по\n");
    printf("           таймауту и отпустит колесо в выбег (тормозной ток по таймауту %.2f / %.2f А)\n",
           (double) ONESHOT.pre[0].t.timeout_brake_current_a,
           (double) ONESHOT.pre[1].t.timeout_brake_current_a);
    printf("           физически: отключить питание батареи\n");

    if (fails) {
        printf("\nОТКАЗ: не выполнено условий — %d. Токен НЕ выдан.\n", fails);
        return;
    }
    ONESHOT.valid = true;
    ONESHOT.fired = false;
    ONESHOT.prepared_us = fc_uptime_us();
    ONESHOT.amps = amps;
    ONESHOT.raw_a = raw;
    ONESHOT.err_deg = err;
    printf("\nВсе условия выполнены. Токен действителен 60 с. Отправить: motor-oneshot-fire\n");
}

static void cmd_oneshot_fire(void) {
    if (!ONESHOT.valid) {
        printf("ОТКЛОНЕНО: нет действительной подготовки. Сначала motor-oneshot-prep\n");
        return;
    }
    if (fc_uptime_us() - ONESHOT.prepared_us > ONESHOT_TTL_US) {
        ONESHOT.valid = false;
        printf("ОТКЛОНЕНО: токен просрочен — чекпойнт описывал прошлое положение.\n");
        return;
    }
    // Условия, которые могли измениться за время ожидания, перепроверяются.
    static RefloatShadowFields sh;
    refloat_facade_shadow(&sh);
    float err = sh.setpoint - sh.balance_pitch;
    int expected = err > 0.0f ? 1 : -1;
    int now_sign = sh.balance_current > 0.0f ? 1 : -1;
    int prep_sign = ONESHOT.amps > 0.0f ? 1 : -1;
    if (!fc_motor_experiment_armed() || fabsf(sh.balance_current) < ONESHOT_ENVELOPE_A ||
        now_sign != prep_sign || expected != prep_sign) {
        ONESHOT.valid = false;
        printf("ОТКЛОНЕНО: с момента подготовки изменились вооружение, угол или знак.\n");
        return;
    }
    // Токен одноразовый и гасится ДО отправки.
    ONESHOT.valid = false;

    static FcMotorExpStats es;
    es = fc_motor_experiment_stats();
    ONESHOT.pairs_before = es.pairs_sent;
    ONESHOT.partial_before = es.pairs_partial;
    ONESHOT.tx_before[0] = es.tx_count[0];
    ONESHOT.tx_before[1] = es.tx_count[1];
    ONESHOT.gate_sent_before = fc_motor_gate_stats().physically_sent;

    fc_can_status_capture_start(800);
    uint32_t verdict = 0;
    ONESHOT.t_request = fc_uptime_us();
    bool ok = fc_motor_experiment_oneshot(ONESHOT.amps, &verdict);
    ONESHOT.t_done = fc_uptime_us();
    ONESHOT.verdict = verdict;
    ONESHOT.fired = true;

    static FcMotorExpStats e2;
    e2 = fc_motor_experiment_stats();
    printf("ОДНОШАГ %s: %+.3f А, вердикт гейта %s\n", ok ? "ОТПРАВЛЕН" : "НЕ ОТПРАВЛЕН",
           (double) ONESHOT.amps, fc_motor_gate_verdict_name((FcGateVerdict) verdict));
    printf("  T_request  %llu мкс\n", (unsigned long long) ONESHOT.t_request);
    printf("  T_TX118    %llu мкс (+%llu), %s\n", (unsigned long long) e2.tx_us[0],
           (unsigned long long) (e2.tx_us[0] - ONESHOT.t_request), e2.tx_ok[0] ? "ушёл" : "СБОЙ");
    printf("  T_TX100    %llu мкс (+%llu), %s\n", (unsigned long long) e2.tx_us[1],
           (unsigned long long) (e2.tx_us[1] - ONESHOT.t_request), e2.tx_ok[1] ? "ушёл" : "СБОЙ");
    printf("  разбег пары %llu мкс\n", (unsigned long long) (e2.tx_us[1] - e2.tx_us[0]));
    printf("  повторов не будет. Итог: через секунду motor-oneshot-report\n");
}

static void cmd_oneshot_report(void) {
    if (!ONESHOT.fired) {
        // Холостой прогон: весь путь печати и чтения телеметрии, без данных
        // выстрела. Нужен, чтобы проверить отчёт до того, как он понадобится:
        // в первом одношаге именно он упал, и данные выстрела были потеряны.
        printf("ХОЛОСТОЙ ПРОГОН ОТЧЁТА: одношаг не выполнялся, числа ниже не о выстреле\n");
    }
    while (fc_can_status_capture_active()) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    static FcMotorExpStats e;
    e = fc_motor_experiment_stats();
    uint64_t pairs = e.pairs_sent - ONESHOT.pairs_before;
    uint64_t partial = e.pairs_partial - ONESHOT.partial_before;
    uint64_t tx118 = e.tx_count[0] - ONESHOT.tx_before[0];
    uint64_t tx100 = e.tx_count[1] - ONESHOT.tx_before[1];
    uint64_t gate = fc_motor_gate_stats().physically_sent - ONESHOT.gate_sent_before;

    printf("=== ОТЧЁТ ОДНОШАГА ============================================\n");
    printf("ПАРЫ       целиком %llu, частичных %llu; кадров 118: %llu, 100: %llu; гейт: %llu\n",
           (unsigned long long) pairs, (unsigned long long) partial, (unsigned long long) tx118,
           (unsigned long long) tx100, (unsigned long long) gate);
    printf("           %s\n", (pairs == 1 && partial == 0 && tx118 == 1 && tx100 == 1)
                                   ? "РОВНО ОДНА пара, второй нет"
                                   : "ВНИМАНИЕ: число пар или кадров не равно одному");

    static FcStatusSample cap[FC_STATUS_CAPTURE_MAX];
    uint32_t n = fc_can_status_capture(cap, FC_STATUS_CAPTURE_MAX);
    uint64_t t0 = e.tx_us[0];
    printf("STATUS     %lu кадров за 800 мс после команды (время от T_TX118):\n", (unsigned long) n);
    float max_i[2] = {0, 0};
    int32_t max_erpm[2] = {0, 0};
    int64_t first_after[2] = {-1, -1};
    int64_t last_nonzero_i[2] = {-1, -1};
    for (uint32_t i = 0; i < n; ++i) {
        int k = cap[i].node == 118 ? 0 : 1;
        int64_t dt = (int64_t) cap[i].t_us - (int64_t) t0;
        printf("  %+7lld мкс  %u  ток %+.1f А  ERPM %+ld  скважность %+.3f\n", (long long) dt,
               cap[i].node, (double) cap[i].current_a, (long) cap[i].erpm, (double) cap[i].duty);
        if (dt >= 0 && first_after[k] < 0) {
            first_after[k] = dt;
        }
        if (fabsf(cap[i].current_a) > fabsf(max_i[k])) {
            max_i[k] = cap[i].current_a;
        }
        if (labs((long) cap[i].erpm) > labs((long) max_erpm[k])) {
            max_erpm[k] = cap[i].erpm;
        }
        if (fabsf(cap[i].current_a) >= 0.05f) {
            last_nonzero_i[k] = dt;
        }
    }
    for (int k = 0; k < 2; ++k) {
        printf("  %u: первый STATUS через %lld мкс; наибольший ток %+.1f А; ERPM до %+ld; "
               "последний ненулевой ток %lld мкс\n", k ? 100 : 118, (long long) first_after[k],
               (double) max_i[k], (long) max_erpm[k], (long long) last_nonzero_i[k]);
    }

    static HalfTelemetry post[2];
    post[0] = read_half(118, false);
    vTaskDelay(pdMS_TO_TICKS(250));
    post[1] = read_half(100, false);
    for (int k = 0; k < 2; ++k) {
        uint8_t id = k ? 100 : 118;
        if (post[k].values_ok && ONESHOT.pre[k].values_ok) {
            long d = (long) (post[k].v.tachometer - ONESHOT.pre[k].v.tachometer);
            printf("ТАХОМЕТР   %u: %ld -> %ld, приращение %+ld (%s); отказ после: %s\n", id,
                   (long) ONESHOT.pre[k].v.tachometer, (long) post[k].v.tachometer, d,
                   d > 0 ? "вперёд, к носу" : (d < 0 ? "назад, к хвосту" : "НЕ сдвинулось"),
                   post[k].v.has_fault ? fc_vesc_fault_name(post[k].v.fault_code) : "?");
        } else {
            printf("ТАХОМЕТР   %u: телеметрия после не получена\n", id);
        }
    }
    printf("ОЖИДАЛОСЬ  %s\n", ONESHOT.amps > 0.0f ? "положительное приращение (к носу)"
                                                  : "отрицательное приращение (к хвосту)");
    static FcFlashPolicyStats fp;
    fp = fc_flash_policy_stats();
    printf("FLASH      ожидает %s, выполнено записей всего %llu (до одношага было учтено в prep)\n",
           fp.pending ? "ДА" : "нет", (unsigned long long) fp.executed);

    // Реальное время за окно (§14): счётчики с последнего timing-reset,
    // который обязан стоять перед prep.
    FcTimingChannel chs[4] = {FC_TIMING_CONTROL, FC_TIMING_MAIN, FC_TIMING_AUX, FC_TIMING_IMU_WAKE};
    printf("REALTIME  ");
    for (int i = 0; i < 4; ++i) {
        static FcTimingStats t;
        t = fc_timing_get(chs[i]);
        printf(" %s missed %" PRIu32 ";", t.name, t.missed);
    }
    printf("\n");
    printf("IMU        политика %s; супервизор %s\n",
           fc_supervisor_last_imu_permit() == FC_IMU_PERMIT_OK ? "OK" : "НЕ OK",
           fc_supervisor_state_name(fc_supervisor_state()));
    // FcCanStats несёт таблицу идентификаторов с последними кадрами — больше
    // килобайта. Именно она на стеке и переполнила его в первом одношаге.
    static FcCanStats cs;
    cs = fc_can_bus_stats();
    printf("CAN        ошибок шины %llu, BUS_OFF %llu\n", (unsigned long long) cs.bus_error_count,
           (unsigned long long) cs.bus_off_count);
}

#endif // FC_MOTOR_BACKEND_AVAILABLE


#if FC_LAB_DIAGNOSTICS
static void cmd_imu_stall(const char *arg) {
    uint32_t ms = arg && *arg ? (uint32_t) strtoul(arg, NULL, 10) : 0;
    if (ms == 0 || ms > 500) {
        printf("imu-stall-confirm <мс 1..500>\n");
        printf("  задерживает задачу чтения IMU ровно один раз.\n");
        printf("  свыше %d мс обязано дать IMU_UNHEALTHY по возрасту.\n",
               (int) (FC_SUP_IMU_TIMEOUT_US / 1000));
        return;
    }
    fc_imu_rt_inject_stall((int) ms);
    printf("задержка %" PRIu32 " мс запрошена; смотреть gaps и supervisor\n", ms);
}
#endif


#if FC_LAB_DIAGNOSTICS
static void cmd_imu_accel_low(const char *arg) {
    int n = arg && *arg ? atoi(arg) : 1;
    if (n < 1 || n > 500) {
        printf("imu-accel-low-confirm <N 1..500>\n");
        printf("  подменяет модуль ускорения у N семплов подряд.\n");
        printf("  до %u подряд — тяга не снимается; до %u — снимается без защёлки;\n",
               (unsigned) (20000u / 2000u), (unsigned) (40000u / 2000u));
        printf("  дальше — защёлка.\n");
        return;
    }
    fc_imu_rt_inject_accel_low(n);
    printf("впрыск: %d семплов с малым модулем ускорения\n", n);
}

static void cmd_imu_policy(void) {
    FcImuHealthStatus h = fc_imu_health_status();
    FcImuTimeline tl = fc_imu_pipeline_timeline();
    FcImuPolicyConfig pc = fc_imu_policy_default_config();
    uint64_t now = fc_uptime_us();
    FcImuPolicyInputs in = {
        .now_us = now,
        .last_valid_us = tl.last_valid_us,
        .have_valid = tl.sequence > 0,
        .consecutive_read_failures = h.consecutive_read_errors,
        .consecutive_invalid = h.consecutive_invalid,
        .consecutive_accel_low = h.consecutive_accel_low,
        .bus_stuck = false,
        .reinit_failed = false,
    };
    uint32_t reasons = 0;
    FcImuPermit p = fc_imu_policy_evaluate(&pc, &in, &reasons);
    printf("политика IMU      %s\n", fc_imu_permit_name(p));
    printf("  пороги          свежесть %" PRIu32 " мкс, защёлка %" PRIu32
           " мкс; серии %" PRIu32 " и %" PRIu32 " семплов\n",
           pc.torque_freshness_us, pc.latch_age_us, pc.hold_samples, pc.latch_samples);
    printf("  возраст         %llu мкс, номер семпла %" PRIu32 "\n",
           (unsigned long long) (now > tl.last_valid_us ? now - tl.last_valid_us : 0),
           tl.sequence);
    printf("  серии подряд    обменов %" PRIu32 ", непригодных %" PRIu32
           ", малое |a| %" PRIu32 " (макс %" PRIu32 ")\n",
           h.consecutive_read_errors, h.consecutive_invalid, h.consecutive_accel_low,
           h.max_consecutive_accel_low);
    printf("  малое |a|       принято %llu семплов, отвергнуто %llu\n",
           (unsigned long long) h.accel_low_accepted,
           (unsigned long long) h.invalid_accel_low);
    if (reasons) {
        for (int b = 0; b < FC_IMU_POLICY_REASON_COUNT; ++b) {
            if (reasons & (1u << b)) {
                printf("    - %s\n", fc_imu_policy_reason_name((FcImuPolicyReason) (1u << b)));
            }
        }
    }
}
#endif


// ------------------------------------------- теневая команда Refloat (v0.9D)

static void cmd_shadow(void) {
    FcShadowStats s = fc_shadow_stats();
    if (s.samples == 0) {
        printf("теневых записей нет: Refloat ещё не просил тока\n");
        return;
    }
    double n = (double) s.samples;
    printf("теневая команда Refloat (НЕ передаётся мотору)\n");
    printf("  записей         %llu, потеряно в кольце %llu\n",
           (unsigned long long) s.samples, (unsigned long long) s.dropped);
    printf("  мёртвая зона    <%.2f А: %llu (%.1f %%)\n", (double) FC_SHADOW_DEADZONE_LOW_A,
           (unsigned long long) s.below_low, 100.0 * (double) s.below_low / n);
    printf("                  %.2f-%.2f А: %llu (%.1f %%)\n",
           (double) FC_SHADOW_DEADZONE_LOW_A, (double) FC_SHADOW_DEADZONE_HIGH_A,
           (unsigned long long) s.in_band, 100.0 * (double) s.in_band / n);
    printf("                  >%.2f А: %llu (%.1f %%)\n", (double) FC_SHADOW_DEADZONE_HIGH_A,
           (unsigned long long) s.above_high, 100.0 * (double) s.above_high / n);
    printf("  пики            +%.3f А, %.3f А\n", (double) s.peak_positive,
           (double) s.peak_negative);
    printf("  интеграл        последний %.4f, пик по модулю %.4f\n",
           (double) s.last_i_term, (double) s.peak_abs_i_term);
    printf("  в мёртвой зоне  входов %llu, наибольший прирост интеграла %.4f\n",
           (unsigned long long) s.deadzone_entries, (double) s.max_i_growth_in_deadzone);
    printf("  переходов через ноль %llu, в насыщении %llu (%.1f %%)\n",
           (unsigned long long) s.zero_crossings, (unsigned long long) s.saturation_samples,
           100.0 * (double) s.saturation_samples / n);
    printf("  при пределе %.2f А: неэффективно %llu (%.1f %%), переход %llu (%.1f %%),"
           " действенно %llu (%.1f %%), в упоре %llu (%.1f %%)\n",
           (double) fc_shadow_envelope(), (unsigned long long) s.env_ineffective,
           100.0 * (double) s.env_ineffective / n, (unsigned long long) s.env_transition,
           100.0 * (double) s.env_transition / n, (unsigned long long) s.env_effective,
           100.0 * (double) s.env_effective / n, (unsigned long long) s.env_saturated,
           100.0 * (double) s.env_saturated / n);
    printf("  разрешено к передаче %llu — источник Refloat в маску гейта не входит\n",
           (unsigned long long) s.deliverable_samples);
    if (s.have_pitch) {
        printf("  крайние углы    %.2f° → %.3f А;  %.2f° → %.3f А\n", (double) s.min_pitch,
               (double) s.current_at_min_pitch, (double) s.max_pitch,
               (double) s.current_at_max_pitch);
    }
    printf("  по углам (|ошибка тангажа|):\n");
    float lo = 0.0f;
    for (unsigned i = 0; i < FC_SHADOW_ANGLE_BINS; ++i) {
        if (s.bin_n[i] == 0) {
            lo = FC_SHADOW_ANGLE_EDGES[i];
            continue;
        }
        double bn = (double) s.bin_n[i];
        if (i + 1 == FC_SHADOW_ANGLE_BINS) {
            printf("    >%.1f°%14s n=%-8llu средний |I| %6.3f  упор ESC %5.1f %%  упор опыта %5.1f %%\n",
                   (double) lo, "", (unsigned long long) s.bin_n[i],
                   (double) (s.bin_sum_raw[i] / (float) bn),
                   100.0 * (double) s.bin_saturated_esc[i] / bn,
                   100.0 * (double) s.bin_saturated_env[i] / bn);
        } else {
            printf("    %.1f-%.1f°%11s n=%-8llu средний |I| %6.3f  упор ESC %5.1f %%  упор опыта %5.1f %%\n",
                   (double) lo, (double) FC_SHADOW_ANGLE_EDGES[i], "",
                   (unsigned long long) s.bin_n[i], (double) (s.bin_sum_raw[i] / (float) bn),
                   100.0 * (double) s.bin_saturated_esc[i] / bn,
                   100.0 * (double) s.bin_saturated_env[i] / bn);
        }
        lo = FC_SHADOW_ANGLE_EDGES[i];
    }
    printf("  история         прорежена 1:%u, глубина %.1f с\n",
           (unsigned) FC_SHADOW_DECIMATION,
           (double) FC_SHADOW_RING * (double) FC_SHADOW_DECIMATION / 500.0);
}

static void cmd_shadow_tail(const char *arg) {
    uint32_t want = arg && *arg ? (uint32_t) strtoul(arg, NULL, 10) : 12;
    if (want > 64) {
        want = 64;
    }
    // Статический, а не на стеке: у консольной задачи его 4 КБ, а 64 записи
    // это около трёх. Ровно на этом я уронил плату (LoadProhibited), увеличив
    // буфер с 40 до 64 и не посмотрев на стек. Задача одна, гонки нет.
    static FcShadowSample buf[64];
    uint32_t n = fc_shadow_tail(buf, want);
    printf("последние %" PRIu32 " записей (свежие сверху)\n", n);
    printf("  %10s %8s %8s %9s %8s %8s %8s %4s\n", "мкс", "ток,А", "pitch", "rate", "setp",
           "P", "I", "sat");
    for (uint32_t i = 0; i < n; ++i) {
        printf("  %10llu %8.3f %8.2f %9.2f %8.2f %8.3f %8.4f %4u\n",
               (unsigned long long) buf[i].timestamp_us, (double) buf[i].current,
               (double) buf[i].pitch, (double) buf[i].pitch_rate, (double) buf[i].setpoint,
               (double) buf[i].pid_p, (double) buf[i].pid_i, (unsigned) buf[i].sat);
    }
}

static void cmd_shadow_reset(void) {
    fc_shadow_reset();
    printf("теневая статистика обнулена\n");
}


static void cmd_limits(void) {
    FcLimitsSyncStatus st = fc_limits_sync_status();
    printf("иерархия пределов тока (мотор, А)\n");
    printf("  1 конфигурация ESC   118 %+.2f/%+.2f   100 %+.2f/%+.2f   %s\n",
           (double) st.half[0].current_max, (double) st.half[0].current_min,
           (double) st.half[1].current_max, (double) st.half[1].current_min,
           (st.half[0].valid && st.half[1].valid) ? "прочитано" : "НЕ ПРОЧИТАНО");
    printf("  2 общее пересечение  %+.2f/%+.2f %s\n", (double) st.common.current_max,
           (double) st.common.current_min, st.common.valid ? "" : "(недоступно)");
    printf("  3 предел опыта       %+.2f/%+.2f (только теневой разбор)\n",
           (double) fc_shadow_envelope(), (double) -fc_shadow_envelope());
    float seen_max = 0.0f, seen_min = 0.0f;
    fc_vesc_if_limits_seen(&seen_max, &seen_min);
    printf("  4 видит Refloat      %+.2f/%+.2f   (модель даёт %+.2f/%+.2f)\n", (double) seen_max,
           (double) seen_min, (double) fc_effective_current_max(),
           (double) fc_effective_current_min());
    printf("  синхронизаций        попыток %" PRIu32 ", удачных %" PRIu32 ", применено %s\n",
           st.attempts, st.successes, st.applied ? "да" : "нет");
}

static void cmd_limits_sync(void) {
    printf(fc_limits_sync() ? "пределы синхронизированы\n"
                            : "синхронизация НЕ удалась, значения не менялись\n");
    cmd_limits();
}


static void cmd_gains(void) {
    RefloatGains g;
    refloat_facade_gains(&g);
    RefloatShadowFields f;
    refloat_facade_shadow(&f);

    float kt_ours = g.speed_constant > 0.0f ? 1.0f / g.speed_constant : 0.0f;
    printf("коэффициенты балансировки Refloat\n");
    printf("  kp  %.3f   kp2 %.3f   ki %.5f   ki_limit %.2f\n", (double) g.kp, (double) g.kp2,
           (double) g.ki, (double) g.ki_limit);
    printf("  kp_brake %.2f   kp2_brake %.2f   mahony_kp %.2f\n", (double) g.kp_brake,
           (double) g.kp2_brake, (double) g.mahony_kp);
    printf("  booster %.2f А   brkbooster %.2f А\n", (double) g.booster_current,
           (double) g.brkbooster_current);
    printf("постоянные момента\n");
    printf("  эталон Refloat  %.4f Н·м/А  (1.5 x 15 x 0.027)\n",
           (double) g.torque_constant_compat);
    printf("  наш мотор       %.4f Н·м/А  (1.5 x 15 x lambda)\n", (double) kt_ours);
    printf("  отношение       %.3f\n",
           kt_ours > 0.0f ? (double) (g.torque_constant_compat / kt_ours) : 0.0);
    printf("масштаб контура (один мотор)\n");
    if (kt_ours > 0.0f) {
        float a_per_deg = g.kp * g.torque_constant_compat / kt_ours;
        printf("  P: %.2f А на градус;  ±0.5 А при %.4f°;  ±5 А при %.3f°\n", (double) a_per_deg,
               (double) (0.5f / a_per_deg), (double) (5.0f / a_per_deg));
    }
    if (refloat_facade_gains_modified()) {
        printf("  ВНИМАНИЕ: коэффициенты ВРЕМЕННО изменены, вернуть — gain-restore\n");
    }
    printf("состояние сейчас\n");
    printf("  pitch %.3f  balance_pitch %.3f  setpoint %.3f  ошибка %.3f°\n", (double) f.pitch,
           (double) f.balance_pitch, (double) f.setpoint,
           (double) (f.setpoint - f.balance_pitch));
    printf("  P %.4f  I %.4f  rate_P %.4f  ->  ток %.3f А\n", (double) f.pid_p, (double) f.pid_i,
           (double) f.pid_rate_p, (double) f.balance_current);
}


static void cmd_gain_scale(const char *arg) {
    if (!arg || !*arg) {
        printf("gain-scale <множитель 0.01..10>  |  gain-restore\n");
        printf("  меняет kp, kp2 и ki ОДНОВРЕМЕННО, только в памяти Refloat.\n");
        printf("  исходные сохраняются при первой правке.\n");
        return;
    }
    float k = strtof(arg, NULL);
    if (!refloat_facade_scale_gains(k)) {
        printf("ОТКЛОНЕНО: Refloat не запущен либо множитель вне 0..10\n");
        return;
    }
    printf("коэффициенты умножены на %.3f. ВЕРНУТЬ: gain-restore\n", (double) k);
    cmd_gains();
}



// Доля процессора по задачам (ТЗ v0.9H §8, §18).
//
// Зачем понадобилось, хотя есть собственные каналы таймингов. Они меряют
// только то, что мы сами обернули метками, и ожидание I²C внутри задачи
// датчика (в среднем около 900 мкс на каждые 2 мс) не попадает ни в один из
// них. Пока доля простоя ядра неизвестна, нельзя отличить «задача спит, и её
// не будят» от «задача готова, и её не выбирают» — а это два совершенно
// разных дефекта.
//
// Считается по РАЗНОСТИ двух снимков, а не по абсолютным счётчикам: иначе
// цифра была бы средним за всё время с загрузки и любой режим тонул бы в
// истории.
// Задачи перечисляются по ИМЕНАМ, а не через uxTaskGetSystemState (ТЗ v0.9I §19).
// Та для каждой задачи побайтно сканирует стек в поисках водяного знака, а у
// Refloat Main и Aux по 12 КБ. На приёмке один вызов этой команды после окна
// совпал с зазором IMU 5620 мкс: диагностика сама рвала реальное время.
// Здесь стек не сканируется вовсе: vTaskGetInfo с xGetFreeStackSpace = pdFALSE.
static const char *const CPU_TASKS[] = {
    "fc_imu_rt", "Refloat Main", "Refloat Aux", "fc_super", "fc_can_rx", "fc_log",
    "fc_nvs",    "fc_console",   "fc_report",   "esp_timer", "ipc0",    "ipc1",
};
#define CPU_N (sizeof(CPU_TASKS) / sizeof(CPU_TASKS[0]) + 2u)  // + IDLE0, IDLE1

static void cpu_sample(TaskHandle_t *h, uint32_t *rt) {
    for (unsigned i = 0; i < CPU_N; ++i) {
        rt[i] = 0;
        if (!h[i]) {
            continue;
        }
        TaskStatus_t st;
        vTaskGetInfo(h[i], &st, pdFALSE, eRunning);
        rt[i] = (uint32_t) st.ulRunTimeCounter;
    }
}


// Таблица задач без остановки планировщика (ТЗ v0.9I §19).
//
// vTaskList и uxTaskGetSystemState для КАЖДОЙ задачи побайтно сканируют стек,
// и делают это, пока планировщик остановлен. Измерено: десять вызовов прежней
// команды tasks дали пять пропущенных периодов контура с провалами до 6.8 мс.
// Здесь водяной знак считается по одной задаче через
// uxTaskGetStackHighWaterMark: он читает память стека, не останавливая никого.
static void print_task_table(void) {
    static const char STATE[] = {'X', 'R', 'B', 'S', 'D', '?'};
    printf("\nname             state prio  стек свободно  core\n");
    const unsigned named = CPU_N - 2u;
    for (unsigned i = 0; i < CPU_N; ++i) {
        TaskHandle_t h = i < named ? xTaskGetHandle(CPU_TASKS[i])
                                   : xTaskGetIdleTaskHandleForCore((BaseType_t) (i - named));
        if (!h) {
            continue;
        }
        eTaskState s = eTaskGetState(h);
        BaseType_t core = xTaskGetCoreID(h);
        printf("%-16s   %c   %3u   %8u B    %s\n", pcTaskGetName(h),
               STATE[(unsigned) s < sizeof(STATE) ? (unsigned) s : sizeof(STATE) - 1u],
               (unsigned) uxTaskPriorityGet(h), (unsigned) uxTaskGetStackHighWaterMark(h),
               core == 0 ? "0" : (core == 1 ? "1" : "любое"));
    }
}

static void cmd_cpu(const char *arg) {
    uint32_t window_ms = 2000;
    if (arg && *arg) {
        uint32_t v = (uint32_t) strtoul(arg, NULL, 10);
        if (v >= 200 && v <= 20000) {
            window_ms = v;
        }
    }
    static TaskHandle_t h[CPU_N];
    static uint32_t a[CPU_N], b[CPU_N];
    const unsigned named = CPU_N - 2u;
    for (unsigned i = 0; i < named; ++i) {
        h[i] = xTaskGetHandle(CPU_TASKS[i]);
    }
    h[named] = xTaskGetIdleTaskHandleForCore(0);
    h[named + 1u] = xTaskGetIdleTaskHandleForCore(1);

    uint64_t t0 = fc_uptime_us();
    cpu_sample(h, a);
    vTaskDelay(pdMS_TO_TICKS(window_ms));
    cpu_sample(h, b);
    uint64_t per_core = fc_uptime_us() - t0;
    if (per_core == 0) {
        return;
    }
    printf("доля процессора за %" PRIu32 " мс (каждое ядро = 100 %%, стек не сканируется):\n",
           window_ms);
    for (unsigned i = 0; i < CPU_N; ++i) {
        if (!h[i]) {
            continue;
        }
        TaskStatus_t st;
        vTaskGetInfo(h[i], &st, pdFALSE, eRunning);
        double pct = 100.0 * (double) (b[i] - a[i]) / (double) per_core;
        if (pct < 0.05) {
            continue;
        }
        int core = (int) st.xCoreID;
        printf("  %-16s ядро %-5s приоритет %2u  %6.2f %%\n", st.pcTaskName,
               (core == 0 || core == 1) ? (core ? "1" : "0") : "любое",
               (unsigned) st.uxCurrentPriority, pct);
    }
    printf("  ------------------------------------------------\n");
    printf("  простой ядра 0 %6.2f %%,  ядра 1 %6.2f %%\n",
           100.0 * (double) (b[named] - a[named]) / (double) per_core,
           100.0 * (double) (b[named + 1u] - a[named + 1u]) / (double) per_core);
}


// ---------------------------------------------- зонд шины I²C (ТЗ v0.9I §3, §5, §9)
//
// Что меряется. Время вызова API чтения N байт при разных N. На проводе оно
// растёт строго линейно: каждый байт — это 9 тактов SCL. Поэтому
//
//     T(N) = a + b·N
//
// где наклон b — время одного байта НА ПРОВОДЕ с учётом всего, что удлиняет
// такт (в том числе медленных фронтов), а свободный член a — фиксированная
// часть: три служебных байта (адрес+W, регистр, адрес+R), START, повторный
// START, STOP и собственные накладные расходы драйвера.
//
// Почему это отвечает на вопрос без осциллографа. «Шина медленная» и «драйвер
// дорогой» раньше были неотличимы: оба дают одни и те же 900 мкс. Здесь первое
// меняет наклон, второе — только свободный член.
//
// Почему минимум, а не среднее. Шину делит задача датчика; если она держит
// блокировку, вызов зонда ждёт, и это ожидание к проводу отношения не имеет.
// Минимум из многих повторов — время вызова без чужого ожидания.
#define PROBE_REPS 100
#define PROBE_LENS 7

static int probe_cmp(const void *a, const void *b) {
    uint32_t x = *(const uint32_t *) a, y = *(const uint32_t *) b;
    return x < y ? -1 : x > y;
}

// Результаты развёртки. Статические: зонд может исполняться в отдельной задаче
// на другом ядре, а печатает их всегда консоль — печать из задачи с высоким
// приоритетом на ядре реального времени сама исказила бы замер.
static struct {
    int ok;               // 0 — не начат, 1 — готов, <0 — отказ
    esp_err_t err;
    int core;
    double mn[PROBE_LENS], med[PROBE_LENS];
    uint32_t p90[PROBE_LENS], mx[PROBE_LENS];
    SemaphoreHandle_t done;
} PROBE;

static const uint8_t PROBE_LEN_TAB[PROBE_LENS] = {1, 2, 4, 8, 14, 20, 26};

static void probe_sweep(void) {
    static uint32_t t[PROBE_REPS];
    uint8_t buf[32];
    PROBE.core = xPortGetCoreID();
    for (int k = 0; k < PROBE_LENS; ++k) {
        int ok = 0;
        for (int r = 0; r < PROBE_REPS; ++r) {
            uint32_t us = 0;
            esp_err_t e = icm20948_probe_read(0x2D, buf, PROBE_LEN_TAB[k], &us);
            if (e == ESP_ERR_INVALID_STATE) {
                PROBE.ok = -1;
                PROBE.err = e;
                return;
            }
            if (e == ESP_OK) {
                t[ok++] = us;
            }
            // Шина общая с задачей датчика: оставляем ей окно на каждом шаге.
            vTaskDelay(1);
        }
        if (ok < PROBE_REPS / 2) {
            PROBE.ok = -2;
            return;
        }
        qsort(t, (size_t) ok, sizeof(t[0]), probe_cmp);
        PROBE.mn[k] = t[0];
        PROBE.med[k] = t[ok / 2];
        PROBE.p90[k] = t[(ok * 9) / 10];
        PROBE.mx[k] = t[ok - 1];
    }
    PROBE.ok = 1;
}

static void probe_task(void *arg) {
    (void) arg;
    probe_sweep();
    xSemaphoreGive(PROBE.done);
    vTaskDelete(NULL);
}

// i2c-probe [ядро]. Без аргумента — в задаче консоли (ядро 0). С аргументом —
// в отдельной задаче, закреплённой за указанным ядром, с приоритетом между
// Refloat Main (12) и задачей датчика (14): иначе на загруженном ядре 1 зонд
// не получил бы процессор вовсе. Только для диагностики в LAB_SAFE.
static void cmd_i2c_probe(const char *arg) {
    icm20948_scl_timing_t tm;
    icm20948_scl_timing(&tm);
    printf("=== такт SCL из РЕГИСТРОВ периферии I2C0 ===========================\n");
    printf("источник          %" PRIu32 " Гц (APB)\n", tm.source_hz);
    printf("scl_low_period    %d   scl_high_period %d   scl_wait_high %d\n", tm.scl_low_period,
           tm.scl_high_period, tm.scl_wait_high);
    printf("фильтр SCL        %s, порог %d тактов;  фильтр SDA %s, порог %d\n",
           tm.scl_filter_en ? "вкл" : "выкл", tm.scl_filter_thres, tm.sda_filter_en ? "вкл" : "выкл",
           tm.sda_filter_thres);
    printf("запрошено         %" PRIu32 " Гц\n", tm.requested_hz);
    printf("по регистрам      %" PRIu32 " Гц   <- без учёта нарастания фронтов\n", tm.programmed_hz);

    memset(&PROBE, 0, sizeof(PROBE));
    if (arg && (*arg == '0' || *arg == '1')) {
        int core = *arg - '0';
        PROBE.done = xSemaphoreCreateBinary();
        if (!PROBE.done || xTaskCreatePinnedToCore(probe_task, "fc_i2c_probe", 4096, NULL, 13, NULL,
                                                   core) != pdPASS) {
            printf("не удалось запустить зонд на ядре %d\n", core);
            return;
        }
        xSemaphoreTake(PROBE.done, portMAX_DELAY);
        vSemaphoreDelete(PROBE.done);
    } else {
        probe_sweep();
    }
    if (PROBE.ok == -1) {
        printf("ОТКЛОНЕНО: банк датчика не 0 — зонд не будет переключать его сам\n");
        return;
    }
    if (PROBE.ok != 1) {
        printf("зонд прерван: слишком мало успешных чтений\n");
        return;
    }

    printf("\n=== развёртка по длине чтения, ЯДРО %d (по %d повторов) ==============\n",
           PROBE.core, PROBE_REPS);
    for (int k = 0; k < PROBE_LENS; ++k) {
        printf("  N=%2u байт   min %5.0f   p50 %5.0f   p90 %5" PRIu32 "   max %5" PRIu32 " мкс\n",
               PROBE_LEN_TAB[k], PROBE.mn[k], PROBE.med[k], PROBE.p90[k], PROBE.mx[k]);
    }

    // Та же арифметика, что проверена тестами на хосте (tests/safety/test_i2c_fit.c).
    FcI2cFit f = fc_i2c_fit(PROBE_LEN_TAB, PROBE.mn, PROBE_LENS);
    if (!f.valid) {
        printf("\nподгонка невозможна: данных недостаточно\n");
        return;
    }
    printf("\n=== подгонка T(N) = a + b*N по минимумам ============================\n");
    printf("наклон b          %.2f мкс на байт  (при 400 кГц было бы 22.50)\n", f.slope_us_per_byte);
    printf("ЧАСТОТА НА ПРОВОДЕ %.0f Гц  (9 тактов на байт, с учётом фронтов)\n", f.wire_hz);
    printf("свободный член a  %.0f мкс\n", f.intercept_us);
    printf("  из них провод   ~%.0f мкс  (служебные 3 байта и START/rSTART/STOP)\n", f.fixed_wire_us);
    printf("  накладные API   ~%.0f мкс  (драйвер, блокировки, прерывания)\n", f.overhead_us);
    printf("худшее отклонение от прямой %.0f мкс — %s\n", f.max_residual_us,
           f.linear ? "зависимость линейна, модель годна"
                    : "НЕЛИНЕЙНО: в минимумы попало ожидание, наклон НЕ скорость провода");
    printf("\nдля N=14 (реальное чтение): провод ~%.0f мкс, накладные ~%.0f мкс, всего %.0f мкс\n",
           f.fixed_wire_us + 14 * f.slope_us_per_byte, f.overhead_us,
           f.intercept_us + 14 * f.slope_us_per_byte);
}


// Таблица выделенных прерываний (ТЗ v0.9I §10, §16). Нужна, чтобы узнать, на
// каком ядре обслуживается I²C: драйвер размещает обработчик на ядре, которое
// создало шину, а шину создаёт старт задачи датчика — не сама задача.
static void cmd_intr(void) {
    esp_intr_dump(stdout);
}


static void cmd_i2c_split(void) {
    FcImuI2cSplit sp = fc_imu_rt_i2c_split();
    printf("вызов I²C в задаче датчика (ядро 1), вызовов %llu:\n", (unsigned long long) sp.calls);
    printf("  настенное время          %7.1f мкс\n", sp.wall_mean_us);
#if FC_DIAG_LOOP_PROBES
    printf("  получили ДРУГИЕ задачи   %7.1f мкс  (Refloat Main, простой ядра 1, супервизор)\n",
           sp.others_mean_us);
    printf("  держала сама задача      %7.1f мкс\n", sp.own_mean_us);
#else
    printf("  раскладка по ядру — только в сборке с -DFC_DIAG_LOOP_PROBES=1\n");
#endif
    printf("  распределение настенного времени вызова:\n");
    for (unsigned b = 0; b < FC_IMU_I2C_HIST_BINS; ++b) {
        if (sp.hist[b] == 0) {
            continue;
        }
        unsigned w = (unsigned) (sp.calls ? (50ull * sp.hist[b]) / sp.calls : 0);
        printf("    %4u…%4u мкс %8" PRIu32 " ", b * 50u, (b + 1u) * 50u, sp.hist[b]);
        for (unsigned k = 0; k < w; ++k) {
            putchar('#');
        }
        putchar('\n');
    }
    printf("  Провод на 14 байт ~412 мкс. Если «другие» близко к нему — задача на\n");
    printf("  проводе блокируется; если около нуля — держит процессор.\n");
    fc_imu_rt_i2c_split_reset();
    printf("  (счётчики сброшены)\n");
}

static void cmd_gain_noi(void) {
    if (!refloat_facade_disable_integral()) {
        printf("ОТКЛОНЕНО: Refloat не запущен\n");
        return;
    }
    printf("интегральная часть отключена (ki = 0, ki_limit = 0).\n");
    printf("  В теневом режиме контур разомкнут, и интеграл упирается в предел\n");
    printf("  за секунды: мерить по нему масштаб нечего. ВЕРНУТЬ: gain-restore\n");
    cmd_gains();
}

static void cmd_gain_restore(void) {
    if (!refloat_facade_restore_gains()) {
        printf("восстанавливать нечего: коэффициенты не менялись\n");
        return;
    }
    printf("исходные коэффициенты восстановлены\n");
    cmd_gains();
}

static void dispatch(const char *line) {
    if (!strcmp(line, "status")) {
        cmd_status();
    } else if (!strcmp(line, "supervisor")) {
        cmd_supervisor();
    } else if (!strcmp(line, "imu")) {
        cmd_imu();
    } else if (!strncmp(line, "cpu", 3) && (line[3] == 0 || line[3] == ' ')) {
        cmd_cpu(line[3] ? line + 4 : NULL);
    } else if (!strcmp(line, "wake-snap")) {
        FcRtClockStats rc = fc_rt_clock_stats();
        printf("аппаратный ритм: %s, ядро %d, период %" PRIu32 " мкс, срабатываний %llu, "
               "ожиданий без ритма %llu\n", rc.running ? "РАБОТАЕТ" : "НЕ ЗАПУЩЕН", rc.core,
               rc.period_us, (unsigned long long) rc.alarms, (unsigned long long) rc.wait_timeouts);
#if FC_DIAG_LOOP_PROBES
        fc_imu_rt_wake_snapshot_print();
        fc_imu_rt_wake_snapshot_reset();
#else
        printf("перекрёстный снимок — только в сборке с -DFC_DIAG_LOOP_PROBES=1\n");
#endif
    } else if (!strcmp(line, "i2c-split")) {
        cmd_i2c_split();
    } else if (!strcmp(line, "intr")) {
        cmd_intr();
    } else if (!strncmp(line, "i2c-probe", 9) && (line[9] == 0 || line[9] == ' ')) {
        cmd_i2c_probe(line[9] ? line + 10 : NULL);
    } else if (!strcmp(line, "tasks")) {
        cmd_tasks();
    } else if (!strcmp(line, "sched")) {
        cmd_sched();
    } else if (!strcmp(line, "sched-reset")) {
        fc_sched_reset();
        printf("sched: статистика обнулена\n");
#if FC_CAN_RX_AVAILABLE
    } else if (!strcmp(line, "can")) {
        cmd_can();
    } else if (!strcmp(line, "can-frames")) {
        cmd_can_frames();
    } else if (!strcmp(line, "can-reset")) {
        fc_can_bus_reset_stats();
        printf("can: статистика обнулена\n");
    } else if (!strcmp(line, "log")) {
        cmd_log();
    } else if (!strcmp(line, "can-health")) {
        cmd_can_health();
#if FC_CAN_DIAG_TX_AVAILABLE
    } else if (!strcmp(line, "vesc-values")) {
        cmd_vesc_values();
    } else if (!strncmp(line, "can-diag-hex ", 13)) {
        cmd_can_diag(line + 13, true);
    } else if (!strncmp(line, "can-diag ", 9)) {
        cmd_can_diag(line + 9, false);
    } else if (!strncmp(line, "can-diag-poll ", 14)) {
        cmd_can_diag_poll(line + 14);
    } else if (!strcmp(line, "can-diag-stats")) {
        print_diag_stats();
    } else if (!strcmp(line, "can-diag-reset")) {
        fc_can_bus_diag_reset_stats();
        printf("диагностика CAN: статистика обнулена\n");
#endif
#endif
    } else if (!strcmp(line, "i2cscan")) {
        cmd_i2cscan();
    } else if (!strcmp(line, "imu-cal-show")) {
        cmd_imu_cal_show();
    } else if (!strcmp(line, "timing")) {
        cmd_timing();
    } else if (!strcmp(line, "timing-hist")) {
        cmd_timing_hist();
    } else if (!strcmp(line, "heap")) {
        cmd_heap();
    } else if (!strcmp(line, "config")) {
        cmd_config();
    } else if (!strcmp(line, "safety")) {
        cmd_safety();
#if FC_LAB_DIAGNOSTICS
    } else if (!strcmp(line, "ready")) {
        cmd_ready();
    } else if (!strcmp(line, "disarm")) {
        cmd_disarm();
    } else if (!strcmp(line, "fault-clear")) {
        cmd_fault_clear();
    } else if (!strncmp(line, "tiltback", 8)) {
        cmd_tiltback(line[8] == ' ' ? line + 9 : "");
    } else if (!strcmp(line, "persist")) {
        cmd_persist();
    } else if (!strcmp(line, "timing-reset")) {
        cmd_timing_reset();
    } else if (!strcmp(line, "restart")) {
        cmd_restart();
    } else if (!strcmp(line, "wdtest-confirm")) {
        cmd_wdtest();
    } else if (!strcmp(line, "crashtest-confirm")) {
        cmd_crashtest();
    } else if (!strcmp(line, "imu-stop-confirm")) {
        cmd_imu_stop();
    } else if (!strcmp(line, "imu-start")) {
        cmd_imu_start();
    } else if (!strcmp(line, "imu-fail-confirm")) {
        cmd_imu_fail();
    } else if (!strcmp(line, "imu-freeze-confirm")) {
        cmd_imu_freeze();
    } else if (!strncmp(line, "imu-cal-detect", 14) &&
               (line[14] == 0 || line[14] == ' ')) {
        cmd_imu_cal_detect(line[14] == ' ' ? line + 15 : NULL);
#if FC_CAN_RX_AVAILABLE
    } else if (!strcmp(line, "can-stop")) {
        cmd_can_stop();
    } else if (!strcmp(line, "can-start")) {
        cmd_can_start();
#endif
    } else if (!strcmp(line, "imu-cal-save")) {
        cmd_imu_cal_save();
    } else if (!strcmp(line, "imu-cal-clear")) {
        cmd_imu_cal_clear();
    } else if (!strcmp(line, "footpad-sim-confirm")) {
        cmd_footpad_sim(true);
    } else if (!strcmp(line, "footpad-sim-off")) {
        cmd_footpad_sim(false);
    } else if (!strcmp(line, "imu_stress-stop")) {
        cmd_imu_stress_stop();
    } else if (!strcmp(line, "imu_stress-log")) {
        fc_imu_stress_print_log();
    } else if (!strncmp(line, "imu_stress", 10) &&
               (line[10] == 0 || line[10] == ' ')) {
        cmd_imu_stress(line[10] == ' ' ? line + 11 : NULL);
#endif
    } else if (!strncmp(line, "gaps", 4) && (line[4] == 0 || line[4] == ' ')) {
        cmd_gaps(line[4] == ' ' ? line + 5 : NULL);
    } else if (!strcmp(line, "gaps-reset")) {
        cmd_gaps_reset();
    } else if (!strcmp(line, "shadow")) {
        cmd_shadow();
    } else if (!strcmp(line, "shadow-reset")) {
        cmd_shadow_reset();
    } else if (!strcmp(line, "gain-no-integral")) {
        cmd_gain_noi();
    } else if (!strcmp(line, "gain-restore")) {
        cmd_gain_restore();
    } else if (!strncmp(line, "gain-scale", 10) && (line[10] == 0 || line[10] == ' ')) {
        cmd_gain_scale(line[10] == ' ' ? line + 11 : NULL);
    } else if (!strcmp(line, "gains")) {
        cmd_gains();
    } else if (!strcmp(line, "limits")) {
        cmd_limits();
    } else if (!strcmp(line, "limits-sync")) {
        cmd_limits_sync();
    } else if (!strncmp(line, "shadow-tail", 11) && (line[11] == 0 || line[11] == ' ')) {
        cmd_shadow_tail(line[11] == ' ' ? line + 12 : NULL);
#if FC_LAB_DIAGNOSTICS
    } else if (!strcmp(line, "imu-policy")) {
        cmd_imu_policy();
    } else if (!strncmp(line, "imu-accel-low-confirm", 21) &&
               (line[21] == 0 || line[21] == ' ')) {
        cmd_imu_accel_low(line[21] == ' ' ? line + 22 : NULL);
#endif
#if FC_LAB_DIAGNOSTICS
    } else if (!strncmp(line, "imu-stall-confirm", 17) &&
               (line[17] == 0 || line[17] == ' ')) {
        cmd_imu_stall(line[17] == ' ' ? line + 18 : NULL);
#endif
#if FC_MOTOR_BACKEND_AVAILABLE
    } else if (!strcmp(line, "motor-status")) {
        cmd_motor_status();
    } else if (!strcmp(line, "motor-oneshot-prep")) {
        cmd_oneshot_prep();
    } else if (!strcmp(line, "motor-oneshot-fire")) {
        cmd_oneshot_fire();
    } else if (!strcmp(line, "motor-oneshot-report")) {
        cmd_oneshot_report();
    } else if (!strcmp(line, "motor-arm")) {
        cmd_motor_arm();
    } else if (!strcmp(line, "motor-disarm")) {
        cmd_motor_disarm();
    } else if (!strcmp(line, "motor-clear-latch")) {
        cmd_motor_clear();
    } else if (!strcmp(line, "motor-stop")) {
        cmd_motor_stop();
    } else if (!strcmp(line, "motor-reset-stats")) {
        cmd_motor_reset();
    } else if (!strncmp(line, "motor-run", 9) && (line[9] == 0 || line[9] == ' ')) {
        cmd_motor_run(line[9] == ' ' ? line + 10 : NULL);
    } else if (!strncmp(line, "motor-inject", 12) && (line[12] == 0 || line[12] == ' ')) {
        cmd_motor_inject(line[12] == ' ' ? line + 13 : NULL);
#endif
    } else {
        cmd_help();
    }
}


static void console_task(void *arg) {
    (void) arg;
    char line[64];
    size_t len = 0;

    for (;;) {
        int c = fgetc(stdin);
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        if (c == '\r' || c == '\n') {
            line[len] = 0;
            if (len) {
                printf("\n");
                dispatch(line);
                fflush(stdout);
            }
            len = 0;
            printf("floatcore> ");
            fflush(stdout);
            continue;
        }
        if (len + 1 < sizeof(line)) {
            line[len++] = (char) c;
        }
    }
}

void fc_console_start(void) {
    setvbuf(stdin, NULL, _IONBF, 0);
    fcntl(fileno(stdin), F_SETFL, fcntl(fileno(stdin), F_GETFL) | O_NONBLOCK);
    // 8192, а не 4096 (ТЗ v0.9J): отчёт одношага с 4 КБ переполнил стек —
    // аппаратная точка останова на конце стека поймала запись в printf с
    // FcCanStats (таблица идентификаторов, больше килобайта) на стеке. Крупные
    // структуры в одношаге теперь статические, а запас нужен и для printf с
    // плавающей точкой. Ядро 0 при этом простаивает, свободной кучи ~120 КБ.
    xTaskCreatePinnedToCore(console_task, "fc_console", 8192, NULL, FC_PRIO_CONSOLE, NULL,
                            FC_CORE_HOUSEKEEPING);
}
