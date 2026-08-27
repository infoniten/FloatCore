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
#include "../drivers/icm20948.h"

#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <fcntl.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// ============================================================ SAFE_READONLY

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
    printf("pitch / roll      %.2f / %.2f deg (mock IMU)\n", (double) s.pitch, (double) s.roll);
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
    printf("motor output      %s\n",
           fc_supervisor_motor_output_permitted() ? "РАЗРЕШЁН" : "запрещён");
    printf("config write      %s\n",
           fc_supervisor_config_write_allowed() ? "разрешена" : "запрещена");
    printf("входы:\n");
    printf("  platform_init   %d\n", st.inputs.platform_initialized);
    printf("  config_valid    %d\n", st.inputs.config_valid);
    printf("  loop_alive      %d (последний тик %.1f ms назад)\n", st.inputs.loop_alive,
           st.last_loop_tick_us ? (double) (now - st.last_loop_tick_us) / 1000.0 : -1.0);
    printf("  imu_healthy     %d (последний семпл %.1f ms назад)\n", st.inputs.imu_healthy,
           st.last_imu_sample_us ? (double) (now - st.last_imu_sample_us) / 1000.0 : -1.0);
    printf("  watchdog        %d\n", st.inputs.watchdog_healthy);
    printf("  footpad_engaged %d\n", st.inputs.footpad_engaged);
    printf("входы под будущие этапы (CAN ещё нет):\n");
    printf("  esc_a/esc_b     %d / %d\n", st.inputs.esc_a_alive, st.inputs.esc_b_alive);
    printf("  can_fresh       %d\n", st.inputs.can_fresh);
    printf("  battery/thermal %d / %d\n", st.inputs.battery_ok, st.inputs.thermal_ok);
}

static void cmd_imu(void) {
    const icm20948_config_t *cfg = icm20948_active_config();
    icm20948_stats_t st = icm20948_stats();
    FcImuHealthStatus h = fc_imu_health_status();

    printf("драйвер           %s\n", fc_imu_real_available() ? "ICM-20948 работает" : "не поднят");
    if (!fc_imu_real_available()) {
        printf("последний шаг     %s\n", icm20948_last_stage());
    }
    printf("инициализация     повторов записи %" PRIu32 ", ожидание после сброса %" PRIu32 " мс\n",
           icm20948_init_retries(), icm20948_reset_wait_us() / 1000);
    printf("шина              SDA=GPIO%d SCL=GPIO%d, %" PRIu32 " Гц, адрес 0x%02x\n",
           cfg->sda_gpio, cfg->scl_gpio, cfg->i2c_hz, cfg->i2c_addr);
    printf("шкалы             accel ±%.0f g (%.0f LSB/g), gyro ±%.0f °/с (%.1f LSB/(°/с))\n",
           (double) icm20948_accel_fs_g(cfg->accel_fs),
           (double) icm20948_accel_lsb_per_g(cfg->accel_fs),
           (double) icm20948_gyro_fs_dps(cfg->gyro_fs),
           (double) icm20948_gyro_lsb_per_dps(cfg->gyro_fs));
    printf("ODR               %.1f Гц (SMPLRT_DIV=%u), DLPF cfg %u\n",
           (double) icm20948_odr_hz(cfg->smplrt_div), cfg->smplrt_div, cfg->dlpf_cfg);
    printf("транзакции        ok=%llu failed=%llu, средняя %.1f мкс, худшая %" PRIu32 " мкс\n",
           (unsigned long long) st.reads_ok, (unsigned long long) st.reads_failed,
           st.reads_ok ? (double) st.sum_transaction_us / (double) st.reads_ok : 0.0,
           st.max_transaction_us);
    printf("health            %s (семплов %llu, ошибок %llu, stuck %llu, stale %llu, timeout %llu)\n",
           fc_imu_health_state_name(h.state), (unsigned long long) h.samples_total,
           (unsigned long long) h.read_errors, (unsigned long long) h.stuck_events,
           (unsigned long long) h.stale_events, (unsigned long long) h.timeout_events);

    // Сырые значения в осях датчика. Никакого пересчёта в оси доски здесь
    // нет и на этом этапе быть не должно (ТЗ v0.6A §4).
    const FcImuRawSample *g = &h.last_good;
    float mag = sqrtf(g->accel_g[0] * g->accel_g[0] + g->accel_g[1] * g->accel_g[1] +
                      g->accel_g[2] * g->accel_g[2]);
    printf("последний семпл   acc %+7.3f %+7.3f %+7.3f g |a|=%.3f\n", (double) g->accel_g[0],
           (double) g->accel_g[1], (double) g->accel_g[2], (double) mag);
    printf("                  gyro %+8.2f %+8.2f %+8.2f °/с, %.1f °C\n", (double) g->gyro_dps[0],
           (double) g->gyro_dps[1], (double) g->gyro_dps[2], (double) g->temperature_c);
    printf("оси               СЫРЫЕ, в системе координат датчика. Привязка к доске отложена\n");
    printf("                  до финального монтажа (docs/esp32_architecture.md, v0.6B)\n");
    printf("в Refloat         НЕ передаются: контур работает от mock (ТЗ v0.6A §29)\n");
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
    if (fc_imu_real_running()) {
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

static void cmd_tasks(void) {
    printf("задачи FloatCore:\n");
    printf("  %-14s свободно минимум %u B из 4096 (контур, mock IMU)\n", "fc_imu",
           (unsigned) fc_imu_stack_watermark());
    printf("  %-14s свободно минимум %u B из 4096 (чтение ICM-20948)\n", "fc_imu_hw",
           (unsigned) fc_imu_real_stack_watermark());
    printf("  %-14s свободно минимум %u B из 4096 (supervisor)\n", "fc_super",
           (unsigned) fc_supervisor_stack_watermark());
    printf("  %-14s свободно минимум %u B из 3072 (хранилище)\n", "fc_nvs",
           (unsigned) fc_storage_stack_watermark());
    for (size_t i = 0; i < fc_thread_count(); ++i) {
        printf("  %-14s свободно минимум %u B из 12288 (задача Refloat)\n", fc_thread_name(i),
               (unsigned) fc_thread_stack_watermark(i));
    }
#if CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS
    static char buf[1024];
    vTaskList(buf);
    printf("\nname          state prio stack  num core\n%s", buf);
#endif
}

static void print_timing(FcTimingChannel ch) {
    FcTimingStats t = fc_timing_get(ch);
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
               " us\n",
               (double) t.exec_sum_us / (double) t.exec_samples, t.exec_p99_us, t.exec_min_us,
               t.exec_max_us);
    }
}

static void cmd_timing(void) {
    printf("периодичность (esp_timer, отметка в момент пробуждения задачи):\n");
    for (int i = 0; i < FC_TIMING_COUNT; ++i) {
        print_timing((FcTimingChannel) i);
    }
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
           fc_supervisor_config_write_allowed() ? "разрешена" : "ЗАПРЕЩЕНА",
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
           fc_supervisor_config_write_allowed() ? "разрешена" : "запрещена");
    printf("  выход на мотор: %s (в LAB_SAFE не разрешает ни одно состояние)\n",
           fc_supervisor_motor_output_permitted() ? "РАЗРЕШЁН" : "запрещён");
}

static void cmd_disarm(void) {
    fc_supervisor_disarm(fc_uptime_us());
    printf("supervisor: состояние %s, запись конфигурации %s\n",
           fc_supervisor_state_name(fc_supervisor_state()),
           fc_supervisor_config_write_allowed() ? "разрешена" : "запрещена");
}

static void cmd_fault_clear(void) {
    bool ok = fc_supervisor_clear_fault(fc_uptime_us());
    printf("supervisor: снятие отказа %s, состояние %s, активные причины %s\n",
           ok ? "выполнено" : "ОТКЛОНЕНО (причина всё ещё активна)",
           fc_supervisor_state_name(fc_supervisor_state()),
           fc_supervisor_fault_name(fc_supervisor_status().faults));
}

static void cmd_persist(void) {
    float before = refloat_facade_config_test_value();
    float next = (before >= 0.9f) ? 0.25f : before + 0.1f;
    printf("persist: leds.status.brightness_headlights_off %.3f -> %.3f\n", (double) before,
           (double) next);
    printf("persist: политика записи сейчас %s (supervisor %s)\n",
           fc_supervisor_config_write_allowed() ? "разрешает" : "ЗАПРЕЩАЕТ",
           fc_supervisor_state_name(fc_supervisor_state()));
    bool ok = refloat_facade_config_save_test(next);
    printf("persist: set_cfg вернул %s\n", ok ? "true" : "false");
    FcStorageStats st = fc_storage_stats();
    printf("persist: записей принято %llu, отклонено %llu\n",
           (unsigned long long) st.writes_accepted, (unsigned long long) st.writes_rejected);
    printf("persist: значение в конфигурации сейчас %.3f\n",
           (double) refloat_facade_config_test_value());
    printf("persist: коммит выполняет отдельная задача хранилища, не контур\n");
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
    fc_imu_real_stop();
    printf("imu-stop: задача чтения ICM-20948 останавливается\n");
    printf("  контур это не затрагивает: он работает от mock\n");
}

static void cmd_imu_start(void) {
    bool ok = fc_imu_real_start();
    printf("imu-start: %s\n", ok ? "задача чтения запущена" : "датчик не отвечает");
}

// Стресс-тест шины I2C (ТЗ v0.6C). Категория STATE_CHANGING: замещает
// штатную задачу чтения датчика, но физически безопасен — путей к мотору в
// нём нет, а Refloat всё это время работает от mock-IMU.
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
    printf("                         tasks | heap | config | safety | help\n");
#if FC_LAB_DIAGNOSTICS
    printf("меняют состояние:        ready | disarm | fault-clear | persist | timing-reset |\n");
    printf("                         restart\n");
    printf("проверка защит:          wdtest-confirm | crashtest-confirm | imu-fail-confirm |\n");
    printf("                         imu-freeze-confirm | imu-stop-confirm | imu-start\n");
    printf("стресс-тест шины I2C:    imu_stress [сек] [кГц] [порог_сброса] | imu_stress-stop |\n");
    printf("                         imu_stress-log\n");
#endif
    printf("профиль сборки:          %s\n", FC_PROFILE_NAME);
    printf("команд управления мотором нет: в этой сборке нет кода, способного что-либо\n");
    printf("отправить — см. docs/esp32_safety.md\n");
}

static void dispatch(const char *line) {
    if (!strcmp(line, "status")) {
        cmd_status();
    } else if (!strcmp(line, "supervisor")) {
        cmd_supervisor();
    } else if (!strcmp(line, "imu")) {
        cmd_imu();
    } else if (!strcmp(line, "tasks")) {
        cmd_tasks();
    } else if (!strcmp(line, "i2cscan")) {
        cmd_i2cscan();
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
    } else if (!strcmp(line, "imu_stress-stop")) {
        cmd_imu_stress_stop();
    } else if (!strcmp(line, "imu_stress-log")) {
        fc_imu_stress_print_log();
    } else if (!strncmp(line, "imu_stress", 10) &&
               (line[10] == 0 || line[10] == ' ')) {
        cmd_imu_stress(line[10] == ' ' ? line + 11 : NULL);
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
    xTaskCreatePinnedToCore(console_task, "fc_console", 4096, NULL, FC_PRIO_CONSOLE, NULL,
                            FC_CORE_HOUSEKEEPING);
}
