// Read-only диагностическая консоль FloatCore (ТЗ §15).
//
// Намеренно примитивна: одна строка — одна команда, разбор без библиотеки
// console/argtable. Команд управления мотором нет и быть не может — этот файл
// не имеет доступа ни к одной функции, способной что-то запросить у мотора.
//
// Доступно: status, tasks, timing, heap, config, safety, help.

#include "fc_platform.h"
#include "../../../compat/refloat_glue/refloat_facade.h"

#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

static void cmd_status(void) {
    RefloatSnapshot s = refloat_facade_snapshot();
    printf("uptime            %.1f s\n", (double) fc_uptime_us() * 1e-6);
    printf("reset reason      %s\n", fc_reset_reason_name());
    printf("boot #            %u (счётчик в NVS)\n", (unsigned) fc_boot_count());
    printf("refloat state     %s\n", refloat_facade_state_name(s.state));
    printf("stop condition    %s\n", refloat_facade_stop_name(s.stop_condition));
    printf("footpad           %s (adc %.2f / %.2f V)\n", refloat_facade_footpad_name(s.footpad_state),
           (double) s.adc_left, (double) s.adc_right);
    printf("pitch / roll      %.2f / %.2f deg\n", (double) s.pitch, (double) s.roll);
    printf("balance pitch     %.2f deg\n", (double) s.balance_pitch);
    printf("imu / main freq   %.1f / %.1f Hz (по счётчикам Refloat)\n",
           (double) s.imu_frequency, (double) s.main_frequency);
    printf("motor backend     %s\n", fc_motor_backend_name());
    printf("can backend       %s\n", fc_can_backend_name());
}

static void cmd_tasks(void) {
    printf("задачи Refloat (созданы через VESC_IF->spawn) и контур:\n");
    printf("  %-14s свободно минимум %u B из 4096 (контур управления)\n", "fc_imu",
           (unsigned) fc_imu_stack_watermark());
    for (size_t i = 0; i < fc_thread_count(); ++i) {
        printf("  %-14s свободно минимум %u B из 12288\n", fc_thread_name(i),
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
    printf("  %-26s iters %-8llu mean %8.1f us  min %6u  max %6u  late %u  (номинал %u us)\n",
           t.name, (unsigned long long) t.iterations, mean, (unsigned) t.min_period_us,
           (unsigned) t.max_period_us, (unsigned) t.late, (unsigned) t.nominal_period_us);
}

static void cmd_timing(void) {
    printf("периодичность (esp_timer, монотонный, 1 мкс):\n");
    for (int i = 0; i < FC_TIMING_COUNT; ++i) {
        print_timing((FcTimingChannel) i);
    }
}

static void cmd_heap(void) {
    printf("free heap         %u B\n", (unsigned) esp_get_free_heap_size());
    printf("min free heap     %u B\n", (unsigned) esp_get_minimum_free_heap_size());
    printf("largest block     %u B\n",
           (unsigned) heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    printf("internal free     %u B\n",
           (unsigned) heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
}

static void cmd_config(void) {
    RefloatSnapshot s = refloat_facade_snapshot();
    printf("custom config     %s\n", fc_config_registered() ? "зарегистрирован" : "НЕТ");
    printf("storage слов      %d (NVS)\n", fc_storage_capacity());
    printf("fault_adc1/2      %.2f / %.2f V\n", (double) s.fault_adc1, (double) s.fault_adc2);
    printf("test param        %.3f (leds.status.brightness_headlights_off)\n",
           (double) refloat_facade_config_test_value());
    printf("пределы тока      %.1f / %.1f A\n", (double) s.motor_current_max,
           (double) s.motor_current_min);
    printf("батарея           %.1f / %.1f A\n", (double) s.motor_batt_current_max,
           (double) s.motor_batt_current_min);
}

static void cmd_safety(void) {
    FcMotorStats m = fc_motor_stats();
    RefloatSnapshot s = refloat_facade_snapshot();
    printf("motor backend     %s\n", fc_motor_backend_name());
    printf("can backend       %s\n", fc_can_backend_name());
    printf("blocked total     %llu\n", (unsigned long long) m.total_blocked);
    printf("  set_current     %llu (последний запрос %.3f A)\n",
           (unsigned long long) m.blocked[FC_MOTOR_CMD_CURRENT],
           (double) m.last_value[FC_MOTOR_CMD_CURRENT]);
    printf("  set_brake       %llu (последний запрос %.3f A)\n",
           (unsigned long long) m.blocked[FC_MOTOR_CMD_BRAKE],
           (double) m.last_value[FC_MOTOR_CMD_BRAKE]);
    printf("  set_duty        %llu (последний запрос %.3f)\n",
           (unsigned long long) m.blocked[FC_MOTOR_CMD_DUTY],
           (double) m.last_value[FC_MOTOR_CMD_DUTY]);
    printf("timeout_reset     %llu\n", (unsigned long long) m.keepalive_calls);
    printf("footpad           %s — %s\n", refloat_facade_footpad_name(s.footpad_state),
           s.footpad_state == 0 ? "disengaged, как и требуется" : "ВНИМАНИЕ: не disengaged");
    printf("adc платформы     %.2f V (порог %.2f/%.2f V)\n", (double) fc_adc_safe_voltage(),
           (double) s.fault_adc1, (double) s.fault_adc2);
}

// Единственная команда, которая что-то меняет. Не имеет отношения к мотору:
// пишет косметический параметр яркости и гоняет его через штатный путь
// сохранения Refloat, чтобы доказать persistence (ТЗ §12).
static void cmd_persist(void) {
    float before = refloat_facade_config_test_value();
    float next = (before >= 0.9f) ? 0.25f : before + 0.1f;
    printf("persist: leds.status.brightness_headlights_off %.3f -> %.3f\n", (double) before,
           (double) next);
    bool ok = refloat_facade_config_save_test(next);
    printf("persist: set_cfg вернул %s\n", ok ? "true" : "false");
    printf("persist: коммит NVS %s\n", fc_storage_commit() ? "выполнен" : "НЕ выполнен");
    printf("persist: значение в конфигурации сейчас %.3f\n",
           (double) refloat_facade_config_test_value());
    printf("persist: перезагрузите плату и вызовите `config` — значение обязано сохраниться\n");
}

// --- диагностические команды, требующие явного подтверждения в имени -------
// Ни одна из них не имеет отношения к мотору. Нужны, чтобы доказать
// требования ТЗ §11 (watchdog) и §14 (panic с backtrace) на живой плате.

static void cmd_wdtest(void) {
    printf("wdtest: контур перестаёт отмечаться в TWDT на 8 с при таймауте %d с\n",
           CONFIG_ESP_TASK_WDT_TIMEOUT_S);
    printf("wdtest: ожидается предупреждение TWDT с backtrace задачи fc_imu\n");
    fc_imu_inject_stall(8000);
}

static void cmd_crashtest(void) {
    printf("crashtest: намеренное разыменование нулевого указателя, panic должен\n");
    printf("           напечатать причину, регистры, backtrace и ОСТАНОВИТЬСЯ\n");
    fflush(stdout);
    volatile int *p = (volatile int *) 0;
    *p = 1;
}

static void cmd_restart(void) {
    printf("restart: esp_restart(), причина следующей загрузки должна быть SW\n");
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
}

static void cmd_help(void) {
    printf("команды: status | tasks | timing | heap | config | safety | persist | restart | help\n");
    printf("диагностика: wdtest-confirm (проверка watchdog), crashtest-confirm (проверка panic)\n");
    printf("команд управления мотором нет и не будет на этом этапе (ТЗ v0.5 §15)\n");
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
                if (!strcmp(line, "status")) {
                    cmd_status();
                } else if (!strcmp(line, "tasks")) {
                    cmd_tasks();
                } else if (!strcmp(line, "timing")) {
                    cmd_timing();
                } else if (!strcmp(line, "heap")) {
                    cmd_heap();
                } else if (!strcmp(line, "config")) {
                    cmd_config();
                } else if (!strcmp(line, "safety")) {
                    cmd_safety();
                } else if (!strcmp(line, "persist")) {
                    cmd_persist();
                } else if (!strcmp(line, "restart")) {
                    cmd_restart();
                } else if (!strcmp(line, "wdtest-confirm")) {
                    cmd_wdtest();
                } else if (!strcmp(line, "crashtest-confirm")) {
                    cmd_crashtest();
                } else {
                    cmd_help();
                }
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
    // stdin по умолчанию блокирующий и построчный; переводим в неблокирующий
    // посимвольный режим, иначе задача заняла бы UART монопольно.
    setvbuf(stdin, NULL, _IONBF, 0);
    fcntl(fileno(stdin), F_SETFL, fcntl(fileno(stdin), F_GETFL) | O_NONBLOCK);
    xTaskCreatePinnedToCore(console_task, "fc_console", 4096, NULL, FC_PRIO_CONSOLE, NULL,
                            FC_CORE_HOUSEKEEPING);
}
