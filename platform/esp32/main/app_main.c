// FloatCore на ESP32 — точка входа (ТЗ v0.5).
//
// Порядок запуска:
//   баннер (данные берутся из runtime API, не захардкожены)
//   → NVS (постоянное хранилище)
//   → VESC_IF (платформенный backend)
//   → mock-IMU (источник ритма контура)
//   → refloat_init() — тот же самый upstream, что и в host-сборке
//   → задача отчётов + read-only консоль
//
// Выход на мотор заблокирован конструктивно, см. fc_motor_blocked.c и
// docs/esp32_safety.md.

#include "fc_platform.h"
#include "../../../compat/refloat_glue/refloat_facade.h"

#include "esp_app_desc.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/efuse_hal.h"
#include "soc/rtc.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "floatcore";

// Причина сброса и номер загрузки нужны не только баннеру: без них проверку
// «плата пережила снятие питания» нельзя сделать постфактум — баннер печатается
// один раз и уходит из буфера UART.
static uint32_t g_boot_count;
static esp_reset_reason_t g_reset_reason;

// Слово хранилища вне диапазона Refloat (0..79) — счётчик загрузок.
// Это и есть «безопасный тестовый параметр» из ТЗ §12: платформенный
// round-trip, который не смешивается с конфигурацией Refloat.
#define FC_BOOT_COUNTER_ADDR 100

static const char *chip_model_name(esp_chip_model_t m) {
    switch (m) {
    case CHIP_ESP32:
        return "ESP32";
    case CHIP_ESP32S2:
        return "ESP32-S2";
    case CHIP_ESP32S3:
        return "ESP32-S3";
    case CHIP_ESP32C3:
        return "ESP32-C3";
    case CHIP_ESP32C2:
        return "ESP32-C2";
    case CHIP_ESP32C6:
        return "ESP32-C6";
    case CHIP_ESP32H2:
        return "ESP32-H2";
    default:
        return "неизвестно";
    }
}

uint32_t fc_boot_count(void) {
    return g_boot_count;
}

const char *fc_reset_reason_name(void);

static const char *reset_reason_name(esp_reset_reason_t r) {
    switch (r) {
    case ESP_RST_POWERON:
        return "POWERON (подача питания)";
    case ESP_RST_EXT:
        return "EXT (внешний сброс, кнопка EN)";
    case ESP_RST_SW:
        return "SW (esp_restart)";
    case ESP_RST_PANIC:
        return "PANIC (исключение)";
    case ESP_RST_INT_WDT:
        return "INT_WDT";
    case ESP_RST_TASK_WDT:
        return "TASK_WDT";
    case ESP_RST_WDT:
        return "WDT (прочий)";
    case ESP_RST_BROWNOUT:
        return "BROWNOUT (просадка питания)";
    case ESP_RST_DEEPSLEEP:
        return "DEEPSLEEP";
    case ESP_RST_USB:
        return "USB";
    default:
        return "UNKNOWN";
    }
}

const char *fc_reset_reason_name(void) {
    return reset_reason_name(g_reset_reason);
}

static void print_banner(uint32_t boot_count) {
    esp_chip_info_t chip;
    esp_chip_info(&chip);

    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);

    rtc_cpu_freq_config_t cpu;
    rtc_clk_cpu_freq_get_config(&cpu);

    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    const esp_app_desc_t *app = esp_app_get_description();
    unsigned rev = efuse_hal_chip_revision();

    printf("\n");
    printf("================ FloatCore ESP32 ================\n");
    printf("chip:        %s rev v%u.%u, %d core(s)\n", chip_model_name(chip.model), rev / 100,
           rev % 100, chip.cores);
    printf("features:    %s%s%s%s\n", (chip.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi " : "",
           (chip.features & CHIP_FEATURE_BT) ? "BT " : "",
           (chip.features & CHIP_FEATURE_BLE) ? "BLE " : "",
           (chip.features & CHIP_FEATURE_EMB_FLASH) ? "embedded-flash " : "");
    printf("cpu freq:    %d MHz (xtal %d MHz)\n", (int) cpu.freq_mhz, (int) rtc_clk_xtal_freq_get());
    printf("flash:       %" PRIu32 " B (%" PRIu32 " MB)\n", flash_size, flash_size / (1024 * 1024));
    printf("mac:         %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4],
           mac[5]);
    printf("heap:        %" PRIu32 " B free, %" PRIu32 " B min\n", esp_get_free_heap_size(),
           esp_get_minimum_free_heap_size());
    printf("ESP-IDF:     %s\n", esp_get_idf_version());
    printf("build:       %s %s (проект %s, версия %s)\n", app->date, app->time, app->project_name,
           app->version);
    printf("reset:       %s\n", reset_reason_name(g_reset_reason));
    printf("boot #:      %" PRIu32 " (счётчик в NVS — доказательство persistence)\n", boot_count);
    printf("motor:       %s\n", fc_motor_backend_name());
    printf("can:         %s\n", fc_can_backend_name());
    printf("imu:         mock (%d Hz, покой)\n", fc_imu_rate_hz());
    printf("adc:         mock (%.2f V, footpad disengaged)\n", (double) fc_adc_safe_voltage());
    printf("================================================\n\n");
}

static void report_task(void *arg) {
    (void) arg;
    const int marks[] = {10, 60};
    size_t next = 0;
    int64_t start = esp_timer_get_time();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (next >= sizeof(marks) / sizeof(marks[0])) {
            vTaskDelay(portMAX_DELAY);
        }
        int elapsed = (int) ((esp_timer_get_time() - start) / 1000000);
        if (elapsed < marks[next]) {
            continue;
        }
        int mark = marks[next++];

        printf("\n---- FloatCore: отчёт через %d с ----\n", mark);
        RefloatSnapshot s = refloat_facade_snapshot();
        printf("refloat state    %s, footpad %s\n", refloat_facade_state_name(s.state),
               refloat_facade_footpad_name(s.footpad_state));
        printf("pitch/roll       %.3f / %.3f deg (mock IMU в покое)\n", (double) s.pitch,
               (double) s.roll);
        printf("freq (Refloat)   imu %.1f Hz, main %.1f Hz\n", (double) s.imu_frequency,
               (double) s.main_frequency);

        for (int i = 0; i < FC_TIMING_COUNT; ++i) {
            FcTimingStats t = fc_timing_get((FcTimingChannel) i);
            if (!t.iterations) {
                continue;
            }
            printf("timing %-26s iters %-8llu mean %8.1f us  min %6" PRIu32 "  max %6" PRIu32
                   "  late %" PRIu32 "\n",
                   t.name, (unsigned long long) t.iterations,
                   (double) t.sum_period_us / (double) t.iterations, t.min_period_us,
                   t.max_period_us, t.late);
        }

        printf("heap             free %" PRIu32 " B, min %" PRIu32 " B\n", esp_get_free_heap_size(),
               esp_get_minimum_free_heap_size());
        // uxTaskGetStackHighWaterMark на ESP-IDF возвращает БАЙТЫ — минимум
        // свободного стека за всё время жизни задачи.
        printf("stack %-14s свободно минимум %" PRIu32 " B из 4096\n", "fc_imu",
               fc_imu_stack_watermark());
        for (size_t i = 0; i < fc_thread_count(); ++i) {
            printf("stack %-14s свободно минимум %" PRIu32 " B из 12288\n", fc_thread_name(i),
                   fc_thread_stack_watermark(i));
        }

        FcMotorStats m = fc_motor_stats();
        printf("motor blocked    всего %llu (current %llu, brake %llu, duty %llu), keepalive %llu\n",
               (unsigned long long) m.total_blocked,
               (unsigned long long) m.blocked[FC_MOTOR_CMD_CURRENT],
               (unsigned long long) m.blocked[FC_MOTOR_CMD_BRAKE],
               (unsigned long long) m.blocked[FC_MOTOR_CMD_DUTY],
               (unsigned long long) m.keepalive_calls);
        printf("-------------------------------------\n\n");
        fflush(stdout);
    }
}

void app_main(void) {
    g_reset_reason = esp_reset_reason();

    // 1. Хранилище — до всего остального: Refloat читает конфигурацию в init().
    bool storage_ok = fc_storage_init();

    uint32_t boot_count = 0;
    if (storage_ok) {
        uint32_t stored = 0xFFFFFFFF;
        fc_storage_read(&stored, FC_BOOT_COUNTER_ADDR);
        boot_count = (stored == 0xFFFFFFFF) ? 1 : stored + 1;
        g_boot_count = boot_count;
        fc_storage_write(boot_count, FC_BOOT_COUNTER_ADDR);
        fc_storage_commit();
    }

    print_banner(boot_count);

    if (!storage_ok) {
        ESP_LOGE(TAG, "NVS недоступна — останов: без хранилища запуск Refloat недоказуем");
        return;
    }

    // 2. Платформенный backend VESC_IF.
    fc_timing_reset();
    fc_vesc_if_init();
    // Главный поток Refloat работает на MAIN_THREAD_FREQ = 500 Гц
    // (refloat-upstream/src/main.c:61), период 2000 мкс.
    fc_timing_set_nominal(FC_TIMING_MAIN, 2000);
    fc_timing_set_nominal(FC_TIMING_AUX, 1000000 / 30);  // LEDS_REFRESH_RATE = 30

    // 3. Mock-IMU: задаёт ритм контура через imu_ref_callback.
    fc_imu_mock_start();

    // 4. Настоящий Refloat. Тот же init(), что и на VESC: refloat_facade_start()
    //    подставляет lib_info.arg и вызывает refloat_init().
    printf("[floatcore] Refloat initialization started\n");
    bool ok = refloat_facade_start();
    if (!ok) {
        ESP_LOGE(TAG, "refloat_init() вернул false — Refloat НЕ запущен");
        return;
    }
    printf("[floatcore] Refloat initialized: %s\n",
           fc_config_registered() ? "config registered" : "config NOT registered");

    // 5. Проверка безопасного состояния footpad сразу после старта (ТЗ §9).
    vTaskDelay(pdMS_TO_TICKS(2500));  // дождаться imu_startup_done и первых итераций
    RefloatSnapshot s = refloat_facade_snapshot();
    printf("[floatcore] footpad: %s, adc %.2f/%.2f V, пороги %.2f/%.2f V -> %s\n",
           refloat_facade_footpad_name(s.footpad_state), (double) s.adc_left,
           (double) s.adc_right, (double) s.fault_adc1, (double) s.fault_adc2,
           s.footpad_state == 0 ? "DISENGAGED (ok)" : "ENGAGED — ОСТАНОВ ПО ТЗ");
    if (s.footpad_state != 0) {
        ESP_LOGE(TAG, "mock ADC привёл Refloat в engaged state — это stop condition ТЗ v0.5");
    }
    printf("[floatcore] refloat state: %s\n", refloat_facade_state_name(s.state));

    // 6. Отчёты и консоль.
    printf("[floatcore] config test value = %.3f (leds.status.brightness_headlights_off)\n",
           (double) refloat_facade_config_test_value());

    xTaskCreatePinnedToCore(report_task, "fc_report", 4096, NULL, 3, NULL, FC_CORE_HOUSEKEEPING);
    fc_console_start();
    printf("\nfloatcore> ");
    fflush(stdout);
}
