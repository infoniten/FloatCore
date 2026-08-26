// Host-тесты платформенных модулей ESP32 (ТЗ v0.5 §20).
//
// Проверяется то, что можно проверить без платы и что стоит слишком дорого,
// чтобы ловить это на железе:
//   * безопасное состояние footpad;
//   * блокировка команд мотору и rate-limit её логов;
//   * арифметика периодов контура.
//
// Модули platform/esp32/main собираются здесь как есть, поверх заглушек
// esp_timer/esp_log из tests/esp32/stubs. Это тот же приём, что и с
// mock-платформой VESC: код прошивки не подстраивается под тест.

#include "stubs/esp_log.h"
#include "stubs/esp_timer.h"

#include "../../platform/esp32/main/fc_platform.h"
#include "../../compat/motor/logical_motor.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define VESC_PIN_ADC1 7
#define VESC_PIN_ADC2 8

// Значения по умолчанию из refloat-upstream/src/conf/settings.xml
// (fault_adc1/fault_adc2, <valDouble>2</valDouble>).
#define REFLOAT_DEFAULT_FAULT_ADC 2.0f

static int g_failures = 0;
static int g_checks = 0;

static void check(bool ok, const char *what) {
    ++g_checks;
    if (ok) {
        printf("      \033[32mPASS\033[0m %s\n", what);
    } else {
        printf("      \033[31mFAIL\033[0m %s\n", what);
        ++g_failures;
    }
}

// Воспроизведение решающего выражения Refloat, footpad_sensor.c:31.
static bool refloat_adc_on(float adc, float threshold) {
    return threshold == 0.0f || adc > threshold;
}

static void test_adc_safe(void) {
    printf("\n  \033[1m1. footpad остаётся disengaged\033[0m\n");

    check(fc_adc_read(VESC_PIN_ADC1) == 0.0f, "ADC1 отдаёт 0.00 В");
    check(fc_adc_read(VESC_PIN_ADC2) == 0.0f, "ADC2 отдаёт 0.00 В");
    check(fc_adc_read(0) < 0.0f, "остальные пины помечены как отсутствующие (-1.0)");

    check(
        !refloat_adc_on(fc_adc_read(VESC_PIN_ADC1), REFLOAT_DEFAULT_FAULT_ADC),
        "при пороге по умолчанию 2.00 В Refloat считает ADC1 отпущенным"
    );
    check(
        !refloat_adc_on(fc_adc_read(VESC_PIN_ADC2), REFLOAT_DEFAULT_FAULT_ADC),
        "то же для ADC2 — состояние FS_NONE"
    );

    // Граничный случай, ради которого 0.0 В недостаточно: порог 0 Refloat
    // трактует как «датчика нет» и считает зону ПОСТОЯННО нажатой.
    check(
        refloat_adc_on(fc_adc_read(VESC_PIN_ADC1), 0.0f),
        "порог 0 В означает engaged при любом ADC — это ловится проверкой конфигурации, "
        "а не значением ADC"
    );

    // Любой ненулевой порог обязан оставлять нас в безопасной зоне.
    bool safe_everywhere = true;
    for (float thr = 0.01f; thr <= 3.3f; thr += 0.01f) {
        if (refloat_adc_on(fc_adc_safe_voltage(), thr)) {
            safe_everywhere = false;
            break;
        }
    }
    check(safe_everywhere, "для любого порога 0.01…3.30 В состояние остаётся disengaged");
}

static void test_motor_blocked(void) {
    printf("\n  \033[1m2. команды мотору блокируются\033[0m\n");

    LogicalMotorConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    esp32_test_set_time_us(0);
    logical_motor_init(&cfg);
    esp32_test_log_reset();

    logical_motor_request_current(12.5f);
    logical_motor_request_brake_current(3.0f);
    logical_motor_request_duty(0.4f);
    logical_motor_keepalive();

    FcMotorStats s = fc_motor_stats();
    check(s.blocked[FC_MOTOR_CMD_CURRENT] == 1, "set_current заблокирован и посчитан");
    check(s.blocked[FC_MOTOR_CMD_BRAKE] == 1, "set_brake_current заблокирован и посчитан");
    check(s.blocked[FC_MOTOR_CMD_DUTY] == 1, "set_duty заблокирован и посчитан");
    check(s.total_blocked == 3, "суммарный счётчик блокировок совпадает");
    check(s.keepalive_calls == 1, "timeout_reset учтён отдельно и ничего не отправляет");
    check(fabsf(s.last_value[FC_MOTOR_CMD_CURRENT] - 12.5f) < 1e-6f,
          "запрошенное значение сохранено для диагностики, но не применено");

    check(!logical_motor_healthy(), "логический мотор всегда unhealthy: ESC не подключены");
    LogicalMotorTelemetry t = logical_motor_telemetry();
    check(!t.esc_a_alive && !t.esc_b_alive, "оба ESC помечены offline");
    check((t.faults & (LM_FAULT_ESC_A_TIMEOUT | LM_FAULT_ESC_B_TIMEOUT)) ==
              (LM_FAULT_ESC_A_TIMEOUT | LM_FAULT_ESC_B_TIMEOUT),
          "выставлены флаги таймаута телеметрии");
    check(t.motor_current == 0.0f && t.rpm == 0.0f, "телеметрия нулевая, а не выдуманная");
}

static void test_motor_log_rate_limit(void) {
    printf("\n  \033[1m3. лог блокировок не забивает serial\033[0m\n");

    LogicalMotorConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    esp32_test_set_time_us(0);
    logical_motor_init(&cfg);
    esp32_test_log_reset();

    // Десять секунд контура на 500 Гц: 5000 запросов.
    for (int i = 0; i < 5000; ++i) {
        esp32_test_set_time_us((int64_t) i * 2000);
        logical_motor_request_current(1.0f);
    }

    int logs = esp32_test_log_count();
    FcMotorStats s = fc_motor_stats();
    check(s.blocked[FC_MOTOR_CMD_CURRENT] == 5000, "заблокированы все 5000 запросов");
    check(logs <= 12, "записей в лог не больше 12 за 10 с (rate-limit 1/с)");
    check(logs >= 9, "но и не меньше 9 — факт блокировки в логе виден");
    printf("      \033[90m·\033[0m 5000 запросов -> %d записей в лог\n", logs);
}

static void test_timing(void) {
    printf("\n  \033[1m4. арифметика периодов контура\033[0m\n");

    esp32_test_set_time_us(0);
    fc_timing_reset();
    fc_timing_set_nominal(FC_TIMING_MAIN, 2000);

    // Идеальные 500 Гц, 100 итераций, затем один срыв на 5000 мкс.
    int64_t now = 0;
    for (int i = 0; i < 101; ++i) {
        esp32_test_set_time_us(now);
        fc_timing_tick(FC_TIMING_MAIN);
        now += 2000;
    }
    FcTimingStats t = fc_timing_get(FC_TIMING_MAIN);
    check(t.iterations == 100, "первая отметка задаёт начало окна, интервалов на один меньше");
    check(t.min_period_us == 2000 && t.max_period_us == 2000, "min = max = 2000 мкс");
    check(t.sum_period_us / t.iterations == 2000, "средний период 2000 мкс = 500 Гц");
    check(t.late == 0, "опозданий нет");

    esp32_test_set_time_us(now + 3000);  // период 5000 мкс вместо 2000
    fc_timing_tick(FC_TIMING_MAIN);
    t = fc_timing_get(FC_TIMING_MAIN);
    check(t.late == 1, "срыв периода в 2.5 раза посчитан как опоздание");
    check(t.max_period_us == 5000, "максимум обновлён");

    // Порог опоздания — 20 %: 2400 мкс ещё не опоздание, 2401 уже.
    fc_timing_reset();
    fc_timing_set_nominal(FC_TIMING_MAIN, 2000);
    esp32_test_set_time_us(0);
    fc_timing_tick(FC_TIMING_MAIN);
    esp32_test_set_time_us(2400);
    fc_timing_tick(FC_TIMING_MAIN);
    check(fc_timing_get(FC_TIMING_MAIN).late == 0, "2400 мкс (+20 %) опозданием не считается");
    esp32_test_set_time_us(2400 + 2401);
    fc_timing_tick(FC_TIMING_MAIN);
    check(fc_timing_get(FC_TIMING_MAIN).late == 1, "2401 мкс уже считается");
}

static void test_uptime(void) {
    printf("\n  \033[1m5. монотонное время платформы\033[0m\n");
    esp32_test_set_time_us(1234567);
    check(fc_uptime_us() == 1234567, "fc_uptime_us берёт время у esp_timer без пересчёта");
}

int main(void) {
    printf("\n\033[1mТесты платформенного слоя ESP32 (без платы)\033[0m\n");
    test_adc_safe();
    test_motor_blocked();
    test_motor_log_rate_limit();
    test_timing();
    test_uptime();

    printf("\n================================================================\n");
    if (g_failures == 0) {
        printf("\033[32mВсе проверки пройдены\033[0m (%d)\n\n", g_checks);
        return 0;
    }
    printf("\033[31mПровалено %d из %d\033[0m\n\n", g_failures, g_checks);
    return 1;
}
