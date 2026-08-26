// Host-тесты платформенных модулей ESP32 (ТЗ v0.5 §20).
//
// Проверяется то, что можно проверить без платы и что стоит слишком дорого,
// чтобы ловить это на железе:
//   * безопасное состояние footpad;
//   * арифметика периодов, перцентилей и времени исполнения.
//
// Блокировка команд мотору проверяется не здесь, а в tests/safety: с v0.6A
// единственная точка выхода — Motor Gate, и он платформенно-нейтрален.
//
// Модули platform/esp32/main собираются здесь как есть, поверх заглушек
// esp_timer/esp_log из tests/esp32/stubs. Это тот же приём, что и с
// mock-платформой VESC: код прошивки не подстраивается под тест.

#include "stubs/esp_log.h"
#include "stubs/esp_timer.h"

#include "../../platform/esp32/main/fc_platform.h"

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

static void test_timing(void) {
    printf("\n  \033[1m2. арифметика периодов контура\033[0m\n");

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

static void test_timing_distribution(void) {
    printf("\n  \033[1m4. перцентили, пропуски и время исполнения\033[0m\n");

    esp32_test_set_time_us(0);
    fc_timing_reset();
    fc_timing_set_nominal(FC_TIMING_CONTROL, 2000);

    // 990 периодов ровно по 2000 мкс и 10 по 4000: p99 обязан показать
    // хвост, а среднее — почти не заметить его.
    int64_t now = 0;
    esp32_test_set_time_us(now);
    fc_timing_tick(FC_TIMING_CONTROL);
    for (int i = 0; i < 1000; ++i) {
        now += (i % 100 == 99) ? 4000 : 2000;
        esp32_test_set_time_us(now);
        fc_timing_tick(FC_TIMING_CONTROL);
    }
    FcTimingStats t = fc_timing_get(FC_TIMING_CONTROL);
    check(t.iterations == 1000, "учтена тысяча интервалов");
    check(t.p50_us <= 2040, "медиана осталась у номинала");
    // Хвост занимает ровно 1 %, поэтому его видит p99.9, а не p99: при
    // накоплении 990 из 1000 порог 99 % достигается ещё в корзине номинала.
    // Это не придирка к формулировке — именно так и надо читать перцентили.
    check(t.p99_us <= 2040, "p99 при хвосте ровно в 1 % остаётся у номинала");
    check(t.p999_us >= 4000, "p99.9 показывает хвост, который среднее не видит");
    check(t.missed == 10, "период вдвое больше номинала считается пропуском");
    check(t.late == 10, "он же считается опозданием");
    printf("      \033[90m·\033[0m mean %.1f, p50 %u, p95 %u, p99 %u, max %u\n",
           (double) t.sum_period_us / (double) t.iterations, (unsigned) t.p50_us,
           (unsigned) t.p95_us, (unsigned) t.p99_us, (unsigned) t.max_period_us);

    // Время исполнения меряется отдельно от периода: это и есть ответ на
    // вопрос «планировщик опаздывает или итерация долго считает».
    fc_timing_reset();
    fc_timing_set_nominal(FC_TIMING_MAIN, 2000);
    now = 0;
    for (int i = 0; i < 100; ++i) {
        esp32_test_set_time_us(now);
        fc_timing_exec_begin(FC_TIMING_MAIN);
        esp32_test_set_time_us(now + 150);
        fc_timing_exec_end(FC_TIMING_MAIN);
        now += 2000;
    }
    t = fc_timing_get(FC_TIMING_MAIN);
    check(t.exec_samples == 100, "сто измерений исполнения");
    check(t.exec_sum_us / t.exec_samples == 150, "среднее время исполнения 150 мкс");
    check(t.exec_min_us == 150 && t.exec_max_us == 150, "min и max совпадают на ровном профиле");
    check(t.iterations == 0, "измерение исполнения не подменяет собой период");

    // Гистограмма выгружается и сумма её корзин равна числу интервалов.
    fc_timing_reset();
    fc_timing_set_nominal(FC_TIMING_CONTROL, 2000);
    now = 0;
    for (int i = 0; i < 51; ++i) {
        esp32_test_set_time_us(now);
        fc_timing_tick(FC_TIMING_CONTROL);
        now += 2000;
    }
    static uint32_t bins[FC_TIMING_BINS + 1];
    uint32_t width = 0;
    uint32_t n = fc_timing_histogram(FC_TIMING_CONTROL, bins, FC_TIMING_BINS + 1, &width);
    uint64_t sum = 0;
    for (uint32_t i = 0; i < n; ++i) {
        sum += bins[i];
    }
    check(n == FC_TIMING_BINS + 1, "гистограмма отдаёт все корзины плюс переполнение");
    check(width == 2000 * 4 / FC_TIMING_BINS, "ширина корзины — четыре номинала на все корзины");
    check(sum == 50, "сумма корзин равна числу интервалов");
}

static void test_uptime(void) {
    printf("\n  \033[1m3. монотонное время платформы\033[0m\n");
    esp32_test_set_time_us(1234567);
    check(fc_uptime_us() == 1234567, "fc_uptime_us берёт время у esp_timer без пересчёта");
}

int main(void) {
    printf("\n\033[1mТесты платформенного слоя ESP32 (без платы)\033[0m\n");
    test_adc_safe();
    test_timing();
    test_timing_distribution();
    test_uptime();

    printf("\n================================================================\n");
    if (g_failures == 0) {
        printf("\033[32mВсе проверки пройдены\033[0m (%d)\n\n", g_checks);
        return 0;
    }
    printf("\033[31mПровалено %d из %d\033[0m\n\n", g_failures, g_checks);
    return 1;
}
