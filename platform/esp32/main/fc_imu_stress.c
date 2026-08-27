// Стресс-тест шины I2C датчика ICM-20948 (ТЗ v0.6C).
//
// Назначение узкое: доказать или опровергнуть, что после перепайки SDA/SCL
// соединение остаётся исправным при механическом воздействии на жгут. Это
// проверка ЖЕЛЕЗА, а не Refloat.
//
// Категория команды — STATE_CHANGING (docs/esp32_safety.md): тест меняет
// состояние подсистемы IMU, но физически безопасен. Пути к мотору он не
// касается вовсе: в этом файле нет ни одного вызова, способного что-либо
// отправить, а единственная точка выхода (Motor Gate) в профиле LAB_SAFE
// не имеет backend'а как кода.
//
// --- три решения, которые стоит объяснить ---------------------------------
//
// 1. Тест НЕ ДОБАВЛЯЕТ вторую задачу на ту же шину, а ЗАМЕЩАЕТ штатную.
//    У драйвера icm20948 нет мьютекса — он рассчитан на одного читателя
//    (fc_imu_real.c). Второй читатель дал бы состязание за шину и породил
//    отказы, неотличимые от искомых аппаратных. Поэтому `imu_stress`
//    останавливает единую realtime-задачу (fc_imu_rt), работает вместо неё, а
//    по завершении поднимает её обратно. С v0.6D это означает, что на время
//    теста контур Refloat не получает семплов вовсе: супервизор увидит
//    таймаут и уйдёт в FAULT. Это объявляется в выводе теста, а не
//    маскируется — исключительное владение шиной важнее красивого лога
//    (ТЗ v0.6D §27).
//
// 2. Статистика ведётся СВОЯ, а не берётся из icm20948_stats().
//    Сброс шины (icm20948_init) внутри себя делает memset всей структуры
//    драйвера, то есть обнуляет и его счётчики. Тест, который теряет
//    статистику ровно в момент отказа, бесполезен.
//
// 3. Health-слой получает успехи с прореживанием до 500 Гц, отказы — все.
//    Датчик выдаёт 562.5 Гц, а тест читает быстрее 1000 Гц, поэтому подряд
//    идущие чтения регулярно возвращают ОДИН И ТОТ ЖЕ семпл. Детектор
//    «зависания» в fc_imu_health (5 одинаковых семплов подряд) на таком
//    потоке сработал бы ложно. Прореживание до 500 Гц даёт health-слою ровно
//    ту же картину, что и штатная задача. Отказы прореживать нельзя: факт
//    неудачной транзакции обязан дойти до Supervisor немедленно.

#include "fc_platform.h"

#include "../drivers/icm20948.h"
#include "../../../compat/safety/fc_imu_health.h"
#include "../../../compat/safety/fc_supervisor.h"

#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

// Кольцо предыстории. 128 записей при частоте выше 1000 Гц — это около 100 мс
// перед отказом: достаточно, чтобы увидеть, деградировала ли длительность
// транзакций заранее, или обрыв случился мгновенно.
#define FC_STRESS_LOG_ENTRIES 128

// Порог перед сбросом шины. Одиночный NACK бывает и на исправной шине при
// наводке; три подряд означают, что ведомый или шина в неопределённом
// состоянии и сама она из него не выйдет.
#define FC_STRESS_DEFAULT_RESET_THRESHOLD 3

// Ограничитель печати. При обрыве шины отказы идут потоком в тысячи в
// секунду; печать каждого забила бы UART (115200 бод — около 11 КБ/с) и
// остановила бы сам тест, то есть измерение уничтожило бы измеряемое.
// Первые записи печатаются полностью, дальше — не чаще одной в 200 мс,
// а подавленные считаются и попадают в посекундную сводку.
#define FC_STRESS_FULL_REPORTS 20
#define FC_STRESS_REPORT_MIN_GAP_US 200000

typedef struct {
    uint64_t t_us;
    uint32_t dur_us;
    int32_t err;  // esp_err_t; ESP_OK — успешное чтение
} FcStressLogEntry;

typedef struct {
    uint64_t reads_total;
    uint64_t reads_ok;
    uint64_t reads_failed;
    uint64_t timeouts;
    uint64_t nacks;
    uint64_t other_errors;
    uint64_t recoveries;
    uint64_t bus_resets;
    uint64_t bus_reset_failures;
    uint32_t max_transaction_us;
    uint32_t min_transaction_us;
    uint64_t sum_transaction_us;
    uint32_t consecutive_failures;
    uint32_t max_consecutive_failures;
    uint64_t suppressed_reports;
    // Сколько чтений вернуло НОВЫЕ слова движения (accel+gyro, без
    // температуры: у неё своя частота обновления). Отношение к общему числу
    // чтений даёт фактическую частоту обновления регистров данных — её
    // невозможно получить из datasheet, если ODR и частота обновления
    // регистров различаются.
    uint64_t motion_changed;
    // Температура кристалла датчика. Читается в каждом семпле всё равно
    // (DS-000189 §8.2), но раньше не выводилась. Понадобилась, когда отказы
    // в покое начали появляться не сразу, а после десятков секунд работы:
    // без температуры отличить прогрев от совпадения нельзя.
    float temp_first_c;
    float temp_last_c;
    float temp_min_c;
    float temp_max_c;
    float temp_at_first_fail_c;
    bool have_temp;
    // Распределение длин серий отказов подряд.
    //
    // Без него нельзя ответить на главный вопрос: восстанавливается шина сама
    // или залипает до принудительного сброса. Пока порог сброса стоял на трёх,
    // максимум подряд во всех прогонах равнялся ровно трём — то есть измерялся
    // порог, а не шина.
    uint64_t burst_hist[9];  // 1, 2, 3, 4, 5, 6-10, 11-50, 51-200, 201+
} FcStressStats;

static struct {
    TaskHandle_t task;
    volatile bool run;
    volatile bool active;
    uint64_t started_us;
    uint64_t deadline_us;
    uint32_t requested_s;
    uint32_t requested_hz;  // 0 — оставить частоту шины как есть
    uint32_t actual_hz;
    uint32_t reset_threshold;  // 0 — не сбрасывать шину никогда

    FcStressStats st;

    FcStressLogEntry ring[FC_STRESS_LOG_ENTRIES];
    uint32_t ring_head;
    uint32_t ring_count;

    // Снимок предыстории, снятый в момент ПЕРВОГО отказа и больше не
    // затираемый: к нему можно вернуться командой `imu_stress-log` после
    // того, как поток последующих ошибок вытеснит кольцо.
    FcStressLogEntry snapshot[FC_STRESS_LOG_ENTRIES];
    uint32_t snapshot_count;
    bool snapshot_taken;
    uint64_t first_error_us;
    int32_t first_error;

    uint64_t last_report_us;

    // Влияние теста на контур Refloat измеряется, а не декларируется.
    FcTimingStats t_control_before;
    FcTimingStats t_main_before;
} S;

// --------------------------------------------------------------- вспомогательное

static const char *err_reason(int32_t err) {
    // Классификация по кодам драйвера i2c_master ESP-IDF v5.5.
    switch (err) {
    case ESP_OK:
        return "OK";
    case ESP_ERR_TIMEOUT:
        return "TIMEOUT";
    case ESP_ERR_INVALID_STATE:
        // Новый драйвер i2c_master отдаёт NACK от ведомого именно так.
        return "NACK";
    case ESP_FAIL:
        return "NACK/FAIL";
    case ESP_ERR_INVALID_ARG:
        return "INVALID_ARG";
    case ESP_ERR_NO_MEM:
        return "NO_MEM";
    default:
        return "DRIVER_ERROR";
    }
}

static bool err_is_timeout(int32_t err) {
    return err == ESP_ERR_TIMEOUT;
}

static bool err_is_nack(int32_t err) {
    return err == ESP_ERR_INVALID_STATE || err == ESP_FAIL;
}

static void burst_record(uint32_t len) {
    int bin;
    if (len <= 5) {
        bin = (int) len - 1;
    } else if (len <= 10) {
        bin = 5;
    } else if (len <= 50) {
        bin = 6;
    } else if (len <= 200) {
        bin = 7;
    } else {
        bin = 8;
    }
    ++S.st.burst_hist[bin];
}

static void ring_push(uint64_t t_us, uint32_t dur_us, int32_t err) {
    S.ring[S.ring_head].t_us = t_us;
    S.ring[S.ring_head].dur_us = dur_us;
    S.ring[S.ring_head].err = err;
    S.ring_head = (S.ring_head + 1) % FC_STRESS_LOG_ENTRIES;
    if (S.ring_count < FC_STRESS_LOG_ENTRIES) {
        ++S.ring_count;
    }
}

static void ring_snapshot(void) {
    uint32_t n = S.ring_count;
    uint32_t start = (S.ring_head + FC_STRESS_LOG_ENTRIES - n) % FC_STRESS_LOG_ENTRIES;
    for (uint32_t i = 0; i < n; ++i) {
        S.snapshot[i] = S.ring[(start + i) % FC_STRESS_LOG_ENTRIES];
    }
    S.snapshot_count = n;
    S.snapshot_taken = true;
}

static void print_log(const FcStressLogEntry *log, uint32_t n, uint64_t ref_us) {
    printf("--- предыстория, последние %" PRIu32 " транзакций перед первым отказом ---\n", n);
    printf("      dt до отказа   длительность  результат\n");
    for (uint32_t i = 0; i < n; ++i) {
        double dt_ms = ((double) log[i].t_us - (double) ref_us) / 1000.0;
        printf("  %3" PRIu32 "  %+10.3f ms  %7" PRIu32 " us  %s\n", i, dt_ms, log[i].dur_us,
               err_reason(log[i].err));
    }
    printf("--- конец предыстории ---\n");
}

// ------------------------------------------------------------------ печать

static double elapsed_s(uint64_t now) {
    return (double) (now - S.started_us) * 1e-6;
}

static void print_periodic(uint64_t now) {
    const FcStressStats *s = &S.st;
    double secs = elapsed_s(now);
    printf("Time: %.0f s\n", secs);
    printf("Reads: %llu\n", (unsigned long long) s->reads_total);
    printf("Failed: %llu\n", (unsigned long long) s->reads_failed);
    printf("Recoveries: %llu\n", (unsigned long long) s->recoveries);
    printf("Timeouts: %llu\n", (unsigned long long) s->timeouts);
    printf("Bus resets: %llu\n", (unsigned long long) s->bus_resets);
    printf("Max transaction: %" PRIu32 " us\n", s->max_transaction_us);
    printf("Average transaction: %.0f us\n",
           s->reads_ok ? (double) s->sum_transaction_us / (double) s->reads_ok : 0.0);
    printf("Rate: %.0f reads/s\n", secs > 0.0 ? (double) s->reads_total / secs : 0.0);
    if (s->have_temp) {
        printf("Sensor temp: %.1f C (от старта %+.1f)\n", (double) s->temp_last_c,
               (double) (s->temp_last_c - s->temp_first_c));
    }
    if (s->suppressed_reports) {
        printf("(подавлено подробных отчётов об отказах: %llu)\n",
               (unsigned long long) s->suppressed_reports);
    }
    printf("\n");
    fflush(stdout);
}

static void print_failure(uint64_t now, int32_t err, bool recovered, bool did_reset) {
    printf("FAIL\n");
    printf("Time: %.3f s\n", elapsed_s(now));
    printf("Reason: %s (esp_err 0x%04" PRIx32 ", %s)\n", err_reason(err), (uint32_t) err,
           esp_err_to_name((esp_err_t) err));
    printf("Consecutive failures: %" PRIu32 "\n", S.st.consecutive_failures);
    printf("Bus recovered: %s%s\n", recovered ? "yes" : "no",
           did_reset ? " (после сброса шины)" : "");
    printf("\n");
    fflush(stdout);
}

static void print_summary(uint64_t now) {
    const FcStressStats *s = &S.st;
    double secs = elapsed_s(now);
    double fail_pct =
        s->reads_total ? 100.0 * (double) s->reads_failed / (double) s->reads_total : 0.0;

    printf("==================== ИТОГ СТРЕСС-ТЕСТА I2C ====================\n");
    printf("Duration:              %.3f s (запрошено %" PRIu32 " s)\n", secs, S.requested_s);
    printf("Bus clock:             %" PRIu32 " Hz\n", S.actual_hz);
    printf("Total reads:           %llu\n", (unsigned long long) s->reads_total);
    printf("Successful reads:      %llu\n", (unsigned long long) s->reads_ok);
    printf("Failed reads:          %llu\n", (unsigned long long) s->reads_failed);
    printf("Failure percentage:    %.6f %%\n", fail_pct);
    printf("  из них timeouts:     %llu\n", (unsigned long long) s->timeouts);
    printf("  из них NACK:         %llu\n", (unsigned long long) s->nacks);
    printf("  прочие ошибки:       %llu\n", (unsigned long long) s->other_errors);
    printf("Recoveries:            %llu\n", (unsigned long long) s->recoveries);
    printf("Bus resets:            %llu (неудачных %llu)\n", (unsigned long long) s->bus_resets,
           (unsigned long long) s->bus_reset_failures);
    printf("Max consecutive fails: %" PRIu32 "%s\n", s->max_consecutive_failures,
           S.reset_threshold ? " (ограничено порогом сброса)" : " (сброс шины отключён)");
    {
        static const char *names[9] = {"1", "2", "3", "4", "5", "6-10", "11-50", "51-200", "201+"};
        bool any = false;
        for (int i = 0; i < 9; ++i) {
            if (s->burst_hist[i]) {
                any = true;
            }
        }
        if (any) {
            printf("Длины серий отказов подряд (только завершившиеся сами):\n");
            for (int i = 0; i < 9; ++i) {
                if (s->burst_hist[i]) {
                    printf("  %-7s %llu\n", names[i], (unsigned long long) s->burst_hist[i]);
                }
            }
        }
    }
    printf("Maximum transaction:   %" PRIu32 " us\n", s->max_transaction_us);
    printf("Minimum transaction:   %" PRIu32 " us\n",
           s->min_transaction_us == UINT32_MAX ? 0 : s->min_transaction_us);
    printf("Average transaction:   %.1f us\n",
           s->reads_ok ? (double) s->sum_transaction_us / (double) s->reads_ok : 0.0);
    printf("Average rate:          %.0f reads/s\n",
           secs > 0.0 ? (double) s->reads_total / secs : 0.0);
    printf("Motion updates:        %llu (%.0f Hz) — фактическая частота обновления регистров\n",
           (unsigned long long) s->motion_changed,
           secs > 0.0 ? (double) s->motion_changed / secs : 0.0);
    if (s->have_temp) {
        printf("Sensor temp:           старт %.1f C, конец %.1f C, мин %.1f, макс %.1f (нагрев %+.1f)\n",
               (double) s->temp_first_c, (double) s->temp_last_c, (double) s->temp_min_c,
               (double) s->temp_max_c, (double) (s->temp_max_c - s->temp_first_c));
        if (s->reads_failed) {
            printf("                       на первом отказе %.1f C\n",
                   (double) s->temp_at_first_fail_c);
        }
    }

    // Влияние на контур: измеренная дельта, а не утверждение.
    FcTimingStats c = fc_timing_get(FC_TIMING_CONTROL);
    FcTimingStats m = fc_timing_get(FC_TIMING_MAIN);
    printf("--- влияние на Refloat за время теста ---\n");
    printf("  контур   итераций %llu, опозданий %" PRIu32 ", пропусков %" PRIu32 "\n",
           (unsigned long long) (c.iterations - S.t_control_before.iterations),
           c.late - S.t_control_before.late, c.missed - S.t_control_before.missed);
    printf("  main thd итераций %llu, опозданий %" PRIu32 ", пропусков %" PRIu32 "\n",
           (unsigned long long) (m.iterations - S.t_main_before.iterations),
           m.late - S.t_main_before.late, m.missed - S.t_main_before.missed);

    bool clean = s->reads_failed == 0 && s->recoveries == 0 && s->timeouts == 0 &&
                 s->bus_resets == 0;
    bool long_enough = secs >= 120.0;

    printf("--- критерий (ТЗ v0.6C, раздел PASS) ---\n");
    printf("  failed_reads == 0      %s\n", s->reads_failed == 0 ? "да" : "НЕТ");
    printf("  recoveries == 0        %s\n", s->recoveries == 0 ? "да" : "НЕТ");
    printf("  timeouts == 0          %s\n", s->timeouts == 0 ? "да" : "НЕТ");
    printf("  bus_resets == 0        %s\n", s->bus_resets == 0 ? "да" : "НЕТ");
    printf("  длительность >= 120 s  %s (%.1f s)\n", long_enough ? "да" : "НЕТ", secs);
    printf("РЕЗУЛЬТАТ: %s\n", (clean && long_enough) ? "PASS" : "FAIL");
    if (clean && !long_enough) {
        printf("  (отказов нет, но тест короче двух минут — это не PASS по ТЗ)\n");
    }
    printf("===============================================================\n");
    if (S.snapshot_taken) {
        printf("первый отказ: %.3f s от старта, причина %s\n",
               (double) (S.first_error_us - S.started_us) * 1e-6, err_reason(S.first_error));
        print_log(S.snapshot, S.snapshot_count, S.first_error_us);
    }
    fflush(stdout);
}

// ------------------------------------------------------------------- задача

static void stress_task(void *arg) {
    (void) arg;
    esp_task_wdt_add(NULL);

    // Копия конфигурации ОБЯЗАТЕЛЬНА: icm20948_bus_init делает memset всей
    // своей структуры, а icm20948_active_config() указывает внутрь неё.
    icm20948_config_t cfg = *icm20948_active_config();
    int16_t prev_motion[6];
    bool have_prev_motion = false;

    // Необязательная смена частоты шины — диагностический разделитель гипотез
    // (docs/i2c_stress_test.md §5). Отказы, вызванные нехваткой запаса по
    // фронтам, на 100 кГц должны почти исчезнуть: время нарастания растёт
    // вчетверо. Отказы механической природы от частоты почти не зависят.
    //
    // Меняется ТОЛЬКО на время теста и только для этой задачи: восстановление
    // идёт через fc_imu_rt_start(), который берёт icm20948_default_config()
    // и тем самым возвращает штатные 400 кГц. Настройки датчика (шкалы, ODR,
    // DLPF) не трогаются — меняется скорость шины, а не режим измерения.
    bool rate_overridden = false;
    if (S.requested_hz && S.requested_hz != cfg.i2c_hz) {
        cfg.i2c_hz = S.requested_hz;
        esp_err_t re = icm20948_init(&cfg);
        rate_overridden = (re == ESP_OK);
        if (!rate_overridden) {
            printf("imu_stress: не удалось поднять шину на %" PRIu32 " Гц (%s), работаю на %"
                   PRIu32 "\n",
                   S.requested_hz, esp_err_to_name(re), icm20948_active_config()->i2c_hz);
            cfg = *icm20948_active_config();
        }
    }
    S.actual_hz = cfg.i2c_hz;

    S.st.min_transaction_us = UINT32_MAX;
    S.started_us = (uint64_t) esp_timer_get_time();
    S.deadline_us = S.started_us + (uint64_t) S.requested_s * 1000000ULL;
    S.last_report_us = S.started_us;
    S.t_control_before = fc_timing_get(FC_TIMING_CONTROL);
    S.t_main_before = fc_timing_get(FC_TIMING_MAIN);

    uint64_t last_health_us = 0;
    uint64_t last_full_report_us = 0;

    printf("\nimu_stress: старт, %" PRIu32 " s, чтение без пауз (ожидаемо >1000 чтений/с)\n",
           S.requested_s);
    printf("imu_stress: частота шины %" PRIu32 " Гц%s\n", S.actual_hz,
           rate_overridden ? " (изменена на время теста, штатная 400000)" : " (штатная)");
    if (S.reset_threshold) {
        printf("imu_stress: сброс шины после %" PRIu32 " отказов подряд\n", S.reset_threshold);
    } else {
        printf("imu_stress: сброс шины ОТКЛЮЧЁН — измеряем, восстановится ли она сама\n");
    }
    printf("imu_stress: штатная задача fc_imu_hw на время теста остановлена\n");
    printf("imu_stress: контур на время теста без данных, супервизор уйдёт в FAULT;\n");
    printf("            выход на мотор при этом остаётся заблокированным\n");
    printf("imu_stress: ПО ОКОНЧАНИИ Supervisor защёлкнет IMU_UNHEALTHY — это ожидаемо, см. итог\n\n");
    fflush(stdout);

    while (S.run) {
        uint64_t t0 = (uint64_t) esp_timer_get_time();
        if (t0 >= S.deadline_us) {
            break;
        }

        icm20948_sample_t s;
        esp_err_t err = icm20948_read(&s);
        uint64_t t1 = (uint64_t) esp_timer_get_time();
        uint32_t dur = (uint32_t) (t1 - t0);

        ++S.st.reads_total;
        ring_push(t1, dur, (int32_t) err);

        FcImuRawSample raw;
        memset(&raw, 0, sizeof(raw));

        if (err == ESP_OK) {
            ++S.st.reads_ok;
            if (!have_prev_motion || memcmp(prev_motion, s.raw, sizeof(prev_motion)) != 0) {
                ++S.st.motion_changed;
                memcpy(prev_motion, s.raw, sizeof(prev_motion));
                have_prev_motion = true;
            }
            if (!S.st.have_temp) {
                S.st.have_temp = true;
                S.st.temp_first_c = s.temperature_c;
                S.st.temp_min_c = s.temperature_c;
                S.st.temp_max_c = s.temperature_c;
            }
            S.st.temp_last_c = s.temperature_c;
            if (s.temperature_c < S.st.temp_min_c) {
                S.st.temp_min_c = s.temperature_c;
            }
            if (s.temperature_c > S.st.temp_max_c) {
                S.st.temp_max_c = s.temperature_c;
            }
            S.st.sum_transaction_us += dur;
            if (dur > S.st.max_transaction_us) {
                S.st.max_transaction_us = dur;
            }
            if (dur < S.st.min_transaction_us) {
                S.st.min_transaction_us = dur;
            }
            if (S.st.consecutive_failures > 0) {
                // Шина вернулась в строй после серии отказов.
                ++S.st.recoveries;
                burst_record(S.st.consecutive_failures);
                printf("RECOVERED after %" PRIu32 " consecutive failures, t = %.3f s\n",
                       S.st.consecutive_failures, elapsed_s(t1));
                fflush(stdout);
                S.st.consecutive_failures = 0;
            }

            // Прореживание до 500 Гц — обоснование в шапке файла.
            if (t1 - last_health_us >= 2000) {
                last_health_us = t1;
                memcpy(raw.accel_g, s.accel_g, sizeof(raw.accel_g));
                memcpy(raw.gyro_dps, s.gyro_dps, sizeof(raw.gyro_dps));
                raw.temperature_c = s.temperature_c;
                raw.timestamp_us = s.timestamp_us;
                raw.sample_counter = s.sample_counter;
                raw.valid = true;
                if (fc_imu_health_update(true, &raw, t1) == FC_IMU_OK) {
                    fc_supervisor_report_imu_sample(t1);
                }
            }
        } else {
            ++S.st.reads_failed;
            ++S.st.consecutive_failures;
            if (S.st.consecutive_failures > S.st.max_consecutive_failures) {
                S.st.max_consecutive_failures = S.st.consecutive_failures;
            }
            if (err_is_timeout(err)) {
                ++S.st.timeouts;
            } else if (err_is_nack(err)) {
                ++S.st.nacks;
            } else {
                ++S.st.other_errors;
            }

            // Предыстория снимается ровно один раз — на первом отказе.
            if (!S.snapshot_taken) {
                ring_snapshot();
                S.first_error_us = t1;
                S.first_error = (int32_t) err;
                S.st.temp_at_first_fail_c = S.st.temp_last_c;
            }

            // Отказ обязан дойти до health и Supervisor немедленно.
            fc_imu_health_update(false, &raw, t1);

            bool did_reset = false;
            if (S.reset_threshold && S.st.consecutive_failures >= S.reset_threshold) {
                ++S.st.bus_resets;
                esp_err_t re = icm20948_init(&cfg);
                did_reset = true;
                if (re != ESP_OK) {
                    ++S.st.bus_reset_failures;
                }
                esp_task_wdt_reset();
            }

            bool full = S.st.reads_failed <= FC_STRESS_FULL_REPORTS ||
                        (t1 - last_full_report_us) >= FC_STRESS_REPORT_MIN_GAP_US;
            if (full) {
                last_full_report_us = t1;
                print_failure(t1, (int32_t) err, false, did_reset);
                // Предыстория здесь НЕ печатается намеренно. 128 строк — это
                // около 5 КБ, то есть 450 мс блокирующего UART на 115200 бод
                // ровно в тот момент, когда важна каждая транзакция: в первом
                // прогоне это дало дыру в измерении 0.853…1.310 с. Снимок уже
                // сохранён в памяти; он печатается в итоговом отчёте и по
                // команде imu_stress-log.
            } else {
                ++S.st.suppressed_reports;
            }

            // Защита от «горячей петли». Если транзакция провалилась мгновенно
            // (шина мертва, драйвер возвращает ошибку сразу), цикл без паузы
            // превратился бы в busy-wait на приоритете 13 и вытеснил бы
            // refloat_thd. Пауза в 1 тик этого не допускает и на исправной
            // шине не выполняется никогда.
            if (dur < 500) {
                vTaskDelay(1);
            }
        }

        uint64_t now = (uint64_t) esp_timer_get_time();
        if (now - S.last_report_us >= 1000000ULL) {
            S.last_report_us = now;
            print_periodic(now);
        }
        esp_task_wdt_reset();
    }

    uint64_t end = (uint64_t) esp_timer_get_time();
    print_summary(end);

    esp_task_wdt_delete(NULL);

    // Вернуть штатное чтение. Делается ДО снятия флага active: пока он
    // взведён, повторный `imu_stress` отвергается, и структура S не может
    // быть обнулена под нами, пока эта задача ещё печатает и поднимает
    // fc_imu_hw. Две задачи на одной шине без мьютекса — то самое
    // состязание, которого мы избегали.
    if (!fc_imu_rt_start()) {
        printf("imu_stress: ВНИМАНИЕ, штатная задача чтения не поднялась, датчик не отвечает\n");
    } else {
        printf("imu_stress: штатная задача fc_imu_hw восстановлена (500 Гц)\n");
    }
    // Побочный эффект, который честнее объявить, чем обойти.
    //
    // Восстановление делает полную реинициализацию датчика: аппаратный сброс
    // 100 мс плюс 40 мс на запуск гироскопа (DS-000189 §10). Всё это время
    // семплов нет, а порог свежести IMU у супервизора — FC_SUP_IMU_TIMEOUT_US
    // = 40 мс. Значит FAULT неизбежен по построению.
    //
    // Подавлять его нельзя: скармливать супервизору выдуманные отметки о
    // семплах ради красивого вывода — это ровно тот обход слоя безопасности,
    // который в проекте запрещён. Поэтому факт объявляется, а снятие остаётся
    // явным действием оператора.
    printf("imu_stress: Supervisor теперь в FAULT/IMU_UNHEALTHY — семплов не было %d мс,\n",
           140);
    printf("            пока датчик реинициализировался (порог свежести %d мс).\n",
           FC_SUP_IMU_TIMEOUT_US / 1000);
    printf("            Это следствие теста, а не отказ железа. Снять: fault-clear или restart.\n");
    fflush(stdout);

    S.run = false;
    S.task = NULL;
    S.active = false;
    vTaskDelete(NULL);
}

// -------------------------------------------------------------------- API

bool fc_imu_stress_running(void) {
    return S.active;
}

bool fc_imu_stress_start(uint32_t seconds, uint32_t i2c_hz, uint32_t reset_threshold) {
    if (S.active) {
        return false;
    }
    if (!fc_imu_rt_available()) {
        return false;
    }

    // Остановить штатного читателя и дождаться, пока он действительно уйдёт:
    // fc_imu_rt_stop() только снимает флаг, задача завершается на следующей
    // итерации (до 2 мс).
    fc_imu_rt_stop();
    for (int i = 0; i < 100 && fc_imu_rt_running(); ++i) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (fc_imu_rt_running()) {
        return false;
    }

    memset(&S, 0, sizeof(S));
    S.requested_s = seconds;
    S.requested_hz = i2c_hz;
    S.reset_threshold = reset_threshold;
    S.run = true;
    S.active = true;

    // Ядро housekeeping, а НЕ realtime — и это решение по измерению.
    //
    // Первая версия работала на ядре 1 с приоритетом штатной задачи. Тест
    // проходил, но refloat_thd получал ровно один пропуск периода в секунду
    // (20 за 20 с при нуле в покое). Причина — посекундная печать: около
    // 200 байт на 115200 бод занимают UART примерно на 17 мс, а вывод в
    // консоль ESP-IDF блокирующий, и всё это время задача приоритета 13
    // держала ядро 1, где живёт контур.
    //
    // Шина I2C к ядру не привязана, поэтому тест перенесён на ядро 0. Ядро 1
    // с контуром Refloat он теперь не трогает вовсе. Приоритет выше консоли
    // (4), чтобы измерение не прерывалось вводом, но ниже системных задач
    // ESP-IDF; консоль при этом остаётся отзывчивой, потому что цикл на 70 %
    // времени спит внутри драйвера I2C.
    if (xTaskCreatePinnedToCore(stress_task, "fc_imu_stress", 5120, NULL, FC_PRIO_IMU_STRESS,
                                &S.task, FC_CORE_HOUSEKEEPING) != pdPASS) {
        S.active = false;
        S.run = false;
        fc_imu_rt_start();
        return false;
    }
    return true;
}

void fc_imu_stress_stop(void) {
    S.run = false;
}

void fc_imu_stress_print_log(void) {
    if (!S.snapshot_taken) {
        printf("imu_stress: отказов не было, предыстория не сохранялась\n");
        return;
    }
    printf("imu_stress: первый отказ на %.3f s от старта, причина %s\n",
           (double) (S.first_error_us - S.started_us) * 1e-6, err_reason(S.first_error));
    print_log(S.snapshot, S.snapshot_count, S.first_error_us);
}
