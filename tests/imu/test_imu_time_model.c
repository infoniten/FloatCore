// Единая временная модель IMU (ТЗ v0.9B §5, §18).
//
// ЗАЧЕМ ЭТИ ТЕСТЫ СУЩЕСТВУЮТ. На v0.8B и v0.9A супервизор защёлкивал
// IMU_UNHEALTHY, а трассировка зазоров не показывала ничего необычного. Это
// выглядело как противоречие приборов, а оказалось следствием того, что в
// отказ ведут ДВА независимых пути и только один из них смотрит на возраст
// семпла. Разбор — docs/imu_time_model.md.
//
// Здесь проверяется, что после сведения к одной канонической шкале
// показания согласованы: зазор, возраст и причина отказа рассказывают одну и
// ту же историю.

#include "../../compat/imu/fc_imu_pipeline.h"
#include "../../compat/safety/fc_supervisor.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

void fc_test_check(bool ok, const char *what);
void fc_test_note(const char *fmt, ...);
#define check fc_test_check
#define note fc_test_note

#define PERIOD_US 2000u

// Каждый вызов даёт ЗАВЕДОМО новый физический семпл: дедупликация смотрит на
// сырые слова, поэтому их надо менять, иначе тест проверял бы детектор
// замирания, а не временную модель.
static int16_t WORD;
static int16_t LAST_WORDS[7];
static float LAST_ACCEL[3], LAST_GYRO[3];

static FcImuPipeVerdict push(bool read_ok, uint64_t now_us) {
    int16_t raw[7];
    for (int i = 0; i < 7; ++i) {
        raw[i] = (int16_t) (WORD + i);
        LAST_WORDS[i] = raw[i];
    }
    ++WORD;
    // Значения тоже обязаны шевелиться: детектор замирания сравнивает именно
    // их, и неподвижные числа он справедливо считает зависшим датчиком.
    float n = (float) (WORD % 17) * 1e-4f;
    float accel[3] = {n, 0.0f, 1.0f};
    float gyro[3] = {n * 10.0f, 0.0f, 0.0f};
    memcpy(LAST_ACCEL, accel, sizeof LAST_ACCEL);
    memcpy(LAST_GYRO, gyro, sizeof LAST_GYRO);
    return fc_imu_pipeline_submit(read_ok, raw, accel, gyro, 25.0f, now_us);
}

// Тот же физический семпл: слова не меняются.
static FcImuPipeVerdict push_duplicate(uint64_t now_us) {
    int16_t raw[7];
    memcpy(raw, LAST_WORDS, sizeof raw);
    return fc_imu_pipeline_submit(true, raw, LAST_ACCEL, LAST_GYRO, 25.0f, now_us);
}

// Довести супервизор до DISARMED: проверка протухания IMU работает только в
// рабочих состояниях, и без этого второй барьер молчал бы не потому, что он
// сломан, а потому, что машина ещё грузится.
static void reset_all(void) {
    WORD = 100;
    FcImuPipelineConfig pc = fc_imu_pipeline_default_config(PERIOD_US);
    FcImuHealthConfig hc = fc_imu_health_default_config();
    fc_imu_pipeline_init(&pc, &hc);

    uint64_t t = 1000;
    fc_supervisor_init(t);
    fc_supervisor_begin_self_test(t += 1000);
    fc_supervisor_self_test_result(true, t += 1000);
    fc_supervisor_report_platform_ready(true, t);
    fc_supervisor_report_config_valid(true, t);
    fc_supervisor_report_watchdog(true, t);
    fc_supervisor_report_calibration_valid(true, t);
    fc_supervisor_report_footpad(false, t);
    fc_supervisor_report_loop_tick(t);
}

static FcSupervisorImuTime mirror(void) {
    FcImuTimeline tl = fc_imu_pipeline_timeline();
    FcSupervisorImuTime t = {
        .last_valid_us = tl.last_valid_us,
        .prev_valid_us = tl.prev_valid_us,
        .last_gap_us = tl.last_gap_us,
        .sequence = tl.sequence,
        .last_verdict = tl.last_verdict,
        .last_poll_us = tl.last_poll_us,
    };
    return t;
}

// Один шаг «задачи супервизора»: опросить здоровье и доложить.
static void supervisor_tick(uint64_t now_us) {
    FcImuHealthState hs = fc_imu_health_poll(now_us);
    FcSupervisorImuTime t = mirror();
    if (hs != FC_IMU_NOT_INITIALIZED) {
        fc_supervisor_report_imu(hs == FC_IMU_OK, (uint32_t) hs, &t, now_us);
    }
    fc_supervisor_poll(now_us);
}

// ------------------------------------------- владение отметкой времени

static void test_ownership(void) {
    note("ВЛАДЕНИЕ КАНОНИЧЕСКОЙ ОТМЕТКОЙ (ТЗ v0.9B §2, §4)");
    reset_all();

    uint64_t t = 1000000;
    check(push(true, t) == FC_IMU_PIPE_ACCEPTED, "первый семпл принят");
    FcImuTimeline tl = fc_imu_pipeline_timeline();
    check(tl.last_valid_us == t, "отметка равна времени завершения транзакции");
    check(tl.prev_valid_us == 0, "предыдущей отметки ещё нет");
    check(tl.last_gap_us == 0, "зазор первого семпла не считается");
    check(tl.sequence == 1, "номер семпла начинается с единицы");

    t += PERIOD_US;
    push(true, t);
    tl = fc_imu_pipeline_timeline();
    check(tl.last_gap_us == PERIOD_US, "зазор равен фактическому интервалу");
    check(tl.prev_valid_us == t - PERIOD_US, "предыдущая отметка сохранена");
    check(tl.sequence == 2, "номер монотонно растёт");

    // Неудачное чтение НЕ двигает шкалу: семпла не было.
    uint64_t before = tl.last_valid_us;
    uint32_t seq_before = tl.sequence;
    t += PERIOD_US;
    check(push(false, t) == FC_IMU_PIPE_READ_FAILED, "неудачное чтение распознано");
    tl = fc_imu_pipeline_timeline();
    check(tl.last_valid_us == before, "неудачное чтение НЕ обновляет отметку");
    check(tl.sequence == seq_before, "неудачное чтение НЕ тратит номер");
    check(tl.last_poll_us == t, "но время опроса записано: опрос был");

    // Дубликат тоже не двигает.
    t += PERIOD_US;
    reset_all();
    t = 2000000;
    push(true, t);
    tl = fc_imu_pipeline_timeline();
    before = tl.last_valid_us;
    seq_before = tl.sequence;
    t += PERIOD_US;
    check(push_duplicate(t) == FC_IMU_PIPE_DUPLICATE, "повтор распознан");
    tl = fc_imu_pipeline_timeline();
    check(tl.last_valid_us == before && tl.sequence == seq_before,
          "повтор НЕ обновляет отметку и не тратит номер");
}

// --------------------------------------------- матрица пауз (ТЗ §5)

static void run_pause(uint32_t pause_us, const char *name, bool expect_fault,
                      FcImuFaultCause expect_cause) {
    reset_all();
    uint64_t t = 5000000;

    // Разогрев: несколько нормальных семплов. Отметка уходит и в супервизор:
    // без этого его проверка возраста вообще не включается, и тест проверял
    // бы только второй путь.
    for (int i = 0; i < 5; ++i) {
        push(true, t);
        fc_supervisor_report_imu_sample(t);
        fc_supervisor_report_loop_tick(t);
        supervisor_tick(t);
        t += PERIOD_US;
    }
    char buf[200];
    snprintf(buf, sizeof buf, "%s: до паузы отказа нет", name);
    check(fc_supervisor_status().state != FC_SUP_FAULT, buf);

    // Пауза: супервизор продолжает опрашивать, семплов нет.
    uint64_t pause_end = t + pause_us;
    for (uint64_t now = t; now < pause_end; now += 5000) {
        supervisor_tick(now);
    }
    supervisor_tick(pause_end);

    // Семпл после паузы.
    push(true, pause_end);
    fc_supervisor_report_imu_sample(pause_end);
    supervisor_tick(pause_end);

    FcSupervisorStatus st = fc_supervisor_status();
    FcImuTimeline tl = fc_imu_pipeline_timeline();

    // Зазор считается от ПОСЛЕДНЕГО принятого семпла, а он был на один
    // номинальный период раньше начала паузы.
    uint32_t expect_gap = pause_us + PERIOD_US;
    snprintf(buf, sizeof buf, "%s: зазор %u мкс измерен верно", name, (unsigned) expect_gap);
    check(tl.last_gap_us == expect_gap, buf);

    bool faulted = (st.faults_latched & FC_FAULT_IMU_UNHEALTHY) != 0;
    snprintf(buf, sizeof buf, "%s: отказ %s, как и ожидалось", name,
             expect_fault ? "есть" : "отсутствует");
    check(faulted == expect_fault, buf);

    if (expect_fault) {
        snprintf(buf, sizeof buf, "%s: причина названа (%s)", name,
                 fc_imu_fault_cause_name(expect_cause));
        check(st.imu_fault.cause == expect_cause, buf);
        // Главное свойство: снимок и трассировка рассказывают одну историю.
        // Отказ срабатывает В ХОДЕ паузы, как только возраст перешёл порог,
        // а не после неё. Поэтому возраст в снимке близок к порогу, а не к
        // полному зазору — и это правильно: ждать конца паузы означало бы
        // узнавать об отказе датчика только когда он починился.
        snprintf(buf, sizeof buf, "%s: возраст в снимке не меньше порога и не больше зазора",
                 name);
        check(st.imu_fault.computed_age_us >= FC_SUP_IMU_TIMEOUT_US &&
                  st.imu_fault.computed_age_us <= expect_gap,
              buf);
    }
}

static void test_pause_matrix(void) {
    note("МАТРИЦА ПАУЗ (ТЗ v0.9B §5)");
    note("порог возраста у супервизора: %d мкс", (int) FC_SUP_IMU_TIMEOUT_US);

    // Возраст считается от последнего ПРИНЯТОГО семпла, а он на один
    // номинальный период старше начала паузы. Поэтому пауза 39 мс даёт
    // возраст 41 мс и отказ: граница проходит по возрасту, а не по паузе.
    run_pause(10000, "пауза 10 мс", false, FC_IMU_FAULT_CAUSE_NONE);
    run_pause(20000, "пауза 20 мс", false, FC_IMU_FAULT_CAUSE_NONE);
    run_pause(37000, "пауза 37 мс (возраст 39)", false, FC_IMU_FAULT_CAUSE_NONE);
    run_pause(41000, "пауза 41 мс", true, FC_IMU_FAULT_CAUSE_HEALTH_STATE);
    run_pause(100000, "пауза 100 мс", true, FC_IMU_FAULT_CAUSE_HEALTH_STATE);

    // ПОЧЕМУ причина именно такая. У модуля здоровья и у супервизора пороги
    // совпадают — оба 40 мс, — а опрос здоровья в задаче стоит раньше. Значит
    // при штатной работе собственная проверка возраста НЕ СРАБАТЫВАЕТ никогда.
    // Это не дефект: она второй барьер на случай, когда здоровье не
    // опрашивается. Что барьер живой — доказывает следующий тест.
    note("оба порога равны %d мкс; путь через здоровье срабатывает первым",
         (int) FC_SUP_IMU_TIMEOUT_US);
}

// Второй барьер: супервизор ловит протухание САМ, без модуля здоровья.
static void test_age_barrier_alone(void) {
    note("ВТОРОЙ БАРЬЕР: проверка возраста без опроса здоровья");
    reset_all();
    uint64_t t = 11000000;
    for (int i = 0; i < 5; ++i) {
        push(true, t);
        fc_supervisor_report_imu_sample(t);
        fc_supervisor_report_loop_tick(t);
        fc_supervisor_poll(t);
        t += PERIOD_US;
    }
    uint64_t last_good = t - PERIOD_US;
    check(fc_supervisor_status().state != FC_SUP_FAULT, "до паузы отказа нет");

    uint64_t now = last_good + FC_SUP_IMU_TIMEOUT_US + 1000;
    fc_supervisor_poll(now);

    FcSupervisorStatus st = fc_supervisor_status();
    check((st.faults_latched & FC_FAULT_IMU_UNHEALTHY) != 0,
          "супервизор поднял отказ сам, без модуля здоровья");
    check(st.imu_fault.cause == FC_IMU_FAULT_CAUSE_AGE, "причина: возраст семпла");
    check(st.imu_fault.computed_age_us >= FC_SUP_IMU_TIMEOUT_US,
          "записанный возраст не меньше порога");
    note("возраст в момент отказа: %u мкс", (unsigned) st.imu_fault.computed_age_us);
}

// ------------------------- отказ чтения: второй путь (ТЗ §2, §19)

static void test_read_failure_path(void) {
    note("ВТОРОЙ ПУТЬ В ОТКАЗ: состояние здоровья, а не возраст");
    reset_all();
    uint64_t t = 7000000;
    for (int i = 0; i < 5; ++i) {
        push(true, t);
        fc_supervisor_report_imu_sample(t);
        fc_supervisor_report_loop_tick(t);
        supervisor_tick(t);
        t += PERIOD_US;
    }
    check(fc_supervisor_status().state != FC_SUP_FAULT, "до сбоя отказа нет");

    // ОДНО неудачное чтение. Возраст семпла при этом крошечный.
    push(false, t);
    supervisor_tick(t);

    FcSupervisorStatus st = fc_supervisor_status();
    check((st.faults_latched & FC_FAULT_IMU_UNHEALTHY) != 0,
          "одиночный сбой чтения защёлкивает отказ IMU");
    check(st.imu_fault.cause == FC_IMU_FAULT_CAUSE_HEALTH_STATE,
          "причина: состояние здоровья, а НЕ возраст");
    check(st.imu_fault.computed_age_us < FC_SUP_IMU_TIMEOUT_US,
          "возраст семпла в этот момент ЗАВЕДОМО меньше порога");
    note("возраст в момент отказа: %u мкс при пороге %d",
         (unsigned) st.imu_fault.computed_age_us, (int) FC_SUP_IMU_TIMEOUT_US);

    // Ровно это и создавало видимость противоречия приборов: трассировка
    // зазоров не обязана была ничего показать, потому что зазора не было.
    FcImuTimeline tl = fc_imu_pipeline_timeline();
    check(tl.last_gap_us == PERIOD_US,
          "зазор остался номинальным: трассировке показывать было нечего");
}

// ------------------------------------------- согласованность снимка

static void test_snapshot_consistency(void) {
    note("СОГЛАСОВАННОСТЬ СНИМКА");
    reset_all();
    uint64_t t = 9000000;
    for (int i = 0; i < 5; ++i) {
        push(true, t);
        fc_supervisor_report_imu_sample(t);
        fc_supervisor_report_loop_tick(t);
        supervisor_tick(t);
        t += PERIOD_US;
    }
    uint64_t last_good = t - PERIOD_US;

    // Долгая пауза, затем опрос.
    uint64_t now = last_good + 60000;
    supervisor_tick(now);

    FcSupervisorStatus st = fc_supervisor_status();
    check(st.imu_fault.cause != FC_IMU_FAULT_CAUSE_NONE, "отказ зафиксирован");
    check(st.imu_fault.time.last_valid_us == last_good,
          "в снимке та же отметка, что в канонической шкале");
    check(st.imu_fault.computed_age_us == (uint32_t) (st.imu_fault.now_us - last_good),
          "возраст в снимке = now минус отметка, без третьего источника");
    check(st.imu_fault.now_us <= now, "момент отказа записан");
    check(st.imu_fault.time.sequence == 5, "номер последнего принятого семпла записан");

    // Снимок принадлежит ПЕРВОМУ отказу: последующие его не затирают.
    uint32_t seq = st.imu_fault.time.sequence;
    supervisor_tick(now + 100000);
    check(fc_supervisor_status().imu_fault.time.sequence == seq,
          "последующие отказы снимок НЕ перезаписывают");
}

void test_imu_time_model_all(void) {
    test_ownership();
    test_pause_matrix();
    test_age_barrier_alone();
    test_read_failure_path();
    test_snapshot_consistency();
}
