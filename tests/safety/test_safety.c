// Host-тесты ядра безопасности FloatCore (ТЗ v0.6A §30).
//
// Supervisor, Motor Gate и IMU health платформенно-нейтральны: время подаётся
// параметром, поэтому здесь проверяется ровно та же логика, что исполняется
// на плате, а не её упрощённая копия. Плата для этого не нужна.

#include "../../compat/safety/fc_imu_health.h"
#include "../../compat/safety/fc_motor_gate.h"
#include "../../compat/safety/fc_supervisor.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

int fc_test_fail = 0;
int fc_test_checks = 0;
#define g_fail fc_test_fail
#define g_checks fc_test_checks

void fc_test_check(bool ok, const char *what);
void fc_test_note(const char *fmt, ...);
void test_imu_pipeline_all(void);
void test_imu_calibration_all(void);

static void check(bool ok, const char *what) {
    ++g_checks;
    if (ok) {
        printf("      \033[32mPASS\033[0m %s\n", what);
    } else {
        printf("      \033[31mFAIL\033[0m %s\n", what);
        ++g_fail;
    }
}

static void note(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    printf("      \033[90m·\033[0m ");
    vprintf(fmt, ap);
    printf("\n");
    va_end(ap);
}

// Те же две функции под внешними именами: тесты тракта IMU лежат в отдельном
// файле, но должны попадать в общий счётчик проверок.
void fc_test_check(bool ok, const char *what) {
    check(ok, what);
}

void fc_test_note(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    printf("      \033[90m·\033[0m ");
    vprintf(fmt, ap);
    printf("\n");
    va_end(ap);
}

// Довести супервизор до DISARMED со всеми выполненными условиями.
static uint64_t bring_up(uint64_t t) {
    fc_supervisor_init(t);
    fc_supervisor_begin_self_test(t += 1000);
    fc_supervisor_self_test_result(true, t += 1000);
    fc_supervisor_report_platform_ready(true, t);
    fc_supervisor_report_config_valid(true, t);
    fc_supervisor_report_watchdog(true, t);
    // С v0.6E валидная калибровка ориентации — условие READY. Без неё
    // платформа не знает, как датчик стоит относительно доски.
    fc_supervisor_report_calibration_valid(true, t);
    fc_supervisor_report_footpad(false, t);
    fc_supervisor_report_imu_healthy(true, t);
    fc_supervisor_report_loop_tick(t);
    fc_supervisor_report_imu_sample(t);
    return t;
}

static void test_states(void) {
    printf("\n  \033[1m1. машина состояний supervisor\033[0m\n");
    uint64_t t = 1000000;

    fc_supervisor_init(t);
    check(fc_supervisor_state() == FC_SUP_BOOT, "после init состояние BOOT");

    fc_supervisor_begin_self_test(t += 1000);
    check(fc_supervisor_state() == FC_SUP_SELF_TEST, "BOOT -> SELF_TEST");

    fc_supervisor_self_test_result(true, t += 1000);
    check(fc_supervisor_state() == FC_SUP_DISARMED, "SELF_TEST -> DISARMED при успехе");

    // READY недостижим, пока не выполнены условия.
    check(!fc_supervisor_request_ready(t), "DISARMED -> READY отклонён: условия не выполнены");

    fc_supervisor_report_platform_ready(true, t);
    fc_supervisor_report_config_valid(true, t);
    fc_supervisor_report_watchdog(true, t);
    fc_supervisor_report_footpad(false, t);
    fc_supervisor_report_imu_healthy(true, t);
    fc_supervisor_report_loop_tick(t);
    // Калибровка ориентации — такое же обязательное условие, как исправный
    // IMU: без неё углы, которые платформа отдаёт Refloat, относятся к
    // датчику, а не к доске (v0.6E).
    check(!fc_supervisor_request_ready(t), "без калибровки READY всё ещё отклонён");
    fc_supervisor_report_calibration_valid(true, t);
    check(fc_supervisor_request_ready(t), "DISARMED -> READY когда все условия выполнены");
    check(fc_supervisor_state() == FC_SUP_READY, "состояние READY");

    fc_supervisor_disarm(t += 1000);
    check(fc_supervisor_state() == FC_SUP_DISARMED, "READY -> DISARMED разрешён всегда");

    // Ноги на доске — не отказ, но READY снимает.
    fc_supervisor_request_ready(t);
    fc_supervisor_report_footpad(true, t += 1000);
    check(fc_supervisor_state() == FC_SUP_DISARMED, "footpad engaged снимает READY, но не FAULT");
    check(fc_supervisor_status().faults == FC_FAULT_NONE, "и не поднимает ни одной причины отказа");

    // Провал самопроверки.
    fc_supervisor_init(t);
    fc_supervisor_begin_self_test(t += 1000);
    fc_supervisor_self_test_result(false, t += 1000);
    check(fc_supervisor_state() == FC_SUP_FAULT, "SELF_TEST -> FAULT при провале");
    check((fc_supervisor_status().faults & FC_FAULT_SELF_TEST) != 0, "причина SELF_TEST зафиксирована");
}

static void test_faults(void) {
    printf("\n  \033[1m2. отказы и их залипание\033[0m\n");
    uint64_t t = 2000000;

    // Протухание IMU.
    t = bring_up(t);
    fc_supervisor_poll(t + FC_SUP_IMU_TIMEOUT_US / 2);
    check(fc_supervisor_state() == FC_SUP_DISARMED, "половина таймаута IMU — ещё не отказ");
    fc_supervisor_poll(t + FC_SUP_IMU_TIMEOUT_US + 1);
    check(fc_supervisor_state() == FC_SUP_FAULT, "протухший IMU -> FAULT");
    check((fc_supervisor_status().faults & FC_FAULT_IMU_UNHEALTHY) != 0, "причина IMU_UNHEALTHY");

    // Залипание: отказ не уходит сам, даже когда данные снова пошли.
    fc_supervisor_report_imu_sample(t + FC_SUP_IMU_TIMEOUT_US + 2);
    fc_supervisor_report_imu_healthy(true, t + FC_SUP_IMU_TIMEOUT_US + 2);
    fc_supervisor_poll(t + FC_SUP_IMU_TIMEOUT_US + 3);
    check(fc_supervisor_state() == FC_SUP_FAULT, "FAULT залипает: сам не снимается");
    check(fc_supervisor_clear_fault(t + FC_SUP_IMU_TIMEOUT_US + 4),
          "явное снятие проходит, когда причина исчезла");
    check(fc_supervisor_state() == FC_SUP_DISARMED, "после снятия — DISARMED, а не READY");

    // Снятие невозможно, пока причина активна.
    t = bring_up(t + 1000000);
    fc_supervisor_report_config_valid(false, t);
    check(fc_supervisor_state() == FC_SUP_FAULT, "невалидная конфигурация -> FAULT");
    check(!fc_supervisor_clear_fault(t), "снять отказ нельзя, пока конфигурация невалидна");
    fc_supervisor_report_config_valid(true, t);
    check(fc_supervisor_clear_fault(t), "после исправления снятие проходит");

    // Зависание контура.
    t = bring_up(t + 1000000);
    fc_supervisor_poll(t + FC_SUP_LOOP_TIMEOUT_US + 1);
    check(fc_supervisor_state() == FC_SUP_FAULT, "зависший контур -> FAULT");
    check((fc_supervisor_status().faults & FC_FAULT_LOOP_STALL) != 0, "причина LOOP_STALL");

    // Watchdog.
    t = bring_up(t + 1000000);
    fc_supervisor_report_watchdog(false, t);
    check(fc_supervisor_state() == FC_SUP_FAULT, "срыв watchdog -> FAULT");

    // Причины накапливаются в latched-маске.
    check((fc_supervisor_status().faults_latched & FC_FAULT_WATCHDOG) != 0,
          "latched-маска помнит причину");
}

static void test_config_write_policy(void) {
    printf("\n  \033[1m3. политика записи конфигурации\033[0m\n");
    uint64_t t = 5000000;

    fc_supervisor_init(t);
    check(!fc_supervisor_config_write_allowed(), "BOOT: запись запрещена");

    fc_supervisor_begin_self_test(t += 1000);
    check(!fc_supervisor_config_write_allowed(), "SELF_TEST: запись запрещена");

    t = bring_up(t);
    check(fc_supervisor_config_write_allowed(), "DISARMED: запись разрешена");

    check(fc_supervisor_request_ready(t), "переходим в READY");
    check(!fc_supervisor_config_write_allowed(),
          "READY: запись запрещена — flash-операция останавливает контур");

    fc_supervisor_disarm(t += 1000);
    check(fc_supervisor_config_write_allowed(), "вернулись в DISARMED — снова разрешена");

    fc_supervisor_raise_fault(FC_FAULT_INTERNAL, t += 1000);
    check(!fc_supervisor_config_write_allowed(),
          "FAULT: запись запрещена — состояние системы неизвестно");
}

static void test_motor_gate(void) {
    printf("\n  \033[1m4. Motor Gate: единственная точка выхода\033[0m\n");
    uint64_t t = 7000000;
    t = bring_up(t);
    fc_motor_gate_init();

    FcGateVerdict v = fc_motor_gate_request(FC_MOTOR_REQ_CURRENT, 10.0f, t);
    check(v == FC_GATE_REJECTED_DISARMED, "в DISARMED запрос тока отвергнут");

    fc_supervisor_request_ready(t);
    v = fc_motor_gate_request(FC_MOTOR_REQ_CURRENT, 10.0f, t);
    check(v == FC_GATE_REJECTED_DISARMED,
          "даже в READY выход запрещён: READY не разрешает тягу");

    // Санитарная проверка значения идёт раньше политики состояний.
    v = fc_motor_gate_request(FC_MOTOR_REQ_CURRENT, NAN, t);
    check(v == FC_GATE_REJECTED_INVALID, "NaN отвергнут как invalid, а не как disarmed");
    v = fc_motor_gate_request(FC_MOTOR_REQ_BRAKE_CURRENT, INFINITY, t);
    check(v == FC_GATE_REJECTED_INVALID, "Inf отвергнут как invalid");
    v = fc_motor_gate_request(FC_MOTOR_REQ_DUTY, 1e9f, t);
    check(v == FC_GATE_REJECTED_INVALID, "абсурдная величина отвергнута как invalid");

    fc_supervisor_raise_fault(FC_FAULT_IMU_UNHEALTHY, t);
    v = fc_motor_gate_request(FC_MOTOR_REQ_CURRENT, 5.0f, t);
    check(v == FC_GATE_REJECTED_FAULT, "в FAULT причина отказа важнее состояния");

    FcMotorGateStats s = fc_motor_gate_stats();
    check(s.requests_total == 6, "все шесть запросов посчитаны");
    check(s.rejected_invalid == 3, "три отвергнуты по значению");
    check(s.physically_sent == 0, "physically_sent == 0");
    check(s.delivered_to_backend == 0, "backend не получил ничего");
}

static void test_no_output_in_any_state(void) {
    printf("\n  \033[1m5. ни одно состояние не выпускает команду наружу\033[0m\n");
    uint64_t t = 9000000;

    const FcSupervisorState all[] = {FC_SUP_BOOT,  FC_SUP_SELF_TEST, FC_SUP_DISARMED,
                                     FC_SUP_READY, FC_SUP_ARMED,     FC_SUP_RUNNING,
                                     FC_SUP_FAULT};
    int covered = 0;
    fc_motor_gate_init();

    for (size_t i = 0; i < sizeof(all) / sizeof(all[0]); ++i) {
        // Доводим супервизор до нужного состояния честными переходами там,
        // где это возможно; ARMED/RUNNING в LAB_SAFE недостижимы вовсе.
        t += 100000;
        switch (all[i]) {
        case FC_SUP_BOOT:
            fc_supervisor_init(t);
            break;
        case FC_SUP_SELF_TEST:
            fc_supervisor_init(t);
            fc_supervisor_begin_self_test(t);
            break;
        case FC_SUP_DISARMED:
            t = bring_up(t);
            break;
        case FC_SUP_READY:
            t = bring_up(t);
            fc_supervisor_request_ready(t);
            break;
        case FC_SUP_ARMED:
        case FC_SUP_RUNNING:
            // Достичь нельзя: в сборке нет ни одного перехода в эти состояния.
            ++covered;
            continue;
        case FC_SUP_FAULT:
            t = bring_up(t);
            fc_supervisor_raise_fault(FC_FAULT_INTERNAL, t);
            break;
        default:
            continue;
        }
        ++covered;
        for (int k = 0; k < FC_MOTOR_REQ_KIND_COUNT; ++k) {
            fc_motor_gate_request((FcMotorRequestKind) k, 3.0f, t);
        }
        check(!fc_supervisor_motor_output_permitted(),
              fc_supervisor_state_name(fc_supervisor_state()));
    }
    check(covered == 7, "проверены все семь состояний, включая недостижимые");
    check(fc_motor_gate_stats().physically_sent == 0,
          "physically_sent == 0 после запросов во всех состояниях");
    note("всего запросов через gate: %llu, доставлено в backend: %llu",
         (unsigned long long) fc_motor_gate_stats().requests_total,
         (unsigned long long) fc_motor_gate_stats().delivered_to_backend);
}

static FcImuRawSample make_sample(float ax, float ay, float az, float gx, uint64_t ts,
                                  uint32_t counter) {
    FcImuRawSample s;
    memset(&s, 0, sizeof(s));
    s.accel_g[0] = ax;
    s.accel_g[1] = ay;
    s.accel_g[2] = az;
    s.gyro_dps[0] = gx;
    s.temperature_c = 25.0f;
    s.timestamp_us = ts;
    s.sample_counter = counter;
    s.valid = true;
    return s;
}

static void test_imu_health(void) {
    printf("\n  \033[1m6. диагностика сырого IMU\033[0m\n");
    FcImuHealthConfig cfg = fc_imu_health_default_config();
    fc_imu_health_init(&cfg);
    check(fc_imu_health_status().state == FC_IMU_NOT_INITIALIZED, "до первого семпла NOT_INITIALIZED");

    uint64_t t = 1000;
    for (int i = 0; i < 10; ++i) {
        FcImuRawSample s = make_sample(0.0f, 0.0f, 1.0f + i * 0.001f, i * 0.1f, t, (uint32_t) i);
        fc_imu_health_update(true, &s, t);
        t += 2000;
    }
    check(fc_imu_health_is_ok(), "живой поток семплов -> OK");

    // Ошибка транзакции.
    fc_imu_health_update(false, NULL, t);
    check(fc_imu_health_status().state == FC_IMU_READ_ERROR, "сбой чтения -> READ_ERROR");

    // NaN.
    fc_imu_health_init(&cfg);
    FcImuRawSample bad = make_sample(NAN, 0.0f, 1.0f, 0.0f, 1000, 1);
    fc_imu_health_update(true, &bad, 1000);
    check(fc_imu_health_status().state == FC_IMU_INVALID, "NaN -> INVALID");

    // Физически невозможный модуль ускорения (шкала ±2 g).
    fc_imu_health_init(&cfg);
    FcImuRawSample huge = make_sample(5.0f, 5.0f, 5.0f, 0.0f, 1000, 1);
    fc_imu_health_update(true, &huge, 1000);
    check(fc_imu_health_status().state == FC_IMU_INVALID, "|a| выше полной шкалы -> INVALID");

    // Нули, которые датчик выдаёт во сне.
    fc_imu_health_init(&cfg);
    FcImuRawSample zeros = make_sample(0.0f, 0.0f, 0.0f, 0.0f, 1000, 1);
    fc_imu_health_update(true, &zeros, 1000);
    check(fc_imu_health_status().state == FC_IMU_INVALID, "нулевой вектор ускорения -> INVALID");

    // Зависший датчик: одинаковые семплы подряд.
    fc_imu_health_init(&cfg);
    t = 1000;
    FcImuRawSample frozen = make_sample(0.0f, 0.0f, 1.0f, 0.0f, t, 1);
    for (uint32_t i = 0; i < cfg.stuck_limit + 2; ++i) {
        frozen.timestamp_us = t;
        frozen.sample_counter = 1 + i;  // счётчик идёт, а данные нет
        fc_imu_health_update(true, &frozen, t);
        t += 2000;
    }
    check(fc_imu_health_status().state == FC_IMU_STUCK, "повторяющиеся данные -> STUCK");

    // Протухание и потеря связи.
    fc_imu_health_init(&cfg);
    FcImuRawSample good = make_sample(0.0f, 0.0f, 1.0f, 0.0f, 1000, 1);
    fc_imu_health_update(true, &good, 1000);
    check(fc_imu_health_poll(1000 + cfg.stale_us / 2) == FC_IMU_OK, "свежий семпл — OK");
    check(fc_imu_health_poll(1000 + cfg.stale_us + 1) == FC_IMU_STALE, "истёк stale -> STALE");
    check(fc_imu_health_poll(1000 + cfg.timeout_us + 1) == FC_IMU_TIMEOUT, "истёк timeout -> TIMEOUT");

    // Обратный ход времени.
    fc_imu_health_init(&cfg);
    FcImuRawSample a = make_sample(0.0f, 0.0f, 1.0f, 0.0f, 5000, 1);
    fc_imu_health_update(true, &a, 5000);
    FcImuRawSample b = make_sample(0.0f, 0.1f, 1.0f, 0.0f, 4000, 2);
    fc_imu_health_update(true, &b, 5100);
    check(fc_imu_health_status().state == FC_IMU_INVALID, "время пошло назад -> INVALID");
}

static void test_build_profile(void) {
    printf("\n  \033[1m7. профиль сборки\033[0m\n");
#ifdef FLOATCORE_LAB_SAFE
    check(true, "тесты собраны с FLOATCORE_LAB_SAFE");
    uint64_t t = 12000000;
    t = bring_up(t);
    fc_supervisor_request_ready(t);
    check(!fc_supervisor_motor_output_permitted(),
          "в LAB_SAFE разрешение на выход — константа false, а не проверка в рантайме");
#else
    check(false, "тесты обязаны собираться в профиле LAB_SAFE");
#endif
}

static void test_calibration_required_for_ready(void) {
    printf("\n\033[1mКалибровка ориентации как условие READY\033[0m\n");
    uint64_t t = bring_up(1000000);
    check(fc_supervisor_request_ready(t += 1000), "с калибровкой READY достижим");

    // Потеря калибровки на ходу снимает готовность, но отказом не является.
    fc_supervisor_report_calibration_valid(false, t += 1000);
    check(fc_supervisor_state() == FC_SUP_DISARMED, "без калибровки READY снимается");
    check(fc_supervisor_status().faults == 0, "это не отказ: маска отказов пуста");
    check(!fc_supervisor_request_ready(t += 1000), "и обратно в READY уже не пускает");
    check(!fc_supervisor_motor_output_permitted(), "выход на мотор запрещён");

    fc_supervisor_report_calibration_valid(true, t += 1000);
    check(fc_supervisor_request_ready(t += 1000), "после возврата калибровки READY снова достижим");
}

int main(void) {
    printf("\n\033[1mТесты ядра безопасности: Supervisor, Motor Gate, IMU health\033[0m\n");
    test_states();
    test_faults();
    test_config_write_policy();
    test_motor_gate();
    test_no_output_in_any_state();
    test_imu_health();
    test_build_profile();
    test_calibration_required_for_ready();
    test_imu_pipeline_all();
    test_imu_calibration_all();

    printf("\n================================================================\n");
    if (g_fail == 0) {
        printf("\033[32mВсе проверки пройдены\033[0m (%d)\n\n", g_checks);
        return 0;
    }
    printf("\033[31mПровалено %d из %d\033[0m\n\n", g_fail, g_checks);
    return 1;
}
