// Политика здоровья IMU: матрица отказов (ТЗ v0.9C §8).
//
// Проверяется главное свойство новой политики: одиночный транзиент больше не
// равен потере датчика, но настоящее протухание по-прежнему гарантированно
// снимает тягу, а неустранимое — защёлкивает.

#include "../../compat/imu/fc_imu_pipeline.h"
#include "../../compat/motor/fc_dual_motor.h"
#include "../../compat/safety/fc_imu_policy.h"
#include "../../compat/safety/fc_supervisor.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

void fc_test_check(bool ok, const char *what);
void fc_test_note(const char *fmt, ...);
#define check fc_test_check
#define note fc_test_note

#define T0 5000000ull

static FcImuPolicyConfig CFG;

static FcImuPolicyInputs base(void) {
    FcImuPolicyInputs in;
    memset(&in, 0, sizeof in);
    in.now_us = T0;
    in.last_valid_us = T0 - 2000; // один период назад
    in.have_valid = true;
    return in;
}

static FcImuPermit eval(FcImuPolicyInputs *in, uint32_t *reasons) {
    return fc_imu_policy_evaluate(&CFG, in, reasons);
}

static void expect(const char *name, FcImuPolicyInputs *in, FcImuPermit want,
                   FcImuPolicyReason want_reason) {
    uint32_t r = 0;
    FcImuPermit got = eval(in, &r);
    char buf[200];
    snprintf(buf, sizeof buf, "%s: вердикт %s", name, fc_imu_permit_name(want));
    check(got == want, buf);
    if (want_reason) {
        snprintf(buf, sizeof buf, "%s: причина названа (%s)", name,
                 fc_imu_policy_reason_name(want_reason));
        check((r & (uint32_t) want_reason) != 0, buf);
    }
}

// ------------------------------------------------- матрица (ТЗ v0.9C §8)

static void test_matrix(void) {
    CFG = fc_imu_policy_default_config();
    note("ПОЛИТИКА IMU: свежесть %u мкс, защёлка %u мкс, серии %u и %u семплов",
         (unsigned) CFG.torque_freshness_us, (unsigned) CFG.latch_age_us,
         (unsigned) CFG.hold_samples, (unsigned) CFG.latch_samples);

    FcImuPolicyInputs in;

    // 1, 2: одиночные и двойные отвергнутые семплы. Раньше первый же из них
    // защёлкивал отказ навсегда — ради этого вся переделка.
    in = base();
    in.consecutive_invalid = 1;
    expect("1 один непригодный семпл", &in, FC_IMU_PERMIT_OK, 0);
    in.consecutive_invalid = 2;
    expect("2 два непригодных подряд", &in, FC_IMU_PERMIT_OK, 0);

    // 3: серия короче порога удержания.
    in = base();
    in.consecutive_invalid = CFG.hold_samples - 1;
    expect("3 серия короче порога", &in, FC_IMU_PERMIT_OK, 0);

    // 4: серия, перешедшая порог.
    in.consecutive_invalid = CFG.hold_samples;
    expect("4 серия достигла порога", &in, FC_IMU_PERMIT_HOLD,
           FC_IMU_POLICY_INVALID_BURST);

    // 5, 6: обмены.
    in = base();
    in.consecutive_read_failures = 1;
    expect("5 один неудачный обмен", &in, FC_IMU_PERMIT_OK, 0);
    in.consecutive_read_failures = CFG.hold_samples;
    expect("6a серия обменов достигла порога", &in, FC_IMU_PERMIT_HOLD,
           FC_IMU_POLICY_COMM_BURST);
    in.consecutive_read_failures = CFG.latch_samples;
    expect("6b серия обменов достигла защёлки", &in, FC_IMU_PERMIT_LOST,
           FC_IMU_POLICY_COMM_BURST);

    // 7, 8: протухание.
    in = base();
    in.last_valid_us = T0 - (CFG.torque_freshness_us + 1000);
    expect("7 возраст больше свежести", &in, FC_IMU_PERMIT_HOLD, FC_IMU_POLICY_STALE);
    in.last_valid_us = T0 - (CFG.latch_age_us + 1000);
    expect("8 возраст больше порога защёлки", &in, FC_IMU_PERMIT_LOST, FC_IMU_POLICY_STALE);

    // 9: восстановление. Свежий семпл возвращает право без участия человека —
    // в этом и смысл состояния HOLD.
    in = base();
    expect("9 восстановление проходит само", &in, FC_IMU_PERMIT_OK, 0);

    // 10: переинициализация не удалась — неустранимо.
    in = base();
    in.reinit_failed = true;
    expect("10 переинициализация не удалась", &in, FC_IMU_PERMIT_LOST,
           FC_IMU_POLICY_REINIT_FAILED);

    // 11: шина залипла — тоже неустранимо, независимо от возраста.
    in = base();
    in.bus_stuck = true;
    expect("11 шина залипла", &in, FC_IMU_PERMIT_LOST, FC_IMU_POLICY_BUS_STUCK);

    // 12, 13: провал модуля ускорения.
    in = base();
    in.consecutive_accel_low = 1;
    expect("12 одиночный провал ускорения", &in, FC_IMU_PERMIT_OK, 0);
    in.consecutive_accel_low = CFG.hold_samples;
    expect("13a затяжной провал ускорения", &in, FC_IMU_PERMIT_HOLD,
           FC_IMU_POLICY_ACCEL_LOW_BURST);
    in.consecutive_accel_low = CFG.latch_samples;
    expect("13b провал ускорения дошёл до защёлки", &in, FC_IMU_PERMIT_LOST,
           FC_IMU_POLICY_ACCEL_LOW_BURST);

    // 14: годного семпла ещё не было — это загрузка, а не отказ.
    in = base();
    in.have_valid = false;
    expect("14 годного семпла ещё не было", &in, FC_IMU_PERMIT_HOLD,
           FC_IMU_POLICY_NO_SAMPLE);

    // 15: отметка из будущего — рассогласование часов, а не свежесть.
    in = base();
    in.last_valid_us = T0 + 1000;
    expect("15 отметка из будущего", &in, FC_IMU_PERMIT_LOST, FC_IMU_POLICY_STALE);

    // 16: несколько причин сразу — названы все, вердикт по худшей.
    in = base();
    in.consecutive_read_failures = CFG.hold_samples;
    in.last_valid_us = T0 - (CFG.latch_age_us + 1000);
    uint32_t r = 0;
    FcImuPermit p = eval(&in, &r);
    check(p == FC_IMU_PERMIT_LOST, "16 вердикт по худшей причине");
    check((r & FC_IMU_POLICY_STALE) && (r & FC_IMU_POLICY_COMM_BURST),
          "16 названы ОБЕ причины, а не первая");

    // Согласованность чисел: серии выведены из времени, а не назначены.
    check(CFG.hold_samples * FC_IMU_POLICY_PERIOD_US == CFG.torque_freshness_us,
          "серия удержания выведена из свежести, а не назначена");
    check(CFG.latch_samples * FC_IMU_POLICY_PERIOD_US == CFG.latch_age_us,
          "серия защёлки выведена из порога защёлки");
    check(CFG.torque_freshness_us < CFG.latch_age_us,
          "между запретом и защёлкой есть восстановимая полоса");
    check(CFG.latch_age_us < 50000,
          "защёлка срабатывает раньше сторожевого таймера ESC в 50 мс");
}

// --------------------------------- поведение супервизора (ТЗ §8 п.17-18)

static void bring_up(uint64_t t) {
    fc_supervisor_init(t);
    fc_supervisor_begin_self_test(t + 1000);
    fc_supervisor_self_test_result(true, t + 2000);
    fc_supervisor_report_platform_ready(true, t + 2000);
    fc_supervisor_report_config_valid(true, t + 2000);
    fc_supervisor_report_watchdog(true, t + 2000);
    fc_supervisor_report_calibration_valid(true, t + 2000);
    fc_supervisor_report_footpad(false, t + 2000);
    fc_supervisor_report_loop_tick(t + 2000);
    fc_supervisor_report_imu_sample(t + 2000);
}

static void test_supervisor_integration(void) {
    note("СУПЕРВИЗОР: HOLD не защёлкивает, LOST защёлкивает");
    CFG = fc_imu_policy_default_config();
    // Координатор приводится в исходное явно: в этом двоичном файле раньше
    // отработали другие наборы, и проверять «не вооружён» на чужом остатке
    // значило бы проверять порядок тестов, а не поведение.
    FcDualMotorConfig dc = fc_dual_motor_default_config();
    fc_dual_motor_init(&dc);

    uint64_t t = 3000000;
    bring_up(t);
    t += 2000;

    FcSupervisorImuTime st;
    memset(&st, 0, sizeof st);
    st.last_valid_us = t;
    st.sequence = 1;

    fc_supervisor_report_imu_permit(FC_IMU_PERMIT_OK, 0, 1, &st, t);
    check(fc_supervisor_status().state != FC_SUP_FAULT, "OK отказа не вызывает");

    // HOLD снимает здоровье, но НЕ защёлкивает: положение восстановимо.
    fc_supervisor_report_imu_permit(FC_IMU_PERMIT_HOLD, FC_IMU_POLICY_STALE, 1, &st, t + 1000);
    FcSupervisorStatus s = fc_supervisor_status();
    check(s.state != FC_SUP_FAULT, "HOLD НЕ переводит супервизор в отказ");
    check((s.faults_latched & FC_FAULT_IMU_UNHEALTHY) == 0, "HOLD ничего не защёлкивает");
    check(!s.inputs.imu_healthy, "HOLD снимает признак здоровья: тяги нет");

    // Восстановление без участия человека.
    fc_supervisor_report_imu_permit(FC_IMU_PERMIT_OK, 0, 1, &st, t + 2000);
    check(fc_supervisor_status().inputs.imu_healthy,
          "из HOLD система выходит сама, когда ориентация снова свежая");

    // LOST защёлкивает.
    fc_supervisor_report_imu_permit(FC_IMU_PERMIT_LOST, FC_IMU_POLICY_STALE, 4, &st, t + 3000);
    s = fc_supervisor_status();
    check(s.state == FC_SUP_FAULT, "LOST переводит в отказ");
    check((s.faults_latched & FC_FAULT_IMU_UNHEALTHY) != 0, "LOST защёлкивает");
    check(s.imu_fault.policy_permit == (uint32_t) FC_IMU_PERMIT_LOST,
          "в снимке записан вердикт политики");
    check(s.imu_fault.policy_reasons == FC_IMU_POLICY_STALE, "в снимке записана причина");

    // 17: исправные входы отказ не снимают.
    fc_supervisor_report_imu_permit(FC_IMU_PERMIT_OK, 0, 1, &st, t + 4000);
    check(fc_supervisor_status().state == FC_SUP_FAULT,
          "17 возврат исправных входов отказ НЕ снимает");

    // 18: снятие отказа — отдельное действие, и оно не вооружает.
    check(fc_supervisor_clear_fault(t + 5000), "18 отказ снимается явной командой");
    check(fc_supervisor_status().state != FC_SUP_FAULT, "18 после снятия отказа нет");
    check(fc_supervisor_status().imu_fault.cause == FC_IMU_FAULT_CAUSE_NONE,
          "18 снимок освобождён: следующий отказ будет чем объяснить");
    check(!fc_dual_motor_stats().armed, "18 вооружение НЕ восстанавливается снятием отказа");
}

void test_imu_policy_all(void) {
    test_matrix();
    test_supervisor_integration();
}
