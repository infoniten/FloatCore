// Сценарии host-тестов Refloat (ТЗ v0.2 §5).
//
// Ни один сценарий не трогает реальное железо: выходы на мотор перехватываются
// mock-платформой и mock-реализацией LogicalMotor.
//
// Этот файл НЕ включает заголовки Refloat (см. harness/refloat_facade.h).

#include "scenarios.h"

#include "logical_motor_mock.h"
#include "mock_vesc_if.h"
#include "refloat_facade.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// ------------------------------------------------------------------ фреймворк

static size_t failures;
static bool verbose = true;

void t_reset(void) {
    failures = 0;
}

size_t t_failures(void) {
    return failures;
}

void t_check(bool ok, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    printf("      %s ", ok ? "\033[32mPASS\033[0m" : "\033[31mFAIL\033[0m");
    vprintf(fmt, ap);
    printf("\n");
    va_end(ap);
    if (!ok) {
        ++failures;
    }
}

void t_info(const char *fmt, ...) {
    if (!verbose) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    printf("      \033[90m·\033[0m ");
    vprintf(fmt, ap);
    printf("\n");
    va_end(ap);
}

// ------------------------------------------------------- вспомогательные функции

#define IMU_PERIOD_US 2000  // 500 Гц — целевая частота контура (ТЗ v0.1 §4)
#define DEG2RAD (3.14159265358979f / 180.0f)

/** Выставляет акселерометр/гироскоп и углы «AHRS прошивки» для заданного тангажа. */
static void set_attitude(float pitch_deg, float roll_deg) {
    float p = pitch_deg * DEG2RAD;
    // Вектор гравитации в системе координат доски при наклоне вокруг оси Y.
    // Знак X подобран так, чтобы balance_pitch совпадал по знаку с imu_get_pitch():
    // именно эта договорённость об осях — риск R-07, и на стенде её придётся
    // подтверждать физическим наклоном доски.
    float accel[3] = {-sinf(p), 0.0f, cosf(p)};
    float gyro[3] = {0.0f, 0.0f, 0.0f};
    mock_imu_set_raw(accel, gyro);
    mock_imu_set_angles_deg(roll_deg, pitch_deg, 0.0f);
}

/** Прогон виртуального времени: шаг = один семпл IMU. */
static void run_for(float seconds) {
    int ticks = (int) (seconds * 1e6f / IMU_PERIOD_US);
    for (int i = 0; i < ticks; ++i) {
        mock_advance_us(IMU_PERIOD_US);
        mock_imu_tick((float) IMU_PERIOD_US * 1e-6f);
    }
}

/** Полная инициализация: mock + Refloat + выход из STARTUP в READY. */
static bool boot(void) {
    mock_init();
    logical_motor_mock_set_clock(mock_now_us);
    logical_motor_init(NULL);
    logical_motor_mock_reset_stats();

    mock_adc_set(0.0f, 0.0f);
    set_attitude(0.0f, 0.0f);

    MockMotorTelemetry tele = {
        .erpm = 0.0f,
        .duty = 0.0f,
        .input_voltage = 75.0f,
        .fet_temp = 30.0f,
        .motor_temp = 30.0f,
    };
    mock_motor_set_telemetry(&tele);

    if (!refloat_facade_start()) {
        t_check(false, "refloat_init() failed");
        return false;
    }
    mock_wait_idle();
    // 2 с: выход из STARTUP, сходимость фильтра ориентации, калибровка частот
    run_for(2.0f);
    return true;
}

static void shutdown(void) {
    refloat_facade_stop();
    mock_deinit();
}

static void engage_footpads(void) {
    mock_adc_set(3.3f, 3.3f);
}

/** Число команд тока, выданных за последние `seconds` секунд. */
static size_t commands_since_clear(float seconds) {
    mock_motor_cmd_clear();
    run_for(seconds);
    return mock_motor_cmd_count_of(MOCK_CMD_CURRENT) +
        mock_motor_cmd_count_of(MOCK_CMD_BRAKE_CURRENT) + mock_motor_cmd_count_of(MOCK_CMD_DUTY);
}

// ------------------------------------------------------------------- сценарии

// 1. Доска ровно, ноги не стоят: система готова, но тяги нет.
static bool sc_board_level(void) {
    if (!boot()) {
        return false;
    }
    RefloatSnapshot s = refloat_facade_snapshot();
    t_info("state=%s balance_pitch=%.2f° imu_freq=%.1f Hz main_freq=%.1f Hz",
           refloat_facade_state_name(s.state), s.balance_pitch, s.imu_frequency, s.main_frequency);

    t_check(s.state == 2 /* READY */, "состояние READY (получено %s)",
            refloat_facade_state_name(s.state));
    t_check(fabsf(s.balance_pitch) < 1.0f, "balance_pitch ≈ 0 (%.3f°)", s.balance_pitch);
    t_check(s.footpad_state == 0, "футпады NONE");

    mock_motor_cmd_clear();
    run_for(0.2f);
    bool found = false;
    MockMotorCmd last = mock_motor_cmd_last(MOCK_CMD_CURRENT, &found);
    t_check(!found || fabsf(last.value) < 0.001f, "тяга отсутствует (последний ток %.3f A)",
            found ? last.value : 0.0f);
    t_check(mock_stats_nan_current_requests() == 0, "NaN в mc_set_current не передавался");
    shutdown();
    return true;
}

// 2. Нос вверх без ног — не должно быть ни запуска, ни тяги.
static bool sc_nose_up(void) {
    if (!boot()) {
        return false;
    }
    set_attitude(15.0f, 0.0f);
    run_for(2.0f);
    RefloatSnapshot s = refloat_facade_snapshot();
    t_info("balance_pitch=%.2f° pitch=%.2f° state=%s", s.balance_pitch, s.pitch,
           refloat_facade_state_name(s.state));
    t_check(s.balance_pitch > 5.0f, "фильтр отработал наклон носом вверх (%.2f°)", s.balance_pitch);
    t_check(s.state == 2, "остаётся READY без футпадов");
    shutdown();
    return true;
}

// 3. Нос вниз — зеркальная проверка знака.
static bool sc_nose_down(void) {
    if (!boot()) {
        return false;
    }
    set_attitude(-15.0f, 0.0f);
    run_for(2.0f);
    RefloatSnapshot s = refloat_facade_snapshot();
    t_info("balance_pitch=%.2f° state=%s", s.balance_pitch, refloat_facade_state_name(s.state));
    t_check(s.balance_pitch < -5.0f, "знак тангажа зеркален (%.2f°)", s.balance_pitch);
    t_check(s.state == 2, "остаётся READY без футпадов");
    shutdown();
    return true;
}

// 4. Постановка ног при ровной доске → RUNNING и появление команд тока.
static bool sc_footpad_engage(void) {
    if (!boot()) {
        return false;
    }
    engage_footpads();
    run_for(0.5f);
    RefloatSnapshot s = refloat_facade_snapshot();
    t_info("state=%s footpads=%s adc=%.1f/%.1f setpoint=%.2f",
           refloat_facade_state_name(s.state), refloat_facade_footpad_name(s.footpad_state),
           s.adc_left, s.adc_right, s.setpoint);

    t_check(s.footpad_state == 3 /* FS_BOTH */, "оба футпада замкнуты");
    t_check(s.state == 3 /* RUNNING */, "состояние RUNNING (получено %s)",
            refloat_facade_state_name(s.state));

    size_t n = commands_since_clear(0.1f);
    t_info("команд мотору за 0.1 с: %zu (ожидается ~%d при 500 Гц)", n, 50);
    t_check(n >= 40 && n <= 60, "команда мотору выдаётся на каждый семпл IMU (%zu за 0.1 с)", n);

    LogicalMotorMockStats st = logical_motor_mock_stats();
    t_info("LogicalMotor: %u запросов тока, %u keepalive, %u CAN-кадров ушло бы",
           st.current_requests, st.keepalives, st.would_send_can_frames);
    t_check(st.keepalives > 0, "timeout_reset доходит до LogicalMotor (watchdog жив)");
    shutdown();
    return true;
}

// 5. Снятие одной ноги на месте → фолт и возврат в READY.
static bool sc_one_footpad_release(void) {
    if (!boot()) {
        return false;
    }
    engage_footpads();
    run_for(0.5f);
    t_check(refloat_facade_snapshot().state == 3, "предусловие: RUNNING");

    mock_adc_set(3.3f, 0.0f);  // правая нога снята
    run_for(1.0f);

    RefloatSnapshot s = refloat_facade_snapshot();
    t_info("state=%s footpads=%s stop=%s", refloat_facade_state_name(s.state),
           refloat_facade_footpad_name(s.footpad_state), refloat_facade_stop_name(s.stop_condition));
    t_check(s.footpad_state == 1 /* FS_LEFT */, "распознан одиночный футпад");
    t_check(s.state == 2 /* READY */, "выход из RUNNING (получено %s)",
            refloat_facade_state_name(s.state));
    t_check(s.stop_condition == 3 || s.stop_condition == 4,
            "причина останова — футпад (%s)", refloat_facade_stop_name(s.stop_condition));
    shutdown();
    return true;
}

// 6. IMU замолчал: контур перестаёт получать такты.
static bool sc_imu_timeout(void) {
    if (!boot()) {
        return false;
    }
    engage_footpads();
    run_for(0.5f);
    t_check(refloat_facade_snapshot().state == 3, "предусловие: RUNNING");

    mock_imu_set_stalled(true);
    mock_motor_cmd_clear();
    LogicalMotorMockStats before = logical_motor_mock_stats();
    run_for(0.5f);  // главный поток продолжает крутиться, IMU молчит
    LogicalMotorMockStats after = logical_motor_mock_stats();

    size_t cmds = mock_motor_cmd_count_of(MOCK_CMD_CURRENT) +
        mock_motor_cmd_count_of(MOCK_CMD_BRAKE_CURRENT) + mock_motor_cmd_count_of(MOCK_CMD_DUTY);
    uint32_t keepalives = after.keepalives - before.keepalives;

    t_info("за 0.5 с без IMU: команд мотору %zu, keepalive %u", cmds, keepalives);
    t_check(cmds == 0, "при молчащем IMU новых команд мотору нет (%zu)", cmds);
    t_check(keepalives == 0,
            "watchdog не продлевается: VESC снимет тягу по timeout_msec (%u)", keepalives);
    t_info("ВЫВОД: сам Refloat не детектирует пропажу IMU — это обязанность supervisor-а");
    shutdown();
    return true;
}

// 7/8. Пропажа одного из ESC.
static bool esc_timeout(int esc, const char *label) {
    if (!boot()) {
        return false;
    }
    engage_footpads();
    run_for(0.5f);
    t_check(refloat_facade_snapshot().state == 3, "предусловие: RUNNING");
    t_check(logical_motor_healthy(), "предусловие: оба ESC живы");

    logical_motor_mock_set_esc_alive(esc, false);
    run_for(0.2f);

    LogicalMotorTelemetry tm = logical_motor_telemetry();
    t_info("%s offline: faults=0x%02x a_alive=%d b_alive=%d", label, tm.faults, tm.esc_a_alive,
           tm.esc_b_alive);
    t_check(!logical_motor_healthy(), "LogicalMotor сообщает о неисправности");
    t_check((tm.faults & (esc == 0 ? 0x01u : 0x02u)) != 0, "выставлен флаг таймаута %s", label);

    RefloatSnapshot s = refloat_facade_snapshot();
    t_check(s.state == 3, "Refloat об отказе не знает и продолжает работу (%s)",
            refloat_facade_state_name(s.state));
    t_info("ВЫВОД: остановка при отказе ESC — обязанность supervisor-а, не Refloat");
    shutdown();
    return true;
}

static bool sc_esc_a_timeout(void) {
    return esc_timeout(0, "ESC A");
}

static bool sc_esc_b_timeout(void) {
    return esc_timeout(1, "ESC B");
}

// 9. Фолт прошивки VESC во время движения.
static bool sc_injected_fault(void) {
    if (!boot()) {
        return false;
    }
    engage_footpads();
    run_for(0.5f);
    t_check(refloat_facade_snapshot().state == 3, "предусловие: RUNNING");

    MockMotorTelemetry tele = {
        .erpm = 0.0f,
        .input_voltage = 75.0f,
        .fet_temp = 30.0f,
        .motor_temp = 30.0f,
        .fault_code = 4,  // FAULT_CODE_ABS_OVER_CURRENT
    };
    mock_motor_set_telemetry(&tele);
    logical_motor_mock_set_esc_fault(0, 4);
    run_for(0.5f);

    RefloatSnapshot s = refloat_facade_snapshot();
    t_info("после фолта прошивки: state=%s stop=%s", refloat_facade_state_name(s.state),
           refloat_facade_stop_name(s.stop_condition));
    t_check(s.state == 3,
            "Refloat НЕ останавливается по mc_get_fault (только алерт) — состояние %s",
            refloat_facade_state_name(s.state));
    t_check(!logical_motor_healthy(), "LogicalMotor помечает фолт ESC");
    t_info("ВЫВОД: снятие тяги по fault code — обязанность supervisor-а");
    shutdown();
    return true;
}

// 10. Некорректные (NaN) данные IMU: отдельно акселерометр и гироскоп.
static bool sc_nan_imu_sample(void) {
    if (!boot()) {
        return false;
    }
    engage_footpads();
    run_for(0.5f);
    t_check(refloat_facade_snapshot().state == 3, "предусловие: RUNNING");

    float zero[3] = {0.0f, 0.0f, 0.0f};
    float nan3[3] = {NAN, NAN, NAN};

    // --- 10a. NaN в акселерометре
    logical_motor_mock_reset_stats();
    mock_motor_cmd_clear();
    mock_imu_set_raw(nan3, zero);
    run_for(0.1f);

    RefloatSnapshot s = refloat_facade_snapshot();
    LogicalMotorMockStats st = logical_motor_mock_stats();
    t_info("10a NaN accel: balance_pitch=%.3f, NaN на выходе %zu раз, invalid=%u",
           s.balance_pitch, mock_stats_nan_current_requests(), st.invalid_requests);
    t_check(!isnan(s.balance_pitch),
            "NaN в акселерометре безвреден: balance_filter.c:86 отбрасывает ветку по "
            "`accel_norm > 0.01` (сравнение с NaN ложно)");
    t_check(mock_stats_nan_current_requests() == 0, "NaN не дошёл до выхода на мотор");

    // --- 10b. NaN в гироскопе
    logical_motor_mock_reset_stats();
    mock_motor_cmd_clear();
    float accel_ok[3] = {0.0f, 0.0f, 1.0f};
    mock_imu_set_raw(accel_ok, nan3);
    run_for(0.1f);

    s = refloat_facade_snapshot();
    st = logical_motor_mock_stats();
    size_t nan_out = mock_stats_nan_current_requests();
    size_t brakes = mock_motor_cmd_count_of(MOCK_CMD_BRAKE_CURRENT);
    size_t duties = mock_motor_cmd_count_of(MOCK_CMD_DUTY);
    size_t currents = mock_motor_cmd_count_of(MOCK_CMD_CURRENT);

    bool have_cur = false;
    MockMotorCmd last_cur = mock_motor_cmd_last(MOCK_CMD_CURRENT, &have_cur);
    t_info("10b NaN gyro: balance_pitch=%.3f, NaN на выходе %zu, ток/тормоз/duty = %zu/%zu/%zu, "
           "последний ток=%.3f A, traction_control=%d",
           s.balance_pitch, nan_out, currents, brakes, duties,
           have_cur ? last_cur.value : 0.0f, s.traction_control);

    t_check(isnan(s.balance_pitch),
            "NaN в гироскопе НЕ фильтруется Refloat: интегрируется прямо в кватернион");
    t_check(nan_out == 0,
            "NaN всё же не доходит до mc_set_current: motor_control.c:112 проверяет isnan()");
    t_check(have_cur && fabsf(last_cur.value) < 0.001f,
            "выход молча вырождается в нулевой ток (%.3f A) — доска перестаёт "
            "балансировать, оставаясь в RUNNING: тихий отказ, который обязан ловить supervisor",
            have_cur ? last_cur.value : NAN);
    t_check(s.state == 3,
            "при этом Refloat остаётся в RUNNING (%s): сам он отказ не распознаёт",
            refloat_facade_state_name(s.state));
    t_check(st.invalid_requests == 0,
            "LogicalMotor некорректных значений не видит — проверки isfinite() недостаточно, "
            "нужен контроль валидности данных IMU на входе");

    // --- 10c. восстановление
    mock_imu_set_raw(accel_ok, zero);
    run_for(0.5f);
    s = refloat_facade_snapshot();
    t_info("10c после возврата корректных данных: balance_pitch=%.3f", s.balance_pitch);
    t_check(isnan(s.balance_pitch),
            "кватернион НЕ восстанавливается сам — требуется перезапуск фильтра "
            "(обязанность compat-слоя)");

    shutdown();
    return true;
}

const Scenario SCENARIOS[] = {
    {"1. board level", sc_board_level},
    {"2. nose-up", sc_nose_up},
    {"3. nose-down", sc_nose_down},
    {"4. footpad engage", sc_footpad_engage},
    {"5. one footpad release", sc_one_footpad_release},
    {"6. IMU timeout", sc_imu_timeout},
    {"7. ESC A timeout", sc_esc_a_timeout},
    {"8. ESC B timeout", sc_esc_b_timeout},
    {"9. injected VESC fault", sc_injected_fault},
    {"10. NaN IMU sample", sc_nan_imu_sample},
};

const size_t SCENARIO_COUNT = sizeof(SCENARIOS) / sizeof(SCENARIOS[0]);
