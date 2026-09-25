// Отложенная запись во flash (ТЗ v0.9I §11-§13, §25).
//
// Тест связывает модуль решения с НАСТОЯЩИМ супервизором, а не с флагом:
// ровно так его зовёт задача сброса на плате. Иначе тест доказывал бы, что
// модуль слушается булева значения, а не что запись не пройдёт в RUNNING.

#include "../../compat/safety/fc_flash_policy.h"
#include "../../compat/safety/fc_supervisor.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

void fc_test_check(bool ok, const char *what);
void fc_test_note(const char *fmt, ...);
#define check fc_test_check
#define note fc_test_note

uint64_t fc_test_bring_up(uint64_t t);

static FcFlashDecision decide(uint64_t now) {
    return fc_flash_policy_decide(fc_supervisor_config_write_allowed(), now);
}

static void test_deferred_in_ready(void) {
    note("запрос в READY откладывается, в DISARMED выполняется");
    uint64_t t = 90000000;
    t = fc_test_bring_up(t);
    fc_flash_policy_init();

    // Принято в DISARMED, но до коммита система ушла в READY — ровно тот путь,
    // что был достижим до v0.9I.
    fc_flash_policy_request(t);
    check(fc_supervisor_request_ready(t += 1000), "перешли в READY");
    check(decide(t += 1000) == FC_FLASH_DEFER, "в READY запись НЕ выполняется");
    FcFlashPolicyStats s = fc_flash_policy_stats();
    check(s.pending, "запрос остаётся ожидающим, а не теряется");
    check(s.executed == 0, "ни одной записи во flash не выполнено");
    check(s.deferred == 1, "эпизод отсрочки учтён");

    // Задача сброса опрашивает часто — счётчик не должен считать опросы.
    for (int i = 0; i < 50; ++i) {
        (void) decide(t += 1000);
    }
    s = fc_flash_policy_stats();
    check(s.deferred == 1, "повторные опросы не раздувают счётчик отсрочек");
    check(s.executed == 0, "и по-прежнему ничего не записано");

    fc_supervisor_disarm(t += 1000);
    check(decide(t += 1000) == FC_FLASH_EXECUTE, "в DISARMED отложенная запись выполняется");
    fc_flash_policy_report(true, 5795, t);
    s = fc_flash_policy_stats();
    check(!s.pending, "после успешной записи ожидания нет");
    check(s.executed == 1, "выполнена ровно одна запись");
    check(s.last_duration_us == 5795, "длительность записи сохранена");
    check(decide(t += 1000) == FC_FLASH_IDLE, "дальше делать нечего");
}

static void test_fault_is_deterministic(void) {
    note("отказ: запись откладывается детерминированно и не выполняется в FAULT");
    uint64_t t = 95000000;
    t = fc_test_bring_up(t);
    fc_flash_policy_init();

    // Восстановимый отказ: его причина задаётся входом, и вход здоров.
    fc_flash_policy_request(t);
    fc_supervisor_raise_fault(FC_FAULT_IMU_UNHEALTHY, t += 1000);
    check(decide(t += 1000) == FC_FLASH_DEFER, "в FAULT запись не выполняется");
    check(fc_flash_policy_stats().pending, "запрос сохранён");

    // Автоматической записи прямо в момент отказа нет: ТЗ §13 запрещает её,
    // если она ломает реальное время. Выполняется она только после явного
    // возврата в безопасное состояние.
    check(fc_supervisor_clear_fault(t += 1000), "восстановимый отказ снят оператором");
    check(fc_supervisor_state() == FC_SUP_DISARMED, "система в DISARMED");
    check(decide(t += 1000) == FC_FLASH_EXECUTE, "теперь отложенная запись выполняется");
    fc_flash_policy_report(true, 5000, t);
}

static void test_internal_fault_keeps_deferred(void) {
    note("невосстановимый отказ: запись отложена до перезагрузки, но не потеряна");
    uint64_t t = 97000000;
    t = fc_test_bring_up(t);
    fc_flash_policy_init();

    fc_flash_policy_request(t);
    fc_supervisor_raise_fault(FC_FAULT_INTERNAL, t += 1000);
    // Нарушенный инвариант кода не снимается по входам (fc_supervisor.c), и
    // писать во flash в таком состоянии — последнее, что стоит делать.
    check(!fc_supervisor_clear_fault(t += 1000), "внутренний отказ оператор не снимает");
    int deferred = 0;
    for (int i = 0; i < 20; ++i) {
        deferred += decide(t += 1000) == FC_FLASH_DEFER;
    }
    check(deferred == 20, "все двадцать опросов в FAULT дали отсрочку");
    FcFlashPolicyStats s = fc_flash_policy_stats();
    check(s.pending, "запрос виден в телеметрии как ожидающий, а не потерян");
    check(s.executed == 0, "во flash ничего не записано");
}

static void test_failure_keeps_pending(void) {
    note("неудачная запись не снимает ожидание");
    fc_flash_policy_init();
    fc_flash_policy_request(1000);
    check(fc_flash_policy_decide(true, 2000) == FC_FLASH_EXECUTE, "запись разрешена");
    fc_flash_policy_report(false, 1200, 3000);
    FcFlashPolicyStats s = fc_flash_policy_stats();
    check(s.pending, "после ошибки данные по-прежнему не сохранены");
    check(s.failed == 1, "ошибка учтена");
    check(fc_flash_policy_decide(true, 4000) == FC_FLASH_EXECUTE, "будет новая попытка");
}

static void test_requests_merge(void) {
    note("повторные просьбы сливаются в одну запись");
    fc_flash_policy_init();
    for (int i = 0; i < 10; ++i) {
        fc_flash_policy_request(1000 + (uint64_t) i);
    }
    FcFlashPolicyStats s = fc_flash_policy_stats();
    check(s.requested == 10, "все просьбы учтены");
    check(s.pending_since_us == 1000, "ожидание отсчитывается от ПЕРВОЙ просьбы");
    check(fc_flash_policy_decide(true, 2000) == FC_FLASH_EXECUTE, "одна запись на все просьбы");
    fc_flash_policy_report(true, 5000, 2000);
    check(fc_flash_policy_decide(true, 3000) == FC_FLASH_IDLE, "второй записи не нужно");
}

void test_flash_policy_all(void) {
    test_deferred_in_ready();
    test_fault_is_deterministic();
    test_internal_fault_keeps_deferred();
    test_failure_keeps_pending();
    test_requests_merge();
}
