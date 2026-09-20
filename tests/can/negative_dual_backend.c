// Негативный тест компиляции (ТЗ v0.8C §8, §16, инвариант I2).
//
// Этот файл ОБЯЗАН НЕ КОМПИЛИРОВАТЬСЯ в профиле LAB_SAFE. Он проверяет
// свойство, которое появилось именно на v0.8C: у координатора двух половин
// теперь есть интерфейс транспорта, и надо доказать, что наличие интерфейса
// не создало пути наружу.
//
// Проверяются два независимых способа:
//
//   1. зарегистрировать backend в Motor Gate — прототип объявлен только вне
//      LAB_SAFE, поэтому в лабораторной сборке имени просто нет;
//   2. позвать сериализатор моторной команды по имени — имена отравлены.
//
// Тестовый транспорт из tests/safety собирается и работает: он пишет вызовы в
// массив. Разница между ним и настоящим — не в объявлении, а в том, что
// настоящего сериализатора в репозитории нет.
//
// Если файл собрался — это провал теста, а не успех.
#include "../../compat/can/fc_can_diag.h"
#include "../../compat/motor/fc_motor_transport.h"
#include "../../compat/safety/fc_motor_gate.h"

static bool pretend_send(FcMotorRequestKind kind, float value, void *ctx) {
    (void) kind;
    (void) value;
    (void) ctx;
    return true;
}

void floatcore_attempt_dual_backend(void) {
    // 1. Backend в Motor Gate: имени нет в этом профиле.
    static const FcMotorBackend b = {.name = "smuggled", .send = pretend_send, .ctx = 0};
    fc_motor_gate_set_backend(&b);

    // 2. Сериализатор моторной команды: имя отравлено.
    fc_can_set_current(118, 5.0f);
}
