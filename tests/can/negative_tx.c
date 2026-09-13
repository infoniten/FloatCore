// Негативный тест компиляции (ТЗ v0.7A §6, §14).
//
// Этот файл ОБЯЗАН НЕ КОМПИЛИРОВАТЬСЯ в пассивном профиле. Если он собрался,
// значит в сборке появился путь передачи в CAN, и это провал теста, а не
// успех. Проверяется целью `make can-negative-test`.
#include "../../platform/esp32/main/fc_can_passive.h"

void floatcore_attempt_can_transmit(void) {
    fc_can_transmit(0x123, (const unsigned char *) "\x01", 1);
}
