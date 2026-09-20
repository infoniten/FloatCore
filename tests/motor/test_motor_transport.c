// Host-тесты транспорта моторных команд (ТЗ v0.9A §4, §26 пункты 1-2).
//
// Собираются В ЭКСПЕРИМЕНТАЛЬНОМ ПРОФИЛЕ, отдельным двоичным файлом.
// Проверять сериализатор в лабораторной сборке было бы самообманом: там его
// нет как кода, и тест прошёл бы, ничего не проверив.

#include "../../compat/can/fc_vesc_can.h"
#include "../../compat/can/fc_vesc_can_motor.h"
#include "../../compat/safety/fc_build_profile.h"

#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

static int checks, failures;

static void check(bool ok, const char *what) {
    ++checks;
    if (!ok) {
        ++failures;
        printf("      \033[31mFAIL\033[0m %s\n", what);
    } else {
        printf("      \033[32mPASS\033[0m %s\n", what);
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

int main(void) {
    printf("\n\033[1mТранспорт моторных команд (профиль %s)\033[0m\n", FC_PROFILE_NAME);

    check(FC_MOTOR_BACKEND_AVAILABLE == 1, "2 в экспериментальном профиле backend доступен");
    check(FC_CAN_TX_AVAILABLE == 1, "транспорт мотору доступен");
    note("профиль CAN: %s", FC_CAN_PROFILE_NAME);

    // Формат: eid = id | (packet << 8), данные — int32 big-endian в мА.
    // Проверяется против таблицы типов из fc_vesc_can.c, а не против самой
    // себя: разойтись они не должны.
    FcMotorFrame f;
    check(fc_vesc_can_motor_build_current(118, 1.0f, &f) == FC_MOTOR_FRAME_OK,
          "кадр для половины 118 собран");
    check(f.eid == (118u | (1u << 8)), "идентификатор = id | (SET_CURRENT << 8)");
    check(f.len == 4, "длина 4 байта");
    check(f.data[0] == 0x00 && f.data[1] == 0x00 && f.data[2] == 0x03 && f.data[3] == 0xE8,
          "1.000 А кодируется как 1000 мА старшим байтом вперёд");

    FcVescCanId d = fc_vesc_can_decode(f.eid, true);
    check(d.vesc_format && d.controller_id == 118 && d.packet_type == 1,
          "собранный кадр разбирается штатным декодером как SET_CURRENT для 118");
    check(d.is_motor_command, "декодер считает этот тип моторной командой");

    uint8_t id = 0;
    float a = 0;
    check(fc_vesc_can_motor_decode_current(&f, &id, &a), "кадр разбирается обратно");
    check(id == 118 && fabsf(a - 1.0f) < 1e-6f, "обратный разбор даёт исходные значения");

    // Отрицательный ток — это торможение, и он обязан кодироваться знаково.
    check(fc_vesc_can_motor_build_current(100, -0.5f, &f) == FC_MOTOR_FRAME_OK,
          "отрицательный ток допустим");
    check(fc_vesc_can_motor_decode_current(&f, &id, &a) && id == 100 && fabsf(a + 0.5f) < 1e-6f,
          "отрицательный ток кодируется и разбирается со знаком");

    // НЕ зажимает. Значение, дошедшее сюда неконечным, означает дыру в
    // проверках выше, и её надо увидеть, а не замазать округлением.
    check(fc_vesc_can_motor_build_current(118, NAN, &f) == FC_MOTOR_FRAME_BAD_VALUE,
          "NaN отвергается, а не превращается в ноль");
    check(fc_vesc_can_motor_build_current(118, INFINITY, &f) == FC_MOTOR_FRAME_BAD_VALUE,
          "Inf отвергается");

    // Широковещательный адрес: на него ответили бы обе половины, и понятие
    // «команда одной половине» перестало бы существовать.
    check(fc_vesc_can_motor_build_current(255, 1.0f, &f) == FC_MOTOR_FRAME_BAD_TARGET,
          "broadcast 255 отвергается");
    check(fc_vesc_can_motor_build_current(50, 1.0f, &f) == FC_MOTOR_FRAME_BAD_TARGET,
          "собственный адрес FloatCore отвергается");

    // Разрешён ровно один тип пакета. Проверяется тем, что собрать другой
    // нечем: функции для duty, rpm и position не существует.
    check(FC_CAN_PACKET_SET_CURRENT == 1u, "разрешён ровно один тип: SET_CURRENT");

    printf("\n================================================================\n");
    if (failures) {
        printf("\033[31mПровалено %d из %d\033[0m\n", failures, checks);
        return 1;
    }
    printf("\033[32mВсе проверки пройдены\033[0m (%d)\n", checks);
    return 0;
}
