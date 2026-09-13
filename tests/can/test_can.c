// Host-тесты слоя CAN (ТЗ v0.7A §14).
//
// Проверяется разбор идентификаторов по формату VESC и гарантии профиля.
// Формат и номера типов взяты из bldc и здесь именно проверяются, а не
// переоткрываются: тест падает, если разбор разойдётся с upstream.

#include "../../compat/can/fc_vesc_can.h"
#include "../../compat/safety/fc_build_profile.h"

#include <stdio.h>
#include <string.h>

void fc_test_check(bool ok, const char *what);
void fc_test_note(const char *fmt, ...);
#define check fc_test_check
#define note fc_test_note

static void test_id_format(void) {
    printf("\n\033[1mРазбор идентификатора: eid = controller_id | (type << 8)\033[0m\n");

    // STATUS (9) от контроллера 118 -> 0x976
    FcVescCanId r = fc_vesc_can_decode((9u << 8) | 118u, true);
    check(r.vesc_format, "расширенный кадр разбирается как VESC");
    check(r.controller_id == 118, "controller_id = младшие 8 бит");
    check(r.packet_type == 9, "тип пакета = старшие биты");
    check(r.known_type && strcmp(r.name, "STATUS") == 0, "тип 9 распознан как STATUS");
    check(r.is_status, "STATUS помечен как периодический статус");
    check(!r.is_motor_command, "STATUS не является командой мотору");
    note("0x%03x -> id=%u type=%u %s", (9u << 8) | 118u, r.controller_id, r.packet_type, r.name);

    // STATUS_6 (58) от контроллера 100 -> 0x3A64
    r = fc_vesc_can_decode((58u << 8) | 100u, true);
    check(r.controller_id == 100 && r.packet_type == 58, "STATUS_6 от контроллера 100");
    check(strcmp(r.name, "STATUS_6") == 0, "имя STATUS_6");

    // Граничный случай: controller_id 255
    r = fc_vesc_can_decode((9u << 8) | 255u, true);
    check(r.controller_id == 255, "controller_id 255 не переполняется");
    // И controller_id 0
    r = fc_vesc_can_decode((9u << 8) | 0u, true);
    check(r.controller_id == 0 && r.packet_type == 9, "controller_id 0 разбирается");
}

static void test_standard_frames(void) {
    printf("\n\033[1mСтандартные кадры протоколом VESC не толкуются\033[0m\n");
    FcVescCanId r = fc_vesc_can_decode(0x123, false);
    check(!r.vesc_format, "11-битный кадр помечен как не-VESC");
    check(r.controller_id == 0 && r.packet_type == 0 && !r.known_type,
          "поля не заполняются: чужой кадр не выдаётся за VESC");
    check(r.name == NULL, "имени нет");
}

static void test_unknown(void) {
    printf("\n\033[1mНеизвестные типы остаются UNKNOWN\033[0m\n");
    // 200 в перечислении bldc отсутствует.
    FcVescCanId r = fc_vesc_can_decode((200u << 8) | 7u, true);
    check(r.vesc_format, "формат всё ещё VESC");
    check(r.controller_id == 7 && r.packet_type == 200, "поля разобраны");
    check(!r.known_type && r.name == NULL, "тип не выдуман");
    check(!r.is_status && !r.is_motor_command, "неизвестный тип не классифицируется");

    // На v0.7A таблица заканчивалась на STATUS_6 = 58, и 59 был примером
    // неизвестного номера. На v0.7B таблица дополнена до 68 по datatypes.h
    // прошивки 6.6, которая реально стоит на стенде, поэтому 59 стал
    // известным типом. Проверяем оба факта: и что таблица дополнена, и что
    // за её пределами номера по-прежнему не выдумываются.
    r = fc_vesc_can_decode((59u << 8) | 1u, true);
    check(r.known_type && strcmp(r.name, "GNSS_TIME") == 0, "таблица дополнена до прошивки 6.6");
    r = fc_vesc_can_decode((68u << 8) | 1u, true);
    check(r.known_type && strcmp(r.name, "BMS_STATUS_5") == 0, "последний известный номер 68");
    r = fc_vesc_can_decode((69u << 8) | 1u, true);
    check(!r.known_type, "номер за пределами таблицы не считается известным");
    r = fc_vesc_can_decode((200u << 8) | 1u, true);
    check(!r.known_type && r.name == NULL, "далёкий номер тоже не выдумывается");
}

static void test_motor_commands(void) {
    printf("\n\033[1mКоманды мотору распознаются поимённо\033[0m\n");
    const struct {
        uint32_t t;
        const char *n;
    } cmds[] = {{0, "SET_DUTY"},   {1, "SET_CURRENT"},      {2, "SET_CURRENT_BRAKE"},
                {3, "SET_RPM"},    {4, "SET_POS"},          {10, "SET_CURRENT_REL"},
                {11, "SET_CURRENT_BRAKE_REL"}, {12, "SET_CURRENT_HANDBRAKE"},
                {13, "SET_CURRENT_HANDBRAKE_REL"}};
    bool all = true;
    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); ++i) {
        FcVescCanId r = fc_vesc_can_decode((cmds[i].t << 8) | 118u, true);
        if (!r.is_motor_command || strcmp(r.name, cmds[i].n) != 0) {
            all = false;
            note("не распознано: тип %u", cmds[i].t);
        }
    }
    check(all, "все девять команд мотору помечены как таковые");

    // Статусы командами не являются — иначе диагностика «кто-то командует
    // мотором» срабатывала бы на штатном трафике.
    bool none = true;
    uint32_t st[] = {9, 14, 15, 16, 27, 58};
    for (size_t i = 0; i < sizeof(st) / sizeof(st[0]); ++i) {
        if (fc_vesc_can_decode((st[i] << 8) | 118u, true).is_motor_command) {
            none = false;
        }
    }
    check(none, "ни один STATUS не помечен как команда мотору");
}

static void test_profile(void) {
    printf("\n\033[1mГарантии профиля сборки\033[0m\n");
    check(FC_CAN_TX_AVAILABLE == 0, "передача в CAN профилем не разрешена");
    check(FC_MOTOR_BACKEND_AVAILABLE == 0, "backend мотора отсутствует");
    note("профиль сборки %s, профиль CAN %s", FC_PROFILE_NAME, FC_CAN_PROFILE_NAME);
#ifdef FLOATCORE_CAN_PASSIVE
    check(FC_CAN_RX_AVAILABLE == 1, "пассивный профиль включает приём");
#else
    check(FC_CAN_RX_AVAILABLE == 0, "без пассивного профиля приёма нет");
#endif
    // Компиляции вызова передачи проверяется отдельной целью make
    // can-negative-test: там успех — это отказ компилятора.
}

static void test_passive_has_no_diag(void) {
    printf("\n\033[1mПассивный профиль: диагностической передачи нет\033[0m\n");
    check(FC_CAN_DIAG_TX_AVAILABLE == 0, "FC_CAN_DIAG_TX_AVAILABLE = 0");
    check(FC_CAN_TX_AVAILABLE == 0, "транспорт команд мотору отсутствует");
    check(FC_CAN_RX_AVAILABLE == 1, "приём при этом доступен");
    // Сам факт, что этот файл собрался, уже доказывает: заголовок
    // fc_can_diag.h в пассивном профиле не объявляет ничего. Обратное
    // проверяется негативным тестом tests/can/negative_diag_in_passive.c.
    note("профиль CAN: %s", FC_CAN_PROFILE_NAME);
}

void test_can_all(void) {
    test_passive_has_no_diag();
    printf("\n\033[1mТесты CAN: разбор идентификаторов VESC и гарантии профиля\033[0m\n");
    test_id_format();
    test_standard_frames();
    test_unknown();
    test_motor_commands();
    test_profile();
}
