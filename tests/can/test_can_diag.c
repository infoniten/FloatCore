// Host-тесты диагностического слоя CAN (ТЗ v0.7B §19).
//
// Собираются в профиле ACTIVE_DIAG — единственном, где слой вообще
// существует. Проверяется логика, которая на плате исполняется дословно та
// же: сборка кадра, белый список, сборка ответа, CRC, темп, здоровье узлов.
//
// Отдельный двоичный файл, а не ветка внутри общего набора: профиль — это
// свойство единицы трансляции, и проверять его в той же сборке, где включён
// пассивный, было бы самообманом.

#include "../../compat/can/fc_can_diag.h"
#include "../../compat/can/fc_can_health.h"
#include "../../compat/can/fc_vesc_can.h"
#include "../../compat/safety/fc_build_profile.h"
#include "../../compat/vesc_protocol/packet.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static int g_fail = 0;
static int g_checks = 0;

static void check(bool ok, const char *what) {
    ++g_checks;
    printf(ok ? "      \033[32mPASS\033[0m %s\n" : "      \033[31mFAIL\033[0m %s\n", what);
    if (!ok) {
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

// -------------------------------------------------------------- профиль

static void test_profile(void) {
    printf("\n\033[1mПрофиль сборки\033[0m\n");
    check(FC_CAN_DIAG_TX_AVAILABLE == 1, "диагностическая передача доступна");
    check(FC_CAN_RX_AVAILABLE == 1, "приём доступен");
    check(FC_CAN_TX_AVAILABLE == 0, "транспорт команд мотору отсутствует");
    check(FC_MOTOR_BACKEND_AVAILABLE == 0, "backend выхода на мотор отсутствует");
    note("профиль CAN: %s, профиль сборки: %s", FC_CAN_PROFILE_NAME, FC_PROFILE_NAME);
}

// ---------------------------------------------------------- белый список

static void test_whitelist_allows(void) {
    printf("\n\033[1mБелый список: разрешённые запросы собираются\033[0m\n");

    FcCanDiagFrame f;
    check(fc_can_diag_build(FC_CAN_DIAG_PING, 50, 118, &f) == FC_CAN_DIAG_BUILD_OK,
          "PING собирается");
    // eid = target | (17 << 8) = 118 | 0x1100 = 0x1176
    check(f.eid == 0x1176u, "PING: eid = target | (PING << 8)");
    check(f.len == 1 && f.data[0] == 50, "PING несёт наш номер, чтобы получить PONG");

    struct {
        FcCanDiagRequest req;
        uint8_t comm;
        const char *name;
    } expect[] = {
        {FC_CAN_DIAG_FW_VERSION, 0, "FW_VERSION"},
        {FC_CAN_DIAG_VALUES, 4, "GET_VALUES"},
        {FC_CAN_DIAG_MCCONF, 14, "GET_MCCONF"},
        {FC_CAN_DIAG_APPCONF, 17, "GET_APPCONF"},
    };
    for (size_t i = 0; i < sizeof(expect) / sizeof(expect[0]); ++i) {
        char msg[96];
        check(fc_can_diag_build(expect[i].req, 50, 118, &f) == FC_CAN_DIAG_BUILD_OK,
              "запрос собирается");
        // eid = target | (PROCESS_SHORT_BUFFER << 8) = 118 | 0x0800 = 0x0876
        snprintf(msg, sizeof(msg), "%s идёт через PROCESS_SHORT_BUFFER", expect[i].name);
        check(f.eid == 0x0876u, msg);
        snprintf(msg, sizeof(msg), "%s: [наш id][режим 0][COMM %u]", expect[i].name,
                 expect[i].comm);
        check(f.len == 3 && f.data[0] == 50 && f.data[1] == 0 && f.data[2] == expect[i].comm, msg);
        snprintf(msg, sizeof(msg), "%s: COMM доказанно read-only", expect[i].name);
        check(fc_vesc_comm_is_read_only(f.data[2]), msg);
    }

    check(fc_can_diag_build(FC_CAN_DIAG_REQUEST_COUNT, 50, 118, &f) ==
              FC_CAN_DIAG_BUILD_UNKNOWN_REQUEST,
          "номер вне перечисления отвергается");
}

static void test_whitelist_rejects_motor(void) {
    printf("\n\033[1mБелый список: ни один разрешённый запрос не моторный\033[0m\n");

    // Ни один собираемый запрос не пользуется моторным типом пакета.
    for (int r = 0; r < FC_CAN_DIAG_REQUEST_COUNT; ++r) {
        uint32_t t = fc_can_diag_request_packet_type((FcCanDiagRequest) r);
        char msg[96];
        snprintf(msg, sizeof(msg), "%s: тип пакета %u не моторный",
                 fc_can_diag_request_name((FcCanDiagRequest) r), (unsigned) t);
        check(!fc_vesc_can_is_motor_packet(t), msg);
        snprintf(msg, sizeof(msg), "%s: тип пакета %u не меняет состояние ESC",
                 fc_can_diag_request_name((FcCanDiagRequest) r), (unsigned) t);
        check(!fc_vesc_can_is_state_changing(t), msg);
    }

    // Каждый моторный тип из bldc поимённо.
    const struct {
        uint32_t t;
        const char *name;
    } motor[] = {
        {0, "CAN_PACKET_SET_DUTY"},
        {1, "CAN_PACKET_SET_CURRENT"},
        {2, "CAN_PACKET_SET_CURRENT_BRAKE"},
        {3, "CAN_PACKET_SET_RPM"},
        {4, "CAN_PACKET_SET_POS"},
        {10, "CAN_PACKET_SET_CURRENT_REL"},
        {11, "CAN_PACKET_SET_CURRENT_BRAKE_REL"},
        {12, "CAN_PACKET_SET_CURRENT_HANDBRAKE"},
        {13, "CAN_PACKET_SET_CURRENT_HANDBRAKE_REL"},
    };
    for (size_t i = 0; i < sizeof(motor) / sizeof(motor[0]); ++i) {
        char msg[96];
        snprintf(msg, sizeof(msg), "%s опознан как моторный", motor[i].name);
        check(fc_vesc_can_is_motor_packet(motor[i].t), msg);
        bool used = false;
        for (int r = 0; r < FC_CAN_DIAG_REQUEST_COUNT; ++r) {
            if (fc_can_diag_request_packet_type((FcCanDiagRequest) r) == motor[i].t) {
                used = true;
            }
        }
        snprintf(msg, sizeof(msg), "%s не используется ни одним запросом", motor[i].name);
        check(!used, msg);
    }

    // Прочие типы, меняющие состояние ESC.
    const struct {
        uint32_t t;
        const char *name;
    } dangerous[] = {
        {19, "DETECT_APPLY_ALL_FOC"}, {22, "CONF_STORE_CURRENT_LIMITS"},
        {24, "CONF_STORE_CURRENT_LIMITS_IN"}, {26, "CONF_STORE_FOC_ERPMS"},
        {30, "CONF_STORE_BATTERY_CUT"}, {31, "SHUTDOWN"},
        {36, "IO_BOARD_SET_OUTPUT_DIGITAL"}, {37, "IO_BOARD_SET_OUTPUT_PWM"},
        {42, "BMS_BAL"}, {47, "PSW_SWITCH"},
        {55, "UPDATE_PID_POS_OFFSET"}, {63, "UPDATE_BAUD"},
    };
    for (size_t i = 0; i < sizeof(dangerous) / sizeof(dangerous[0]); ++i) {
        char msg[96];
        snprintf(msg, sizeof(msg), "%s помечен как меняющий состояние", dangerous[i].name);
        check(fc_vesc_can_is_state_changing(dangerous[i].t), msg);
    }
}

static void test_comm_whitelist(void) {
    printf("\n\033[1mБелый список COMM: опасные номера не проходят\033[0m\n");

    const struct {
        uint8_t c;
        const char *name;
    } forbidden[] = {
        {1, "COMM_JUMP_TO_BOOTLOADER"}, {2, "COMM_ERASE_NEW_APP"},
        {3, "COMM_WRITE_NEW_APP_DATA"}, {5, "COMM_SET_DUTY"},
        {6, "COMM_SET_CURRENT"}, {7, "COMM_SET_CURRENT_BRAKE"},
        {8, "COMM_SET_RPM"}, {9, "COMM_SET_POS"},
        {10, "COMM_SET_HANDBRAKE"}, {11, "COMM_SET_DETECT"},
        {12, "COMM_SET_SERVO_POS"}, {13, "COMM_SET_MCCONF"},
        {16, "COMM_SET_APPCONF"}, {20, "COMM_TERMINAL_CMD"},
        {24, "COMM_DETECT_MOTOR_PARAM"}, {25, "COMM_DETECT_MOTOR_R_L"},
        {26, "COMM_DETECT_MOTOR_FLUX_LINKAGE"}, {27, "COMM_DETECT_ENCODER"},
        {28, "COMM_DETECT_HALL_FOC"}, {29, "COMM_REBOOT"},
        {30, "COMM_ALIVE"}, {36, "COMM_CUSTOM_APP_DATA"},
        {48, "COMM_SET_MCCONF_TEMP"}, {49, "COMM_SET_MCCONF_TEMP_SETUP"},
    };
    for (size_t i = 0; i < sizeof(forbidden) / sizeof(forbidden[0]); ++i) {
        char msg[96];
        snprintf(msg, sizeof(msg), "%s не read-only", forbidden[i].name);
        check(!fc_vesc_comm_is_read_only(forbidden[i].c), msg);
        bool used = false;
        for (int r = 0; r < FC_CAN_DIAG_REQUEST_COUNT; ++r) {
            if (fc_can_diag_request_comm_id((FcCanDiagRequest) r) == forbidden[i].c) {
                used = true;
            }
        }
        snprintf(msg, sizeof(msg), "%s не используется ни одним запросом", forbidden[i].name);
        check(!used, msg);
    }

    // Умолчание — «нельзя»: номера из будущих версий прошивки тоже опасны.
    int allowed = 0;
    for (int c = 0; c < 256; ++c) {
        if (fc_vesc_comm_is_read_only((uint8_t) c)) {
            ++allowed;
        }
    }
    check(allowed == 7, "разрешённых номеров COMM ровно семь");
    note("разрешены: FW_VERSION, GET_VALUES, GET_MCCONF(+DEFAULT), GET_APPCONF(+DEFAULT), "
         "GET_VALUES_SELECTIVE");
}

static void test_macro_matches_function(void) {
    printf("\n\033[1mПроверка компиляции и проверка исполнения совпадают\033[0m\n");

    // Белый список проверяется дважды: макросом на этапе компиляции и
    // функцией во время работы. Разъехаться незаметно они не должны.
    int mismatch_motor = 0, mismatch_comm = 0;
    for (int i = 0; i < 256; ++i) {
        if ((bool) FC_CAN_TYPE_IS_MOTOR((uint32_t) i) != fc_vesc_can_is_motor_packet((uint32_t) i)) {
            ++mismatch_motor;
        }
        if ((bool) FC_COMM_IS_READ_ONLY((uint8_t) i) != fc_vesc_comm_is_read_only((uint8_t) i)) {
            ++mismatch_comm;
        }
    }
    check(mismatch_motor == 0, "FC_CAN_TYPE_IS_MOTOR совпадает с fc_vesc_can_is_motor_packet");
    check(mismatch_comm == 0, "FC_COMM_IS_READ_ONLY совпадает с fc_vesc_comm_is_read_only");
}

static void test_addressing(void) {
    printf("\n\033[1mАдресация\033[0m\n");
    FcCanDiagFrame f;
    check(fc_can_diag_build(FC_CAN_DIAG_PING, 50, 255, &f) == FC_CAN_DIAG_BUILD_BROADCAST,
          "широковещательный адрес 255 запрещён");
    check(fc_can_diag_build(FC_CAN_DIAG_PING, 50, 50, &f) == FC_CAN_DIAG_BUILD_SELF,
          "запрос самому себе отвергается");
    check(fc_can_diag_build(FC_CAN_DIAG_PING, 50, 100, &f) == FC_CAN_DIAG_BUILD_OK &&
              (f.eid & 0xFFu) == 100u,
          "адресуется именно указанный контроллер");
    check(fc_can_diag_build(FC_CAN_DIAG_PING, 50, 118, &f) == FC_CAN_DIAG_BUILD_OK &&
              (f.eid & 0xFFu) == 118u,
          "вторая половина адресуется отдельно");
    check(FC_CAN_SELF_ID != 118 && FC_CAN_SELF_ID != 100,
          "наш номер не совпадает ни с одной половиной FSESC");
}

// ------------------------------------------------------------ сборка ответа

static void test_rx_pong(void) {
    printf("\n\033[1mОтвет: PONG\033[0m\n");
    FcCanDiagRx rx;
    fc_can_diag_rx_init(&rx, 50);

    uint8_t d[2] = {118, 0};  // [номер ответившего][тип железа]
    check(fc_can_diag_rx_frame(&rx, 50u | (18u << 8), d, 2) == FC_CAN_DIAG_RX_COMPLETE,
          "PONG на наш адрес принят");
    check(rx.from_id == 118, "ответивший опознан");
    check(rx.pongs == 1, "счётчик PONG");

    // Ответ, адресованный не нам: половины FSESC разговаривают между собой.
    fc_can_diag_rx_init(&rx, 50);
    check(fc_can_diag_rx_frame(&rx, 100u | (18u << 8), d, 2) == FC_CAN_DIAG_RX_IGNORED,
          "чужой PONG игнорируется");
    check(rx.foreign == 1, "чужой кадр посчитан");
    check(!rx.complete, "чужой ответ не подменяет наш");
}

static void test_rx_short(void) {
    printf("\n\033[1mОтвет: короткий буфер\033[0m\n");
    FcCanDiagRx rx;
    fc_can_diag_rx_init(&rx, 50);

    // [кто ответил][режим][полезная нагрузка]
    uint8_t d[6] = {118, 1, 0x00, 0x06, 0x06, 0x00};
    check(fc_can_diag_rx_frame(&rx, 50u | (8u << 8), d, 6) == FC_CAN_DIAG_RX_COMPLETE,
          "короткий ответ собран");
    check(rx.payload_len == 4, "обёртка снята, осталась полезная нагрузка");
    check(rx.payload[0] == 0x00 && rx.payload[3] == 0x00, "содержимое не искажено");

    fc_can_diag_rx_init(&rx, 50);
    check(fc_can_diag_rx_frame(&rx, 50u | (8u << 8), d, 1) == FC_CAN_DIAG_RX_IGNORED,
          "обрезанный кадр не разбирается как ответ");
}

static void test_rx_long(void) {
    printf("\n\033[1mОтвет: длинный буфер с CRC\033[0m\n");

    uint8_t payload[100];
    for (int i = 0; i < 100; ++i) {
        payload[i] = (uint8_t) (i * 7 + 3);
    }
    uint16_t crc = vesc_crc16(payload, sizeof(payload));

    FcCanDiagRx rx;
    fc_can_diag_rx_init(&rx, 50);

    // Так шлёт bldc: куски по 7 байт с однобайтовым смещением.
    for (int off = 0; off < 100; off += 7) {
        uint8_t d[8];
        int n = (off + 7 <= 100) ? 7 : 100 - off;
        d[0] = (uint8_t) off;
        memcpy(d + 1, payload + off, (size_t) n);
        FcCanDiagRxResult r = fc_can_diag_rx_frame(&rx, 50u | (5u << 8), d, (uint8_t) (n + 1));
        if (r != FC_CAN_DIAG_RX_PARTIAL) {
            check(false, "фрагмент принят как частичный");
            return;
        }
    }
    check(true, "все фрагменты приняты");

    uint8_t fin[6] = {118, 1, 0, 100, (uint8_t) (crc >> 8), (uint8_t) (crc & 0xFF)};
    check(fc_can_diag_rx_frame(&rx, 50u | (7u << 8), fin, 6) == FC_CAN_DIAG_RX_COMPLETE,
          "завершающий кадр собрал ответ");
    check(rx.payload_len == 100, "длина ответа как объявлена");
    check(memcmp(rx.payload, payload, 100) == 0, "содержимое совпало байт в байт");
    check(rx.long_replies == 1, "счётчик длинных ответов");

    // Испорченная CRC.
    fc_can_diag_rx_init(&rx, 50);
    uint8_t d[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    fc_can_diag_rx_frame(&rx, 50u | (5u << 8), d, 8);
    uint8_t bad[6] = {118, 1, 0, 7, 0xDE, 0xAD};
    check(fc_can_diag_rx_frame(&rx, 50u | (7u << 8), bad, 6) == FC_CAN_DIAG_RX_CRC_ERROR,
          "несовпадение CRC — ответ отвергнут");
    check(rx.crc_errors == 1 && !rx.complete, "испорченный ответ не считается принятым");

    // Объявленная длина больше собранного.
    fc_can_diag_rx_init(&rx, 50);
    fc_can_diag_rx_frame(&rx, 50u | (5u << 8), d, 8);
    uint8_t big[6] = {118, 1, 0x01, 0x00, 0x00, 0x00};  // 256 байт, а собрано 7
    check(fc_can_diag_rx_frame(&rx, 50u | (7u << 8), big, 6) == FC_CAN_DIAG_RX_OVERFLOW,
          "объявленная длина больше собранного — отказ");

    // Смещение за пределы буфера.
    fc_can_diag_rx_init(&rx, 50);
    uint8_t far[8] = {0xFF, 0xF8, 1, 2, 3, 4, 5, 6};
    check(fc_can_diag_rx_frame(&rx, 50u | (6u << 8), far, 8) == FC_CAN_DIAG_RX_OVERFLOW,
          "смещение за пределы буфера не переполняет память");
}

static void test_rx_misc(void) {
    printf("\n\033[1mОтвет: посторонние и повторные кадры\033[0m\n");
    FcCanDiagRx rx;
    fc_can_diag_rx_init(&rx, 50);

    uint8_t st[8] = {0};
    check(fc_can_diag_rx_frame(&rx, 118u | (9u << 8), st, 8) == FC_CAN_DIAG_RX_IGNORED,
          "периодический STATUS не путается с ответом");
    check(rx.foreign == 0, "STATUS не считается чужим ответом — он вообще не ответ");

    uint8_t d[2] = {118, 0};
    check(fc_can_diag_rx_frame(&rx, 50u | (18u << 8), d, 2) == FC_CAN_DIAG_RX_COMPLETE,
          "первый PONG принят");
    uint8_t d2[2] = {100, 0};
    check(fc_can_diag_rx_frame(&rx, 50u | (18u << 8), d2, 2) == FC_CAN_DIAG_RX_COMPLETE,
          "повторный ответ тоже разбирается");
    check(rx.from_id == 100 && rx.pongs == 2,
          "повтор перезаписывает результат, а не смешивается с ним");

    fc_can_diag_rx_abort(&rx);
    check(!rx.complete && rx.payload_len == 0, "сброс по таймауту забывает недособранное");
}

// -------------------------------------------------------------- темп

static void test_rate_limit(void) {
    printf("\n\033[1mОграничение темпа запросов\033[0m\n");
    FcCanDiagRate r;
    fc_can_diag_rate_init(&r, FC_CAN_DIAG_MIN_INTERVAL_US);

    check(fc_can_diag_rate_allow(&r, 1000000), "первый запрос проходит");
    check(!fc_can_diag_rate_allow(&r, 1000000 + 1000), "через 1 мс — придержан");
    check(!fc_can_diag_rate_allow(&r, 1000000 + 199000), "через 199 мс — придержан");
    check(fc_can_diag_rate_allow(&r, 1000000 + 200000), "через 200 мс — проходит");
    check(r.allowed == 2 && r.throttled == 2, "счётчики темпа");
    note("минимальный интервал %u мкс = не чаще %.1f запросов в секунду",
         FC_CAN_DIAG_MIN_INTERVAL_US, 1e6 / FC_CAN_DIAG_MIN_INTERVAL_US);

    // 500 Гц опроса не должно проходить даже близко.
    fc_can_diag_rate_init(&r, FC_CAN_DIAG_MIN_INTERVAL_US);
    int passed = 0;
    for (int i = 0; i < 500; ++i) {
        if (fc_can_diag_rate_allow(&r, (uint64_t) i * 2000)) {
            ++passed;
        }
    }
    check(passed == 5, "за секунду попыток с шагом 2 мс проходит ровно 5");
}

// ------------------------------------------------------- здоровье узлов

static void test_health(void) {
    printf("\n\033[1mЗдоровье узлов\033[0m\n");
    FcCanHealth h;
    fc_can_health_init(&h, FC_CAN_NODE_STALE_US);
    fc_can_health_expect(&h, 118);
    fc_can_health_expect(&h, 100);

    check(!fc_can_health_all_expected_healthy(&h), "до первого кадра узлы не считаются живыми");

    uint64_t t = 1000000;
    fc_can_health_on_status(&h, 118, t);
    fc_can_health_on_status(&h, 100, t);
    fc_can_health_tick(&h, t);
    check(fc_can_health_all_expected_healthy(&h), "оба узла доступны после статуса");

    // Одна половина замолчала.
    for (int i = 1; i <= 30; ++i) {
        fc_can_health_on_status(&h, 118, t + (uint64_t) i * 20000);
    }
    t += 600000;
    fc_can_health_tick(&h, t);
    check(fc_can_health_node(&h, 118)->healthy, "говорящий узел остался доступен");
    check(!fc_can_health_node(&h, 100)->healthy, "молчащий узел стал недоступен");
    check(!fc_can_health_all_expected_healthy(&h), "сводка учитывает пропавший узел");
    check(fc_can_health_node(&h, 100)->health_drops == 1, "падение доступности посчитано");

    // Вернулся.
    fc_can_health_on_status(&h, 100, t);
    fc_can_health_tick(&h, t);
    check(fc_can_health_all_expected_healthy(&h), "узел вернулся — сводка снова зелёная");

    // Таймаут диагностического запроса при живом статусе узел не хоронит.
    fc_can_health_on_diag_timeout(&h, 118);
    fc_can_health_tick(&h, t);
    check(fc_can_health_node(&h, 118)->healthy,
          "таймаут запроса при живом STATUS не объявляет узел мёртвым");
    check(fc_can_health_node(&h, 118)->diag_timeouts == 1, "таймаут посчитан отдельно");

    // Посторонний узел на шине не считается ожидаемым.
    fc_can_health_on_status(&h, 7, t);
    fc_can_health_tick(&h, t);
    check(!fc_can_health_node(&h, 7)->expected, "незаявленный узел помечен посторонним");
    check(fc_can_health_all_expected_healthy(&h), "посторонний узел не влияет на сводку");

    // Ни одного ожидаемого узла — не повод отвечать «всё хорошо».
    FcCanHealth empty;
    fc_can_health_init(&empty, FC_CAN_NODE_STALE_US);
    check(!fc_can_health_all_expected_healthy(&empty), "пустая таблица не считается здоровой");
}

int main(void) {
    printf("\n\033[1mТесты диагностического CAN (профиль ACTIVE_DIAG)\033[0m\n");
    test_profile();
    test_whitelist_allows();
    test_whitelist_rejects_motor();
    test_comm_whitelist();
    test_macro_matches_function();
    test_addressing();
    test_rx_pong();
    test_rx_short();
    test_rx_long();
    test_rx_misc();
    test_rate_limit();
    test_health();

    printf("\n================================================================\n");
    if (g_fail == 0) {
        printf("\033[32mВсе проверки пройдены\033[0m (%d)\n\n", g_checks);
        return 0;
    }
    printf("\033[31mПровалено %d из %d\033[0m\n\n", g_fail, g_checks);
    return 1;
}
