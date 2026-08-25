// Тесты протокольного слоя, включая негативные (ТЗ v0.4 §13).
//
// Линкуется ТОЛЬКО с compat/vesc_protocol: ни Refloat, ни mock-платформы,
// ни pthread здесь нет. Если этот бинарник собрался — протокольный слой
// действительно независим от платформы.
//
// Главное требование: парсер не падает, не зависает и не выходит за границы
// буфера ни при каком входе.

#include "../../compat/vesc_protocol/commands.h"
#include "../../compat/vesc_protocol/packet.h"
#include "../../compat/vesc_protocol/vesc_buffer.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ------------------------------------------------------------------ фреймворк

static int failures;
static int checks;

static void check(bool ok, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    ++checks;
    printf("      %s ", ok ? "\033[32mPASS\033[0m" : "\033[31mFAIL\033[0m");
    vprintf(fmt, ap);
    printf("\n");
    va_end(ap);
    if (!ok) {
        ++failures;
    }
}

static void info(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    printf("      \033[90m·\033[0m ");
    vprintf(fmt, ap);
    printf("\n");
    va_end(ap);
}

static void section(const char *name) {
    printf("\n  \033[1m%s\033[0m\n", name);
}

// ------------------------------------------------------------- окружение теста

#define CAPTURE_MAX 65536

typedef struct {
    uint8_t data[CAPTURE_MAX];
    size_t len;
    size_t frames;
} Capture;

static Capture g_tx;
static uint8_t g_last_payload[VESC_PACKET_MAX_PL_LEN];
static size_t g_last_payload_len;
static size_t g_payloads;

static void capture_send(void *ctx, const uint8_t *data, size_t len) {
    (void) ctx;
    if (g_tx.len + len <= CAPTURE_MAX) {
        memcpy(g_tx.data + g_tx.len, data, len);
        g_tx.len += len;
    }
    ++g_tx.frames;
}

static void capture_process(void *ctx, const uint8_t *payload, size_t len) {
    (void) ctx;
    ++g_payloads;
    g_last_payload_len = len < sizeof(g_last_payload) ? len : sizeof(g_last_payload);
    memcpy(g_last_payload, payload, g_last_payload_len);
}

static void capture_reset(void) {
    memset(&g_tx, 0, sizeof(g_tx));
    g_payloads = 0;
    g_last_payload_len = 0;
}

/** Собрать корректный кадр вокруг payload. */
static size_t frame(const uint8_t *payload, size_t len, uint8_t *out) {
    return vesc_packet_encode(payload, len, out, VESC_PACKET_BUFFER_LEN);
}

// ------------------------------------------------------------------- CRC/фрейминг

static void test_crc(void) {
    section("CRC16 (CCITT/XMODEM)");

    // Контрольные векторы стандарта CRC-16/XMODEM
    check(vesc_crc16((const uint8_t *) "123456789", 9) == 0x31C3,
          "crc16(\"123456789\") = 0x%04X, ожидалось 0x31C3",
          vesc_crc16((const uint8_t *) "123456789", 9));
    check(vesc_crc16((const uint8_t *) "", 0) == 0x0000, "crc16(\"\") = 0");
    check(vesc_crc16((const uint8_t *) "A", 1) == 0x58E5, "crc16(\"A\") = 0x%04X",
          vesc_crc16((const uint8_t *) "A", 1));
}

static void test_roundtrip(void) {
    section("Кодирование и разбор кадра");

    VescPacket p;
    vesc_packet_init(&p, capture_send, capture_process, NULL);

    // короткий пакет — стартовый байт 2
    capture_reset();
    uint8_t small[4] = {COMM_FW_VERSION, 1, 2, 3};
    uint8_t buf[VESC_PACKET_BUFFER_LEN];
    size_t n = frame(small, sizeof(small), buf);
    check(n == sizeof(small) + 5 && buf[0] == 2, "len<=255 → стартовый байт 2, кадр %zu байт", n);
    vesc_packet_process_buffer(&p, buf, n);
    check(g_payloads == 1 && g_last_payload_len == 4, "payload разобран (%zu байт)",
          g_last_payload_len);
    check(memcmp(g_last_payload, small, 4) == 0, "содержимое совпадает");

    // длинный пакет — стартовый байт 3
    capture_reset();
    static uint8_t big[400];
    for (size_t i = 0; i < sizeof(big); ++i) {
        big[i] = (uint8_t) i;
    }
    n = frame(big, sizeof(big), buf);
    check(n == sizeof(big) + 6 && buf[0] == 3, "len>255 → стартовый байт 3, кадр %zu байт", n);
    vesc_packet_process_buffer(&p, buf, n);
    check(g_payloads == 1 && g_last_payload_len == sizeof(big), "длинный payload разобран");

    // побайтовая подача — должно работать так же
    capture_reset();
    n = frame(small, sizeof(small), buf);
    for (size_t i = 0; i < n; ++i) {
        vesc_packet_process_byte(&p, buf[i]);
    }
    check(g_payloads == 1, "побайтовая подача даёт тот же результат");

    // два кадра подряд в одном блоке
    capture_reset();
    size_t n1 = frame(small, sizeof(small), buf);
    size_t n2 = frame(small, sizeof(small), buf + n1);
    vesc_packet_process_buffer(&p, buf, n1 + n2);
    check(g_payloads == 2, "два кадра в одном блоке разобраны (%zu)", g_payloads);
}

// --------------------------------------------------------------- негативные тесты

static void test_bad_crc(void) {
    section("Негативный: испорченный CRC");

    VescPacket p;
    vesc_packet_init(&p, capture_send, capture_process, NULL);
    capture_reset();

    uint8_t payload[8] = {COMM_GET_VALUES, 1, 2, 3, 4, 5, 6, 7};
    uint8_t buf[VESC_PACKET_BUFFER_LEN];
    size_t n = frame(payload, sizeof(payload), buf);
    buf[n - 2] ^= 0xFF;  // портим CRC

    vesc_packet_process_buffer(&p, buf, n);
    check(g_payloads == 0, "кадр с плохим CRC не доставлен наверх");
    check(p.stats.crc_errors >= 1, "ошибка CRC зафиксирована (%u)", p.stats.crc_errors);

    // после плохого кадра парсер обязан принять следующий корректный
    n = frame(payload, sizeof(payload), buf);
    vesc_packet_process_buffer(&p, buf, n);
    check(g_payloads == 1, "парсер восстановился и принял следующий кадр");
}

static void test_truncated(void) {
    section("Негативный: обрезанный пакет");

    VescPacket p;
    vesc_packet_init(&p, capture_send, capture_process, NULL);
    capture_reset();

    uint8_t payload[16];
    memset(payload, 0xAB, sizeof(payload));
    payload[0] = COMM_GET_VALUES;
    uint8_t buf[VESC_PACKET_BUFFER_LEN];
    size_t n = frame(payload, sizeof(payload), buf);

    // подаём всё, кроме последних трёх байт
    vesc_packet_process_buffer(&p, buf, n - 3);
    check(g_payloads == 0, "неполный кадр наверх не доставлен");

    // досылаем хвост — кадр должен собраться
    vesc_packet_process_buffer(&p, buf + n - 3, 3);
    check(g_payloads == 1, "после досылки хвоста кадр разобран");

    // обрезанный и брошенный кадр не должен мешать следующему
    capture_reset();
    vesc_packet_process_buffer(&p, buf, n / 2);
    vesc_packet_process_buffer(&p, buf, n);
    check(g_payloads >= 1, "после брошенного огрызка следующий кадр разобран (%zu)", g_payloads);
}

static void test_oversized(void) {
    section("Негативный: превышение длины");

    VescPacket p;
    vesc_packet_init(&p, capture_send, capture_process, NULL);
    capture_reset();

    // Заявляем длину больше максимума: 0xFFFF при лимите 512
    uint8_t hdr[8] = {3, 0xFF, 0xFF, 0, 0, 0, 0, 0};
    vesc_packet_process_buffer(&p, hdr, sizeof(hdr));
    check(g_payloads == 0, "кадр с завышенной длиной не доставлен");
    check(p.stats.oversized >= 1, "превышение длины зафиксировано (%u)", p.stats.oversized);

    // Попытка отправить слишком длинный payload
    static uint8_t huge[VESC_PACKET_MAX_PL_LEN + 1];
    bool sent = vesc_packet_send(&p, huge, sizeof(huge));
    check(!sent && p.stats.send_rejected >= 1, "отправка слишком длинного payload отклонена");

    // Пустой payload
    sent = vesc_packet_send(&p, huge, 0);
    check(!sent, "отправка пустого payload отклонена");
}

static void test_framing_garbage(void) {
    section("Негативный: мусор и неверные стоп-байты");

    VescPacket p;
    vesc_packet_init(&p, capture_send, capture_process, NULL);
    capture_reset();

    // Неверный стартовый байт
    uint8_t garbage[64];
    for (size_t i = 0; i < sizeof(garbage); ++i) {
        garbage[i] = (uint8_t) (0x10 + i);
    }
    vesc_packet_process_buffer(&p, garbage, sizeof(garbage));
    check(g_payloads == 0, "мусор не превращается в пакеты");

    // Корректный кадр с испорченным стоп-байтом
    uint8_t payload[4] = {COMM_ALIVE, 0, 0, 0};
    uint8_t buf[VESC_PACKET_BUFFER_LEN];
    size_t n = frame(payload, sizeof(payload), buf);
    buf[n - 1] = 0x55;
    vesc_packet_process_buffer(&p, buf, n);
    check(g_payloads == 0, "кадр с неверным стоп-байтом отброшен");

    // Нулевая длина
    uint8_t zero_len[5] = {2, 0, 0, 0, 3};
    vesc_packet_process_buffer(&p, zero_len, sizeof(zero_len));
    check(g_payloads == 0, "кадр нулевой длины отброшен");

    // Длина 16 бит там, где хватило бы 8 (запрещено форматом)
    uint8_t short_16b[10] = {3, 0x00, 0x02, 0xAA, 0xBB, 0, 0, 3, 0, 0};
    vesc_packet_process_buffer(&p, short_16b, sizeof(short_16b));
    check(g_payloads == 0, "16-битная длина < 255 отброшена");

    check(p.stats.framing_errors > 0, "ошибки фрейминга посчитаны (%u)", p.stats.framing_errors);
}

static void test_fuzz(void) {
    section("Негативный: поток случайных байт (fuzz)");

    VescPacket p;
    vesc_packet_init(&p, capture_send, capture_process, NULL);
    capture_reset();

    // Детерминированный ГПСЧ: воспроизводимость важнее качества распределения.
    uint32_t state = 0x12345678u;
    const size_t total = 2u * 1000u * 1000u;
    for (size_t i = 0; i < total; ++i) {
        state = state * 1664525u + 1013904223u;
        vesc_packet_process_byte(&p, (uint8_t) (state >> 24));
    }

    info("скормлено %zu случайных байт, кадров «разобрано» %zu, CRC-ошибок %u, "
         "фрейминг-ошибок %u",
         total, g_payloads, p.stats.crc_errors, p.stats.framing_errors);
    check(true, "парсер не упал и не завис на случайном потоке");
    check(p.stats.bytes_in == total, "все байты обработаны (%u)", p.stats.bytes_in);

    // После фаззинга парсер обязан восстановиться.
    //
    // Мгновенного восстановления не бывает и в самой прошивке VESC: если
    // последние случайные байты выглядели как заголовок длинного пакета,
    // парсер ждёт bytes_left байт, прежде чем снова пытаться декодировать.
    // Проверяем, что ресинхронизация укладывается в размер буфера.
    capture_reset();
    uint8_t payload[4] = {COMM_FW_VERSION, 1, 2, 3};
    uint8_t buf[VESC_PACKET_BUFFER_LEN];
    size_t n = frame(payload, sizeof(payload), buf);

    size_t fed = 0;
    while (g_payloads == 0 && fed < VESC_PACKET_BUFFER_LEN * 2) {
        vesc_packet_process_buffer(&p, buf, n);
        fed += n;
    }
    info("ресинхронизация заняла %zu байт (потолок — размер буфера %d)", fed,
         VESC_PACKET_BUFFER_LEN);
    check(g_payloads > 0 && fed <= VESC_PACKET_BUFFER_LEN + n,
          "парсер ресинхронизировался в пределах одного буфера");
}

// -------------------------------------------------------------- тесты сервера

static int stub_get_xml(void *ctx, int ind, const uint8_t **data) {
    static const uint8_t xml[300] = {0xDE, 0xAD, 0xBE, 0xEF};
    (void) ctx;
    if (ind != 0) {
        return -1;
    }
    *data = xml;
    return (int) sizeof(xml);
}

static uint8_t stub_config[64] = {1, 2, 3, 4};
static bool stub_set_called;
static bool stub_set_accepts = true;

static int stub_get(void *ctx, int ind, uint8_t *buf, size_t cap, bool is_default) {
    (void) ctx;
    (void) is_default;
    if (ind != 0 || cap < sizeof(stub_config)) {
        return -1;
    }
    memcpy(buf, stub_config, sizeof(stub_config));
    return (int) sizeof(stub_config);
}

static bool stub_set(void *ctx, int ind, const uint8_t *buf, size_t len) {
    (void) ctx;
    (void) buf;
    (void) len;
    if (ind != 0) {
        return false;
    }
    stub_set_called = true;
    return stub_set_accepts;
}

static size_t custom_app_rx_len;
static bool custom_app_rx_called;

static void stub_to_firmware(void *ctx, const uint8_t *data, size_t len) {
    (void) ctx;
    (void) data;
    custom_app_rx_called = true;
    custom_app_rx_len = len;
}

static void stub_telemetry(void *ctx, VescValues *out) {
    (void) ctx;
    memset(out, 0, sizeof(*out));
    out->v_in = 75.5f;
    out->rpm = 1234.0f;
    out->duty = 0.42f;
    out->temp_mos = 31.5f;
}

static void server_setup(VescServer *s) {
    vesc_server_init(s, capture_send, NULL);
    s->config.get_xml = stub_get_xml;
    s->config.get = stub_get;
    s->config.set = stub_set;
    s->config.config_count = 1;
    s->custom_app.to_firmware = stub_to_firmware;
    s->telemetry_provider = stub_telemetry;
}

/** Отправить команду серверу и вернуть payload последнего ответа. */
static size_t server_request(
    VescServer *s, const uint8_t *payload, size_t len, uint8_t *reply, size_t reply_cap
) {
    capture_reset();
    uint8_t buf[VESC_PACKET_BUFFER_LEN];
    size_t n = frame(payload, len, buf);
    vesc_server_feed(s, buf, n);

    if (g_tx.len < 3) {
        return 0;
    }

    // Разобрать собранный сервером кадр обратно
    VescPacket parser;
    vesc_packet_init(&parser, NULL, capture_process, NULL);
    size_t saved = g_payloads;
    g_payloads = 0;
    vesc_packet_process_buffer(&parser, g_tx.data, g_tx.len);
    size_t got = g_payloads;
    g_payloads = saved;

    if (got == 0) {
        return 0;
    }
    size_t out_len = g_last_payload_len < reply_cap ? g_last_payload_len : reply_cap;
    memcpy(reply, g_last_payload, out_len);
    return out_len;
}

static void test_server_handshake(void) {
    section("Handshake: COMM_FW_VERSION");

    VescServer s;
    server_setup(&s);

    uint8_t req = COMM_FW_VERSION;
    uint8_t rep[VESC_PACKET_MAX_PL_LEN];
    size_t n = server_request(&s, &req, 1, rep, sizeof(rep));

    check(n > 0, "получен ответ (%zu байт)", n);
    check(rep[0] == COMM_FW_VERSION, "идентификатор ответа = COMM_FW_VERSION");
    check(rep[1] == 6 && rep[2] == 6, "версия %u.%02u — известна VESC Tool", rep[1], rep[2]);

    const char *hw = (const char *) (rep + 3);
    check(strcmp(hw, "FloatCore") == 0, "hw_name = \"%s\"", hw);

    size_t ind = 3 + strlen(hw) + 1;
    ind += FW_INFO_UUID_LEN;  // uuid
    ind += 1;                 // is_paired
    ind += 1;                 // test fw
    check(rep[ind] == VESC_HW_TYPE_CUSTOM_MODULE,
          "hw_type = CUSTOM_MODULE (%u): FloatCore не выдаёт себя за VESC", rep[ind]);
    ind += 1;
    check(rep[ind] == 1, "custom_config_num = %u", rep[ind]);
    ind += 1;
    ind += 1;  // phase filters
    check(rep[ind] == 0, "qml_hw = %u", rep[ind]);
    ind += 1;
    check(rep[ind] == 1, "qml_app = %u — VESC Tool запросит UI Refloat", rep[ind]);
    ind += 1;
    ind += 1;  // nrf flags
    check(strcmp((const char *) (rep + ind), "FloatCore") == 0, "fw_name = \"%s\"", rep + ind);
}

static void test_server_telemetry(void) {
    section("Телеметрия: COMM_GET_VALUES");

    VescServer s;
    server_setup(&s);

    uint8_t req = COMM_GET_VALUES;
    uint8_t rep[VESC_PACKET_MAX_PL_LEN];
    size_t n = server_request(&s, &req, 1, rep, sizeof(rep));

    check(n > 0 && rep[0] == COMM_GET_VALUES, "получен ответ COMM_GET_VALUES (%zu байт)", n);

    size_t ind = 1;
    int16_t temp_mos = vb_get_int16(rep, &ind);
    check(temp_mos == 315, "temp_mos закодирован как %d (31.5 °C × 10)", temp_mos);

    ind = 1 + 2 + 2 + 4 + 4 + 4 + 4;  // до duty
    int16_t duty = vb_get_int16(rep, &ind);
    check(duty == 420, "duty закодирован как %d (0.42 × 1000)", duty);

    int32_t rpm = vb_get_int32(rep, &ind);
    check(rpm == 1234, "rpm = %d", rpm);

    int16_t v_in = vb_get_int16(rep, &ind);
    check(v_in == 755, "v_in = %d (75.5 В × 10)", v_in);

    // Селективный вариант
    uint8_t sel[5];
    size_t si = 0;
    vb_append_uint8(sel, COMM_GET_VALUES_SELECTIVE, &si);
    vb_append_uint32(sel, VALUES_V_IN | VALUES_RPM, &si);
    n = server_request(&s, sel, si, rep, sizeof(rep));
    check(n == 1 + 4 + 4 + 2, "селективный ответ содержит только запрошенные поля (%zu байт)", n);
}

static void test_server_config(void) {
    section("Конфигурация: XML, чтение, запись");

    VescServer s;
    server_setup(&s);

    uint8_t rep[VESC_PACKET_MAX_PL_LEN];

    // XML чанк
    uint8_t req[16];
    size_t ri = 0;
    vb_append_uint8(req, COMM_GET_CUSTOM_CONFIG_XML, &ri);
    vb_append_int8(req, 0, &ri);
    vb_append_int32(req, 100, &ri);
    vb_append_int32(req, 0, &ri);
    size_t n = server_request(&s, req, ri, rep, sizeof(rep));

    size_t ind = 1;
    int8_t conf_ind = vb_get_int8(rep, &ind);
    int32_t total = vb_get_int32(rep, &ind);
    int32_t offset = vb_get_int32(rep, &ind);
    check(rep[0] == COMM_GET_CUSTOM_CONFIG_XML && conf_ind == 0, "ответ XML для конфигурации 0");
    check(total == 300 && offset == 0, "заявлен полный размер %d, смещение %d", total, offset);
    check(n == 10 + 100, "отдан запрошенный чанк 100 байт (всего %zu)", n);

    // Чанк за границей данных
    ri = 0;
    vb_append_uint8(req, COMM_GET_CUSTOM_CONFIG_XML, &ri);
    vb_append_int8(req, 0, &ri);
    vb_append_int32(req, 400, &ri);
    vb_append_int32(req, 250, &ri);
    n = server_request(&s, req, ri, rep, sizeof(rep));
    check(n == 10 + 50, "запрос за границей обрезан по данным (%zu байт)", n);

    // Неверный индекс конфигурации
    ri = 0;
    vb_append_uint8(req, COMM_GET_CUSTOM_CONFIG_XML, &ri);
    vb_append_int8(req, 7, &ri);
    vb_append_int32(req, 10, &ri);
    vb_append_int32(req, 0, &ri);
    n = server_request(&s, req, ri, rep, sizeof(rep));
    check(n == 0, "несуществующая конфигурация — ответа нет, падения нет");

    // Чтение конфигурации
    ri = 0;
    vb_append_uint8(req, COMM_GET_CUSTOM_CONFIG, &ri);
    vb_append_int8(req, 0, &ri);
    n = server_request(&s, req, ri, rep, sizeof(rep));
    check(n == 2 + sizeof(stub_config) && rep[0] == COMM_GET_CUSTOM_CONFIG,
          "конфигурация прочитана (%zu байт)", n);

    // Запись конфигурации
    stub_set_called = false;
    uint8_t wreq[80];
    size_t wi = 0;
    vb_append_uint8(wreq, COMM_SET_CUSTOM_CONFIG, &wi);
    vb_append_int8(wreq, 0, &wi);
    memcpy(wreq + wi, stub_config, sizeof(stub_config));
    wi += sizeof(stub_config);
    n = server_request(&s, wreq, wi, rep, sizeof(rep));
    check(stub_set_called, "запись доведена до источника истины (конфигурация Refloat)");
    check(n > 0 && rep[0] == COMM_GET_CUSTOM_CONFIG, "после записи отдана актуальная конфигурация");

    // Негативный: невалидная конфигурация отвергается источником
    stub_set_accepts = false;
    n = server_request(&s, wreq, wi, rep, sizeof(rep));
    check(n > 0, "отказ записи не рвёт соединение — ответ всё равно отправлен");
    stub_set_accepts = true;

    // Негативный: обрезанный SET_CUSTOM_CONFIG
    uint8_t only_cmd = COMM_SET_CUSTOM_CONFIG;
    uint32_t before = s.stats.truncated_payloads;
    server_request(&s, &only_cmd, 1, rep, sizeof(rep));
    check(s.stats.truncated_payloads == before + 1, "обрезанный SET_CUSTOM_CONFIG распознан");
}

static void test_server_qml(void) {
    section("QML: COMM_GET_QML_UI_APP");

    static const uint8_t qml[1000] = {0x11, 0x22};
    VescServer s;
    server_setup(&s);

    // провайдер QML
    s.qml_app_provider = NULL;
    uint8_t req[16];
    uint8_t rep[VESC_PACKET_MAX_PL_LEN];
    size_t ri = 0;
    vb_append_uint8(req, COMM_GET_QML_UI_APP, &ri);
    vb_append_int32(req, 10, &ri);
    vb_append_int32(req, 0, &ri);
    size_t n = server_request(&s, req, ri, rep, sizeof(rep));
    check(n == 0 && s.stats.unsupported_commands >= 1, "без провайдера QML ответа нет");

    struct QmlCtx {
        const uint8_t *data;
        int len;
    };
    // простой провайдер через статические данные
    s.qml_app_provider = NULL;
    (void) qml;

    // используем лямбду-заменитель: отдельная функция ниже
    extern int test_qml_provider(void *ctx, const uint8_t **data);
    s.qml_app_provider = test_qml_provider;

    ri = 0;
    vb_append_uint8(req, COMM_GET_QML_UI_APP, &ri);
    vb_append_int32(req, 400, &ri);
    vb_append_int32(req, 0, &ri);
    n = server_request(&s, req, ri, rep, sizeof(rep));

    size_t ind = 1;
    int32_t total = vb_get_int32(rep, &ind);
    int32_t offset = vb_get_int32(rep, &ind);
    check(rep[0] == COMM_GET_QML_UI_APP, "ответ COMM_GET_QML_UI_APP");
    check(total == 1000 && offset == 0, "полный размер %d, смещение %d", total, offset);
    check(n == 9 + 400, "чанк 400 байт (%zu)", n);

    // Чанк с конца
    ri = 0;
    vb_append_uint8(req, COMM_GET_QML_UI_APP, &ri);
    vb_append_int32(req, 400, &ri);
    vb_append_int32(req, 900, &ri);
    n = server_request(&s, req, ri, rep, sizeof(rep));
    check(n == 9 + 100, "последний чанк обрезан до 100 байт (%zu)", n);

    // Смещение за границей
    ri = 0;
    vb_append_uint8(req, COMM_GET_QML_UI_APP, &ri);
    vb_append_int32(req, 10, &ri);
    vb_append_int32(req, 5000, &ri);
    n = server_request(&s, req, ri, rep, sizeof(rep));
    check(n == 0, "смещение за пределами данных — ответа нет, падения нет");
}

int test_qml_provider(void *ctx, const uint8_t **data) {
    static uint8_t qml[1000] = {0x11, 0x22};
    (void) ctx;
    *data = qml;
    return (int) sizeof(qml);
}

static void test_server_custom_app(void) {
    section("Custom App Data: транспорт до прошивки и обратно");

    VescServer s;
    server_setup(&s);

    uint8_t rep[VESC_PACKET_MAX_PL_LEN];
    uint8_t req[16];
    size_t ri = 0;
    vb_append_uint8(req, COMM_CUSTOM_APP_DATA, &ri);
    vb_append_uint8(req, 101, &ri);  // условная команда Refloat
    vb_append_uint8(req, 1, &ri);

    custom_app_rx_called = false;
    server_request(&s, req, ri, rep, sizeof(rep));
    check(custom_app_rx_called && custom_app_rx_len == 2,
          "данные доведены до прошивки без изменений (%zu байт)", custom_app_rx_len);

    // Пустой custom app data — не должно быть падения
    uint8_t only_cmd = COMM_CUSTOM_APP_DATA;
    custom_app_rx_called = false;
    custom_app_rx_len = 999;
    server_request(&s, &only_cmd, 1, rep, sizeof(rep));
    check(custom_app_rx_called && custom_app_rx_len == 0, "пустой payload передан как 0 байт");

    // Обратное направление
    capture_reset();
    uint8_t from_fw[8] = {101, 5, 5, 5, 5, 5, 5, 5};
    bool ok = vesc_server_send_custom_app_data(&s, from_fw, sizeof(from_fw));
    check(ok && g_tx.frames == 1, "ответ прошивки упакован и отправлен");
    check(g_tx.data[0] == 2 && g_tx.data[2] == COMM_CUSTOM_APP_DATA,
          "кадр начинается со стартового байта и COMM_CUSTOM_APP_DATA");

    // Слишком длинные данные от прошивки
    static uint8_t too_long[VESC_PACKET_MAX_PL_LEN];
    ok = vesc_server_send_custom_app_data(&s, too_long, sizeof(too_long));
    check(!ok, "слишком длинный ответ прошивки отвергнут, а не обрезан молча");
}

static void test_server_safety_and_unknown(void) {
    section("Безопасность и неизвестные команды");

    VescServer s;
    server_setup(&s);
    uint8_t rep[VESC_PACKET_MAX_PL_LEN];

    const uint8_t motor_cmds[] = {COMM_SET_DUTY,      COMM_SET_CURRENT, COMM_SET_CURRENT_BRAKE,
                                  COMM_SET_RPM,       COMM_SET_POS,     COMM_SET_HANDBRAKE,
                                  COMM_SET_CURRENT_REL};
    for (size_t i = 0; i < sizeof(motor_cmds); ++i) {
        uint8_t req[8];
        size_t ri = 0;
        vb_append_uint8(req, motor_cmds[i], &ri);
        vb_append_int32(req, 100000, &ri);  // 1000 А, если бы кто-то это исполнил
        size_t n = server_request(&s, req, ri, rep, sizeof(rep));
        if (n != 0) {
            check(false, "команда мотору %s дала ответ — это недопустимо",
                  vesc_command_name(motor_cmds[i]));
        }
    }
    check(s.stats.motor_commands_blocked == sizeof(motor_cmds),
          "все %zu команд управления мотором заблокированы (%u)", sizeof(motor_cmds),
          s.stats.motor_commands_blocked);

    // Известная, но не реализованная команда
    uint8_t known = COMM_GET_MCCONF;
    uint32_t before = s.stats.unsupported_commands;
    size_t n = server_request(&s, &known, 1, rep, sizeof(rep));
    check(n == 0 && s.stats.unsupported_commands == before + 1,
          "COMM_GET_MCCONF: известна, не реализована, тихо игнорируется");

    // Полностью неизвестный идентификатор
    uint8_t unknown = 251;
    before = s.stats.unknown_commands;
    n = server_request(&s, &unknown, 1, rep, sizeof(rep));
    check(n == 0 && s.stats.unknown_commands == before + 1,
          "неизвестный идентификатор игнорируется без ответа");

    // Обрезанный GET_VALUES_SELECTIVE
    uint8_t sel[3] = {COMM_GET_VALUES_SELECTIVE, 0, 0};
    before = s.stats.truncated_payloads;
    server_request(&s, sel, sizeof(sel), rep, sizeof(rep));
    check(s.stats.truncated_payloads == before + 1, "обрезанная маска распознана");

    // COMM_ALIVE не должен ничего ломать и не требует ответа
    uint8_t alive = COMM_ALIVE;
    n = server_request(&s, &alive, 1, rep, sizeof(rep));
    check(n == 0, "COMM_ALIVE обработан без ответа, как в прошивке VESC");
}

static void test_server_fuzz(void) {
    section("Негативный: случайные кадры на входе сервера");

    VescServer s;
    server_setup(&s);

    uint32_t state = 0xC0FFEEu;
    uint8_t payload[VESC_PACKET_MAX_PL_LEN];
    uint8_t buf[VESC_PACKET_BUFFER_LEN];

    for (int iter = 0; iter < 20000; ++iter) {
        state = state * 1664525u + 1013904223u;
        size_t len = 1 + (state >> 20) % 64;
        for (size_t i = 0; i < len; ++i) {
            state = state * 1664525u + 1013904223u;
            payload[i] = (uint8_t) (state >> 16);
        }
        capture_reset();
        size_t n = frame(payload, len, buf);
        vesc_server_feed(&s, buf, n);
    }

    info("20000 случайных корректно оформленных кадров: неподдержанных %u, неизвестных %u, "
         "обрезанных %u, моторных заблокировано %u",
         s.stats.unsupported_commands, s.stats.unknown_commands, s.stats.truncated_payloads,
         s.stats.motor_commands_blocked);
    check(true, "сервер не упал на случайных командах");
    check(s.stats.rx_frames == 20000, "все кадры обработаны (%u)", s.stats.rx_frames);
}

int main(void) {
    printf("\nFloatCore protocol tests — только compat/vesc_protocol, без платформы\n");
    printf("================================================================\n");

    test_crc();
    test_roundtrip();
    test_bad_crc();
    test_truncated();
    test_oversized();
    test_framing_garbage();
    test_fuzz();
    test_server_handshake();
    test_server_telemetry();
    test_server_config();
    test_server_qml();
    test_server_custom_app();
    test_server_safety_and_unknown();
    test_server_fuzz();

    printf("\n================================================================\n");
    if (failures == 0) {
        printf("\033[32mВсе проверки пройдены\033[0m (%d)\n\n", checks);
        return 0;
    }
    printf("\033[31mПровалено: %d из %d\033[0m\n\n", failures, checks);
    return 1;
}
