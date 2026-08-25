// FloatCore Host — запуск неизменённого Refloat на десктопе с мостом к VESC Tool.
//
// Что здесь происходит:
//   * mock-платформа VESC (tests/host/mock) даёт Refloat его 74 функции;
//   * поток симуляции гонит виртуальное время в реальном темпе, 500 Гц;
//   * TCP-сервер говорит на оригинальном протоколе VESC;
//   * LogicalMotor остаётся mock-ом: вывод на моторы физически невозможен.
//
// Заголовки Refloat здесь не подключаются (см. tests/host/harness/refloat_facade.h).

#include "../../compat/motor/logical_motor.h"
#include "../../compat/vesc_protocol/commands.h"
#include "../../tests/host/mock/logical_motor_mock.h"
#include "../../tests/host/mock/mock_vesc_if.h"
#include "../../tests/host/harness/refloat_facade.h"
#include "qml_app.h"
#include "transport.h"

#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define IMU_PERIOD_US 2000  // 500 Гц
#define DEG2RAD (3.14159265358979f / 180.0f)

typedef struct {
    VescServer server;
    Transport *transport;

    // Единственная блокировка, воспроизводящая модель «одно ядро»:
    // поток симуляции держит её на время такта, сетевой поток — на время
    // вызова в Refloat. Параллельного доступа к Data не возникает.
    pthread_mutex_t refloat_lock;

    // Сериализация отправки в сокет (RX-обработчик и Refloat пишут независимо).
    pthread_mutex_t tx_lock;

    volatile bool running;
    bool trace;
} Host;

static Host H;

// --------------------------------------------------------------------- утилиты

static void log_line(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\n");
    fflush(stdout);
}

static void refloat_log_sink(const char *fmt, va_list ap) {
    printf("  [refloat] ");
    vprintf(fmt, ap);
    fflush(stdout);
}

static void trace_sink(void *ctx, const char *line) {
    (void) ctx;
    log_line("  [proto] %s", line);
}

// ------------------------------------------------------- мост протокол → сокет

static void server_send(void *ctx, const uint8_t *data, size_t len) {
    (void) ctx;
    pthread_mutex_lock(&H.tx_lock);
    if (H.transport && H.transport->is_connected(H.transport)) {
        if (!H.transport->send(H.transport, data, len)) {
            H.transport->disconnect(H.transport);
        }
    }
    pthread_mutex_unlock(&H.tx_lock);
}

// ------------------------------------------------------ мост custom app data

/** VESC Tool → Refloat. Вызывается из сетевого потока. */
static void custom_app_to_firmware(void *ctx, const uint8_t *data, size_t len) {
    (void) ctx;
    pthread_mutex_lock(&H.refloat_lock);
    mock_app_data_to_firmware(data, (unsigned int) len);
    pthread_mutex_unlock(&H.refloat_lock);
}

/** Refloat → VESC Tool. Вызывается из потоков Refloat. */
static void app_data_from_firmware(void *ctx, const uint8_t *data, unsigned int len) {
    (void) ctx;
    vesc_server_send_custom_app_data(&H.server, data, len);
}

// ------------------------------------------------------------ мост конфигурации

static int cfg_get_xml(void *ctx, int conf_ind, const uint8_t **data) {
    (void) ctx;
    if (conf_ind != 0) {
        return -1;
    }
    pthread_mutex_lock(&H.refloat_lock);
    int len = mock_custom_config_get_xml(data);
    pthread_mutex_unlock(&H.refloat_lock);
    return len;
}

static int cfg_get(void *ctx, int conf_ind, uint8_t *buf, size_t cap, bool is_default) {
    (void) ctx;
    if (conf_ind != 0) {
        return -1;
    }
    pthread_mutex_lock(&H.refloat_lock);
    int len = mock_custom_config_get(buf, cap, is_default);
    pthread_mutex_unlock(&H.refloat_lock);
    return len;
}

static bool cfg_set(void *ctx, int conf_ind, const uint8_t *buf, size_t len) {
    (void) ctx;
    (void) len;
    if (conf_ind != 0) {
        return false;
    }
    pthread_mutex_lock(&H.refloat_lock);
    bool ok = mock_custom_config_set(buf);
    pthread_mutex_unlock(&H.refloat_lock);
    log_line("  [config] запись конфигурации Refloat: %s", ok ? "принята" : "ОТКЛОНЕНА");
    return ok;
}

// -------------------------------------------------------------------- телеметрия

static void telemetry_provider(void *ctx, VescValues *out) {
    (void) ctx;
    pthread_mutex_lock(&H.refloat_lock);
    // Источник данных — исключительно LogicalMotorTelemetry (ТЗ v0.4 §6).
    // Её наполняет поток симуляции; реального CAN за ней нет.
    LogicalMotorTelemetry lm = logical_motor_telemetry();
    pthread_mutex_unlock(&H.refloat_lock);

    telemetry_from_logical_motor(&lm, 0.0f, 0.0f, 0, out);
}

static int qml_app_provider(void *ctx, const uint8_t **data) {
    (void) ctx;
    *data = qml_app_data;
    return QML_APP_SIZE;
}

// ------------------------------------------------------------- поток симуляции

static void *sim_thread(void *arg) {
    (void) arg;

    struct timespec period = {.tv_sec = 0, .tv_nsec = IMU_PERIOD_US * 1000};

    // Доска стоит ровно, ноги не на футпадах: безопасное состояние.
    float accel[3] = {0.0f, 0.0f, 1.0f};
    float gyro[3] = {0.0f, 0.0f, 0.0f};

    // Правдоподобные показания «двух VESC», которые видит и Refloat, и VESC Tool.
    MockMotorTelemetry tele = {
        .erpm = 0.0f,
        .duty = 0.0f,
        .input_voltage = 75.2f,
        .fet_temp = 31.5f,
        .motor_temp = 26.0f,
    };

    while (H.running) {
        nanosleep(&period, NULL);

        pthread_mutex_lock(&H.refloat_lock);
        mock_imu_set_raw(accel, gyro);
        mock_motor_set_telemetry(&tele);
        mock_advance_us(IMU_PERIOD_US);
        mock_imu_tick((float) IMU_PERIOD_US * 1e-6f);

        // Агрегированная телеметрия логического мотора: на этапе mock-бэкенда
        // она формируется из тех же значений, что видит Refloat.
        RefloatSnapshot snap = refloat_facade_snapshot();
        LogicalMotorTelemetry lm = {
            .rpm = snap.motor_erpm,
            .duty = snap.motor_duty,
            .motor_current = snap.motor_current,
            .input_current = 0.0f,
            .input_voltage = tele.input_voltage,
            .fet_temp = tele.fet_temp,
            .motor_temp = tele.motor_temp,
            .esc_a_alive = true,
            .esc_b_alive = true,
        };
        logical_motor_mock_set_telemetry(&lm);
        pthread_mutex_unlock(&H.refloat_lock);
    }
    return NULL;
}

// -------------------------------------------------------------- сетевой цикл

static void network_loop(void) {
    uint8_t buf[4096];

    while (H.running) {
        log_line("[host] жду подключения VESC Tool…");
        if (!H.transport->accept(H.transport)) {
            if (!H.running) {
                break;
            }
            continue;
        }

        // Состояние парсера обнуляется на каждое соединение, состояние Refloat — нет.
        vesc_packet_reset(&H.server.packet);

        while (H.running) {
            int n = H.transport->recv(H.transport, buf, sizeof(buf));
            if (n < 0) {
                break;
            }
            if (n > 0) {
                vesc_server_feed(&H.server, buf, (size_t) n);
            }
        }

        H.transport->disconnect(H.transport);
        log_line(
            "[host] сессия завершена: RX %u кадров, TX %u, CRC-ошибок %u, "
            "неподдержанных команд %u, заблокированных моторных %u",
            H.server.stats.rx_frames, H.server.stats.tx_frames, H.server.packet.stats.crc_errors,
            H.server.stats.unsupported_commands, H.server.stats.motor_commands_blocked
        );
    }
}

static void on_sigint(int sig) {
    (void) sig;
    H.running = false;
    if (H.transport) {
        H.transport->disconnect(H.transport);
    }
}

// --------------------------------------------------------------------- main

static void usage(const char *argv0) {
    printf(
        "FloatCore Host — Refloat + мост к VESC Tool\n\n"
        "Использование: %s [опции]\n"
        "  --port <n>       TCP-порт (по умолчанию 65102, как в VESC Tool)\n"
        "  --eeprom <файл>  файл постоянного хранения конфигурации\n"
        "                   (по умолчанию build/floatcore_eeprom.bin)\n"
        "  --trace          включить трассировку протокола\n"
        "  --verbose        показывать лог Refloat\n"
        "  --help\n\n"
        "В VESC Tool: Connection → TCP → 127.0.0.1 : <порт> → Connect\n",
        argv0
    );
}

int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IONBF, 0);

    uint16_t port = 65102;
    const char *eeprom_path = "build/floatcore_eeprom.bin";
    bool verbose = false;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            port = (uint16_t) atoi(argv[++i]);
        } else if (strcmp(argv[i], "--eeprom") == 0 && i + 1 < argc) {
            eeprom_path = argv[++i];
        } else if (strcmp(argv[i], "--trace") == 0) {
            H.trace = true;
        } else if (strcmp(argv[i], "--verbose") == 0) {
            verbose = true;
        } else if (strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "Неизвестный аргумент: %s\n", argv[i]);
            usage(argv[0]);
            return 1;
        }
    }

    pthread_mutex_init(&H.refloat_lock, NULL);
    pthread_mutex_init(&H.tx_lock, NULL);

    // --- платформа и Refloat
    mock_init();
    if (verbose) {
        mock_set_log_sink(refloat_log_sink);
    }
    logical_motor_mock_set_clock(mock_now_us);
    logical_motor_init(NULL);

    if (mock_eeprom_load_file(eeprom_path)) {
        log_line("[host] конфигурация загружена из %s", eeprom_path);
    } else {
        log_line("[host] %s отсутствует — Refloat возьмёт значения по умолчанию", eeprom_path);
    }
    mock_eeprom_set_autosave(eeprom_path);

    mock_set_app_data_sink(app_data_from_firmware, NULL);

    if (!refloat_facade_start()) {
        fprintf(stderr, "[host] не удалось запустить Refloat\n");
        return 1;
    }
    log_line("[host] Refloat запущен, конфигурация зарегистрирована: %s",
             mock_has_custom_config() ? "да" : "нет");

    // --- протокольный сервер
    vesc_server_init(&H.server, server_send, NULL);
    H.server.trace = H.trace;
    H.server.trace_sink = trace_sink;
    H.server.telemetry_provider = telemetry_provider;
    H.server.qml_app_provider = qml_app_provider;
    H.server.custom_app.to_firmware = custom_app_to_firmware;
    H.server.config.get_xml = cfg_get_xml;
    H.server.config.get = cfg_get;
    H.server.config.set = cfg_set;
    H.server.config.config_count = 1;

    log_line(
        "[host] идентификация: %s / %s, версия протокола %u.%02u, hw_type=%u, "
        "конфигураций=%u, QML=%u байт",
        H.server.fw.hw_name, H.server.fw.fw_name, H.server.fw.fw_major, H.server.fw.fw_minor,
        H.server.fw.hw_type, H.server.fw.custom_config_num, (unsigned) QML_APP_SIZE
    );

    // --- транспорт
    H.transport = tcp_transport_create(port);
    if (!H.transport) {
        fprintf(stderr, "[host] не удалось открыть порт %u\n", port);
        return 1;
    }
    log_line("[host] TCP-сервер на порту %u. В VESC Tool: Connection → TCP → 127.0.0.1:%u", port,
             port);
    log_line("[host] вывод на моторы физически невозможен: backend — mock, CAN отсутствует");

    H.running = true;
    signal(SIGINT, on_sigint);
    signal(SIGPIPE, SIG_IGN);

    pthread_t sim;
    pthread_create(&sim, NULL, sim_thread, NULL);

    network_loop();

    H.running = false;
    pthread_join(sim, NULL);

    pthread_mutex_lock(&H.refloat_lock);
    refloat_facade_stop();
    mock_deinit();
    pthread_mutex_unlock(&H.refloat_lock);

    H.transport->destroy(H.transport);
    log_line("[host] остановлен");
    return 0;
}
