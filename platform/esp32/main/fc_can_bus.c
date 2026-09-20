#include "fc_can_bus.h"

#if FC_CAN_RX_AVAILABLE

#include "fc_platform.h"

#include "../../../compat/can/fc_vesc_can.h"

#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <string.h>

static const char *TAG = "can_rx";

// Разводка задана монтажом (ТЗ v0.7A): ESP32 GPIO26 -> CTX трансивера,
// GPIO27 <- CRX. Передатчик в listen-only отключён аппаратно, но пин TX всё
// равно назначается драйвером — он остаётся в рецессивном состоянии.
#define FC_CAN_TX_GPIO 26
#define FC_CAN_RX_GPIO 27

// Скорость взята не по умолчанию, а из прочитанной конфигурации ESC:
// appconf.can_baud_rate = 2 = CAN_BAUD_500K на обеих половинах Dual FSESC
// (docs/can_bringup.md). Несовпадение скорости дало бы шторм ошибок, а не
// тишину, поэтому значение обязано быть именно измеренным.
#define FC_CAN_TIMING TWAI_TIMING_CONFIG_500KBITS()

// Режим — константа, выведенная из профиля сборки. Единственное место, где
// он задаётся, и здесь же проверка: подменить режим, не сменив профиль,
// нельзя — не соберётся.
#if FC_CAN_DIAG_TX_AVAILABLE
#define FC_CAN_MODE TWAI_MODE_NORMAL
_Static_assert(FC_CAN_MODE == TWAI_MODE_NORMAL,
               "диагностический профиль требует normal mode");
// Очередь передачи на несколько кадров: запросы одиночные и редкие, очередь
// нужна лишь чтобы twai_transmit не блокировался на арбитраже.
#define FC_CAN_TX_QUEUE 4
#else
#define FC_CAN_MODE TWAI_MODE_LISTEN_ONLY
_Static_assert(FC_CAN_MODE == TWAI_MODE_LISTEN_ONLY,
               "пассивный профиль допускает только listen-only");
#define FC_CAN_TX_QUEUE 0
#endif

// Очередь приёма. При 50 Гц статуса от двух половин это 100 кадров/с;
// 64 кадра — больше секунды запаса, даже если задача приёма задержится.
#define FC_CAN_RX_QUEUE 64

static struct {
    TaskHandle_t task;
    volatile bool run;
    bool installed;
    FcCanStats st;
    FcCanFrame ring[FC_CAN_RING];
    uint32_t ring_head;
    uint32_t ring_count;
    FcCanHealth health;
#if FC_CAN_DIAG_TX_AVAILABLE
    FcCanDiagRx rx;
    FcCanDiagRate rate;
    FcCanDiagStats dst;
    SemaphoreHandle_t done;
    volatile bool awaiting;
#endif
#if FC_CAN_TX_AVAILABLE
    FcCanMotorStats mst;
#endif
} C;

const char *fc_can_bus_mode_name(void) {
#if FC_CAN_DIAG_TX_AVAILABLE
    return "NORMAL";
#else
    return "LISTEN_ONLY";
#endif
}

FcCanHealth fc_can_bus_health(void) {
    fc_can_health_tick(&C.health, fc_uptime_us());
    return C.health;
}

const char *fc_can_bus_state_name(uint32_t state) {
    switch (state) {
    case TWAI_STATE_STOPPED:
        return "STOPPED";
    case TWAI_STATE_RUNNING:
        return "RUNNING";
    case TWAI_STATE_BUS_OFF:
        return "BUS_OFF";
    case TWAI_STATE_RECOVERING:
        return "RECOVERING";
    default:
        return "?";
    }
}

static void account(const twai_message_t *m, uint64_t now) {
    ++C.st.frames_total;
    if (m->extd) {
        ++C.st.frames_ext;
    } else {
        ++C.st.frames_std;
    }
    if (m->rtr) {
        ++C.st.frames_rtr;
    }
    uint8_t dlc = m->data_length_code <= 8 ? m->data_length_code : 8;
    ++C.st.dlc_hist[dlc];
    C.st.last_frame_us = now;

    // Таблица по идентификаторам. Линейный поиск: идентификаторов единицы, а
    // хеш здесь усложнил бы код без выигрыша.
    for (uint32_t i = 0; i < C.st.id_count; ++i) {
        if (C.st.ids[i].id == m->identifier && C.st.ids[i].extended == (bool) m->extd) {
            ++C.st.ids[i].count;
            C.st.ids[i].last_dlc = dlc;
            C.st.ids[i].last_us = now;
            memcpy(C.st.ids[i].last_data, m->data, dlc);
            goto ring;
        }
    }
    if (C.st.id_count < FC_CAN_MAX_IDS) {
        FcCanIdStat *s = &C.st.ids[C.st.id_count++];
        s->id = m->identifier;
        s->extended = m->extd;
        s->count = 1;
        s->last_dlc = dlc;
        s->last_us = now;
        memcpy(s->last_data, m->data, dlc);
    } else {
        ++C.st.ids_overflow;
    }

ring:
    C.ring[C.ring_head].id = m->identifier;
    C.ring[C.ring_head].extended = m->extd;
    C.ring[C.ring_head].rtr = m->rtr;
    C.ring[C.ring_head].dlc = dlc;
    memcpy(C.ring[C.ring_head].data, m->data, dlc);
    C.ring[C.ring_head].t_us = now;
    C.ring_head = (C.ring_head + 1) % FC_CAN_RING;
    if (C.ring_count < FC_CAN_RING) {
        ++C.ring_count;
    }

    // Здоровье узлов. Признаком жизни считается периодический STATUS: у него
    // в младшем байте идентификатора стоит номер отправителя, и приходит он
    // независимо от того, спрашивал ли его кто-нибудь.
    FcVescCanId v = fc_vesc_can_decode(m->identifier, m->extd);
    if (v.vesc_format && v.is_status) {
        fc_can_health_on_status(&C.health, v.controller_id, now);
    }

#if FC_CAN_DIAG_TX_AVAILABLE
    if (C.awaiting) {
        FcCanDiagRxResult r = fc_can_diag_rx_frame(&C.rx, m->identifier, m->data, dlc);
        if (r == FC_CAN_DIAG_RX_COMPLETE) {
            C.awaiting = false;
            xSemaphoreGive(C.done);
        } else if (r == FC_CAN_DIAG_RX_CRC_ERROR) {
            ++C.dst.crc_errors;
        }
    }
#endif
}

static void refresh_status(void) {
    twai_status_info_t s;
    if (twai_get_status_info(&s) != ESP_OK) {
        return;
    }
    C.st.bus_state = s.state;
    C.st.msgs_to_rx = s.msgs_to_rx;
    C.st.rx_missed = s.rx_missed_count;
    C.st.rx_overrun = s.rx_overrun_count;
    C.st.bus_error_count = s.bus_error_count;
    C.st.arb_lost_count = s.arb_lost_count;
    C.st.tx_failed_count = s.tx_failed_count;
    C.st.tx_error_counter = s.tx_error_counter;
    C.st.rx_error_counter = s.rx_error_counter;

    // Политика BUS_OFF. Контроллер уходит туда сам, когда счётчик ошибок
    // передачи переваливает 255, и сам оттуда не возвращается. Молча остаться
    // выключенным хуже, чем попробовать восстановиться: приём — единственный
    // источник телеметрии, и его потеря должна быть заметна и обратима.
    //
    // Восстановление запускается один раз на переход и считается отдельно от
    // самих переходов: если счётчики начнут расти, это будет видно, а не
    // спрячется за автоматическим перезапуском.
    static uint32_t prev_state = TWAI_STATE_STOPPED;
    if (s.state == TWAI_STATE_BUS_OFF && prev_state != TWAI_STATE_BUS_OFF) {
        ++C.st.bus_off_count;
        if (twai_initiate_recovery() == ESP_OK) {
            ++C.st.recoveries;
        }
    }
    prev_state = s.state;
}

static void can_rx_task(void *arg) {
    (void) arg;
    // Задача на ядре housekeeping и с низким приоритетом: контур Refloat живёт
    // на другом ядре, и приём CAN не должен на него влиять вовсе. Разбора
    // внутри прерывания нет — обработка идёт здесь, после twai_receive().
    uint32_t since_status = 0;
    while (C.run) {
        twai_message_t m;
        esp_err_t err = twai_receive(&m, pdMS_TO_TICKS(200));
        uint64_t now = fc_uptime_us();
        if (err == ESP_OK) {
            account(&m, now);
        } else if (err != ESP_ERR_TIMEOUT) {
            ++C.st.receive_errors;
        }
        // Состояние контроллера опрашивается не на каждом кадре: при 100
        // кадрах/с это была бы лишняя работа, а меняется оно редко.
        if (++since_status >= 50) {
            since_status = 0;
            refresh_status();
            fc_can_health_tick(&C.health, now);
        }
    }
    refresh_status();
    C.task = NULL;
    vTaskDelete(NULL);
}

bool fc_can_bus_start(void) {
    if (C.installed) {
        return true;
    }
#if FC_CAN_DIAG_TX_AVAILABLE
    // Семафор переживает переустановку драйвера: пересоздавать его на каждом
    // can-start значило бы течь по хендлам.
    SemaphoreHandle_t keep = C.done;
#endif
    memset(&C, 0, sizeof(C));
#if FC_CAN_DIAG_TX_AVAILABLE
    C.done = keep;
#endif

    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(FC_CAN_TX_GPIO, FC_CAN_RX_GPIO,
                                                          FC_CAN_MODE);
    g.rx_queue_len = FC_CAN_RX_QUEUE;
    g.tx_queue_len = FC_CAN_TX_QUEUE;
    twai_timing_config_t t = FC_CAN_TIMING;
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g, &t, &f);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_driver_install: %s", esp_err_to_name(err));
        return false;
    }
    err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_start: %s", esp_err_to_name(err));
        twai_driver_uninstall();
        return false;
    }
    C.installed = true;
    C.run = true;
    C.st.started_us = fc_uptime_us();

    // Обе половины Dual FSESC объявлены ожидаемыми: их молчание — это
    // «пропал узел», а не «такого узла не бывало». Номера прочитаны из
    // конфигурации ESC на v0.7A, а не угаданы.
    fc_can_health_init(&C.health, FC_CAN_NODE_STALE_US);
    fc_can_health_expect(&C.health, 118);
    fc_can_health_expect(&C.health, 100);

#if FC_CAN_DIAG_TX_AVAILABLE
    fc_can_diag_rx_init(&C.rx, FC_CAN_SELF_ID);
    fc_can_diag_rate_init(&C.rate, FC_CAN_DIAG_MIN_INTERVAL_US);
    if (!C.done) {
        C.done = xSemaphoreCreateBinary();
    }
#endif
    refresh_status();

    ESP_LOGI(TAG, "TWAI %s на GPIO%d/GPIO%d, 500 кбит/с, очередь приёма %d, передачи %d",
             fc_can_bus_mode_name(), FC_CAN_TX_GPIO, FC_CAN_RX_GPIO, FC_CAN_RX_QUEUE,
             FC_CAN_TX_QUEUE);
    xTaskCreatePinnedToCore(can_rx_task, "fc_can_rx", 3584, NULL, FC_PRIO_CAN_RX, &C.task,
                            FC_CORE_HOUSEKEEPING);
    return true;
}

void fc_can_bus_stop(void) {
    C.run = false;
    for (int i = 0; i < 100 && C.task; ++i) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (C.installed) {
        twai_stop();
        twai_driver_uninstall();
        C.installed = false;
    }
}

bool fc_can_bus_running(void) {
    return C.installed;
}

FcCanStats fc_can_bus_stats(void) {
    refresh_status();
    return C.st;
}

uint32_t fc_can_bus_ring(FcCanFrame *out, uint32_t max) {
    uint32_t n = C.ring_count < max ? C.ring_count : max;
    uint32_t start = (C.ring_head + FC_CAN_RING - n) % FC_CAN_RING;
    for (uint32_t i = 0; i < n; ++i) {
        out[i] = C.ring[(start + i) % FC_CAN_RING];
    }
    return n;
}

void fc_can_bus_reset_stats(void) {
    uint64_t now = fc_uptime_us();
    memset(&C.st, 0, sizeof(C.st));
    memset(C.ring, 0, sizeof(C.ring));
    C.ring_head = 0;
    C.ring_count = 0;
    C.st.started_us = now;
    refresh_status();
}

uint32_t fc_can_bus_stack_watermark(void) {
    return C.task ? (uint32_t) uxTaskGetStackHighWaterMark(C.task) : 0;
}


#if FC_CAN_DIAG_TX_AVAILABLE

// Единственная функция, передающая ДИАГНОСТИЧЕСКИЙ кадр.
//
// static — то есть её нет в таблице символов как внешней, и ни один другой
// модуль не может её вызвать даже по ошибке. Аргумент — уже собранный
// белым списком кадр, а не пара (идентификатор, байты): подставить сюда
// произвольный пакет неоткуда, потому что построить его нечем.
//
// В профиле с транспортом мотору рядом появляется вторая такая функция —
// transmit_motor() ниже. Их ровно две, обе static, и никакой третьей в
// прошивке нет: это проверяется аудитом символов (ТЗ v0.9A §5, §22).
static bool transmit_whitelisted(const FcCanDiagFrame *f, uint32_t timeout_ms) {
    twai_message_t m;
    memset(&m, 0, sizeof(m));
    m.identifier = f->eid;
    m.extd = 1;  // протокол VESC пользуется только расширенными кадрами
    m.data_length_code = f->len;
    memcpy(m.data, f->data, f->len);
    return twai_transmit(&m, pdMS_TO_TICKS(timeout_ms)) == ESP_OK;
}

bool fc_can_bus_diag_request(FcCanDiagRequest req, uint8_t target_id, uint32_t timeout_ms,
                             uint8_t *payload, uint16_t payload_max, uint16_t *len_out) {
    if (len_out) {
        *len_out = 0;
    }
    if (!C.installed || !C.done) {
        return false;
    }

    uint64_t now = fc_uptime_us();
    if (!fc_can_diag_rate_allow(&C.rate, now)) {
        ++C.dst.throttled;
        return false;
    }

    FcCanDiagFrame f;
    if (fc_can_diag_build(req, FC_CAN_SELF_ID, target_id, &f) != FC_CAN_DIAG_BUILD_OK) {
        ++C.dst.build_rejected;
        return false;
    }

    // Сборщик ответа готовится ДО передачи: ответ может прийти через сотни
    // микросекунд, и опоздать с подготовкой значит потерять первый кадр.
    fc_can_diag_rx_init(&C.rx, FC_CAN_SELF_ID);
    xSemaphoreTake(C.done, 0);  // сбросить возможный «хвост» прошлого ответа
    C.awaiting = true;

    uint64_t t0 = fc_uptime_us();
    if (!transmit_whitelisted(&f, 50)) {
        C.awaiting = false;
        ++C.dst.tx_failures;
        return false;
    }
    ++C.dst.requests;

    if (xSemaphoreTake(C.done, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        C.awaiting = false;
        fc_can_diag_rx_abort(&C.rx);
        ++C.dst.timeouts;
        fc_can_health_on_diag_timeout(&C.health, target_id);
        return false;
    }

    uint32_t rtt = (uint32_t) (fc_uptime_us() - t0);
    C.dst.last_rtt_us = rtt;
    if (rtt > C.dst.max_rtt_us) {
        C.dst.max_rtt_us = rtt;
    }
    ++C.dst.responses;
    fc_can_health_on_diag_response(&C.health, C.rx.from_id, fc_uptime_us());

    uint16_t n = C.rx.payload_len;
    if (payload && payload_max) {
        if (n > payload_max) {
            n = payload_max;
        }
        memcpy(payload, C.rx.payload, n);
    }
    if (len_out) {
        *len_out = C.rx.payload_len;
    }
    return true;
}

FcCanDiagStats fc_can_bus_diag_stats(void) {
    return C.dst;
}

void fc_can_bus_diag_reset_stats(void) {
    memset(&C.dst, 0, sizeof(C.dst));
}

#endif  // FC_CAN_DIAG_TX_AVAILABLE

#endif  // FC_CAN_RX_AVAILABLE


#if FC_CAN_TX_AVAILABLE

// ------------------------------------------- транспорт команд мотору (v0.9A)
//
// ВТОРАЯ И ПОСЛЕДНЯЯ функция в прошивке, вызывающая twai_transmit().
//
// Она не принимает решений о безопасности и не умеет собрать кадр сама:
// сериализация живёт в compat/can/fc_vesc_can_motor.c, право на команду —
// в Motor Gate и координаторе. Здесь только передача уже готового кадра.
//
// Почему отдельная функция, а не расширение transmit_whitelisted. Потому что
// тогда исчезла бы граница, по которой сегодня видно, что именно способна
// послать сборка: одна функция с флагом «это моторный кадр» означает, что
// любая ошибка в вызывающем коде превращается в моторную команду.
static bool transmit_motor(const FcMotorFrame *f, uint32_t timeout_ms) {
    twai_message_t m;
    memset(&m, 0, sizeof(m));
    m.identifier = f->eid;
    m.extd = 1;
    m.data_length_code = f->len;
    memcpy(m.data, f->data, f->len);
    return twai_transmit(&m, pdMS_TO_TICKS(timeout_ms)) == ESP_OK;
}

bool fc_can_bus_motor_send_current(uint8_t target_id, float amps) {
    if (!C.installed || !C.run) {
        return false;
    }
    FcMotorFrame f;
    if (fc_vesc_can_motor_build_current(target_id, amps, &f) != FC_MOTOR_FRAME_OK) {
        // Сюда негодное значение доходить не должно: его обязаны были
        // отвергнуть выше. Если дошло — это ошибка архитектуры, и она
        // считается отказом передачи, а не тихо округляется.
        ++C.mst.build_rejected;
        return false;
    }
    ++C.mst.attempts;
    // Таймаут передачи короткий намеренно. Команда идёт каждые 2 мс, и
    // застрявшая в очереди на десятки миллисекунд уже неактуальна: лучше
    // считать её неудачной и дать сработать отказному пути, чем доставить
    // устаревшую тягу.
    if (transmit_motor(&f, 2)) {
        ++C.mst.sent;
        return true;
    }
    ++C.mst.failed;
    return false;
}

FcCanMotorStats fc_can_bus_motor_stats(void) {
    return C.mst;
}

void fc_can_bus_motor_reset_stats(void) {
    memset(&C.mst, 0, sizeof C.mst);
}

#endif  // FC_CAN_TX_AVAILABLE
