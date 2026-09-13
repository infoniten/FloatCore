#include "fc_can_passive.h"

#if FC_CAN_RX_AVAILABLE

#include "fc_platform.h"

#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
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

// Режим — константа. Единственное место, где он задаётся, и здесь же
// проверка: сборка не соберётся, если кто-то поменяет его на передающий.
#define FC_CAN_MODE TWAI_MODE_LISTEN_ONLY
_Static_assert(FC_CAN_MODE == TWAI_MODE_LISTEN_ONLY,
               "пассивный профиль допускает только listen-only");

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
} C;

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
        }
    }
    refresh_status();
    C.task = NULL;
    vTaskDelete(NULL);
}

bool fc_can_passive_start(void) {
    if (C.installed) {
        return true;
    }
    memset(&C, 0, sizeof(C));

    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(FC_CAN_TX_GPIO, FC_CAN_RX_GPIO,
                                                          FC_CAN_MODE);
    g.rx_queue_len = FC_CAN_RX_QUEUE;
    g.tx_queue_len = 0;  // очередь передачи не нужна и не создаётся
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
    refresh_status();

    ESP_LOGI(TAG, "TWAI listen-only на GPIO%d/GPIO%d, 500 кбит/с, очередь %d",
             FC_CAN_TX_GPIO, FC_CAN_RX_GPIO, FC_CAN_RX_QUEUE);
    xTaskCreatePinnedToCore(can_rx_task, "fc_can_rx", 3584, NULL, FC_PRIO_CAN_RX, &C.task,
                            FC_CORE_HOUSEKEEPING);
    return true;
}

void fc_can_passive_stop(void) {
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

bool fc_can_passive_running(void) {
    return C.installed;
}

FcCanStats fc_can_passive_stats(void) {
    refresh_status();
    return C.st;
}

uint32_t fc_can_passive_ring(FcCanFrame *out, uint32_t max) {
    uint32_t n = C.ring_count < max ? C.ring_count : max;
    uint32_t start = (C.ring_head + FC_CAN_RING - n) % FC_CAN_RING;
    for (uint32_t i = 0; i < n; ++i) {
        out[i] = C.ring[(start + i) % FC_CAN_RING];
    }
    return n;
}

void fc_can_passive_reset_stats(void) {
    uint64_t now = fc_uptime_us();
    memset(&C.st, 0, sizeof(C.st));
    memset(C.ring, 0, sizeof(C.ring));
    C.ring_head = 0;
    C.ring_count = 0;
    C.st.started_us = now;
    refresh_status();
}

uint32_t fc_can_passive_stack_watermark(void) {
    return C.task ? (uint32_t) uxTaskGetStackHighWaterMark(C.task) : 0;
}

#endif  // FC_CAN_RX_AVAILABLE
