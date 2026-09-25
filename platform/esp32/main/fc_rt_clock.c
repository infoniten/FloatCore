#include "fc_rt_clock.h"

#include "driver/gptimer.h"
#include "esp_attr.h"

#include <string.h>

static struct {
    gptimer_handle_t timer;
    volatile TaskHandle_t sub[FC_RT_CLOCK_SLOTS];
    FcRtClockStats st;
} C;

static bool IRAM_ATTR on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *ev,
                               void *ctx) {
    (void) timer;
    (void) ev;
    (void) ctx;
    BaseType_t woken = pdFALSE;
    ++C.st.alarms;
    for (int i = 0; i < FC_RT_CLOCK_SLOTS; ++i) {
        TaskHandle_t h = C.sub[i];
        if (h) {
            vTaskNotifyGiveFromISR(h, &woken);
        }
    }
    return woken == pdTRUE;
}

bool fc_rt_clock_start(uint32_t period_us) {
    if (C.st.running || period_us < 100) {
        return C.st.running;
    }
    gptimer_config_t cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,  // 1 мкс
    };
    if (gptimer_new_timer(&cfg, &C.timer) != ESP_OK) {
        return false;
    }
    gptimer_event_callbacks_t cbs = {.on_alarm = on_alarm};
    // Обработчик прерывания размещается здесь — на ядре вызывающей задачи.
    if (gptimer_register_event_callbacks(C.timer, &cbs, NULL) != ESP_OK) {
        return false;
    }
    gptimer_alarm_config_t alarm = {
        .alarm_count = period_us,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    if (gptimer_set_alarm_action(C.timer, &alarm) != ESP_OK || gptimer_enable(C.timer) != ESP_OK ||
        gptimer_start(C.timer) != ESP_OK) {
        return false;
    }
    C.st.period_us = period_us;
    C.st.core = (int) xPortGetCoreID();
    C.st.running = true;
    return true;
}

void fc_rt_clock_subscribe(int slot, TaskHandle_t task) {
    if (slot >= 0 && slot < FC_RT_CLOCK_SLOTS) {
        C.sub[slot] = task;
    }
}

bool fc_rt_clock_running(void) {
    return C.st.running;
}

uint32_t fc_rt_clock_period_us(void) {
    return C.st.period_us;
}

bool fc_rt_clock_wait(uint32_t timeout_ms) {
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms)) == 0) {
        ++C.st.wait_timeouts;
        return false;
    }
    return true;
}

FcRtClockStats fc_rt_clock_stats(void) {
    return C.st;
}
