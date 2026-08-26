#include "esp_log.h"
#include "esp_timer.h"

#include <stdarg.h>
#include <stdio.h>

static int64_t g_now_us = 0;
static int g_log_count = 0;

int64_t esp_timer_get_time(void) {
    return g_now_us;
}

void esp32_test_set_time_us(int64_t us) {
    g_now_us = us;
}

void esp32_test_log(const char *tag, const char *fmt, ...) {
    (void) tag;
    (void) fmt;
    ++g_log_count;
}

int esp32_test_log_count(void) {
    return g_log_count;
}

void esp32_test_log_reset(void) {
    g_log_count = 0;
}
