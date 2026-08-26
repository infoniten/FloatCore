// Заглушка esp_log для host-тестов: логи собираются в буфер, чтобы проверить
// сам факт и частоту записей (rate-limit блокировок мотора).
#pragma once

void esp32_test_log(const char *tag, const char *fmt, ...);
int esp32_test_log_count(void);
void esp32_test_log_reset(void);

#define ESP_LOGW(tag, ...) esp32_test_log(tag, __VA_ARGS__)
#define ESP_LOGI(tag, ...) esp32_test_log(tag, __VA_ARGS__)
#define ESP_LOGE(tag, ...) esp32_test_log(tag, __VA_ARGS__)
