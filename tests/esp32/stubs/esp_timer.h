// Заглушка esp_timer для host-тестов модулей platform/esp32.
// Время задаёт тест — это и позволяет проверить арифметику периодов
// детерминированно, без платы.
#pragma once

#include <stdint.h>

int64_t esp_timer_get_time(void);
void esp32_test_set_time_us(int64_t us);
