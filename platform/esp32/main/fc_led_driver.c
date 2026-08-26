// Реализация led_driver.h для ESP32, этап v0.5.
//
// Единственный непортируемый файл Refloat — src/led_driver.c (регистры TIM+DMA
// STM32). Заголовок upstream остаётся неизменным, подменяется только .c —
// ровно как в host-сборке (tests/host/mock/led_driver_host.c).
//
// Лента к плате не подключена, драйвер на RMT — отдельный этап. Возврат false
// из setup() Refloat трактует как «LED-ов нет» и дальше их не трогает.

#include "led_driver.h"

#include <stddef.h>

void led_driver_init(LedDriver *driver) {
    driver->bitbuffer = NULL;
    driver->bitbuffer_length = 0;
}

bool led_driver_setup(
    LedDriver *driver, LedPin pin, LedPinConfig pin_config, const LedStrip **led_strips
) {
    (void) pin_config;
    (void) led_strips;
    driver->pin = pin;
    return false;  // лента не подключена (ТЗ v0.5)
}

void led_driver_paint(LedDriver *driver) {
    (void) driver;
}

void led_driver_destroy(LedDriver *driver) {
    (void) driver;
}
