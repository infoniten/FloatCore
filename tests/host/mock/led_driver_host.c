// Host-реализация led_driver.h вместо STM32-версии (TIM+DMA).
//
// Демонстрирует, что единственный непортируемый файл Refloat заменяется без
// изменения upstream-заголовка: led_driver.h остаётся как есть, меняется только .c.
// На ESP32 сюда встанет драйвер на RMT.

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
    return false;  // LED-и в host-сборке отсутствуют
}

void led_driver_paint(LedDriver *driver) {
    (void) driver;
}

void led_driver_destroy(LedDriver *driver) {
    (void) driver;
}
