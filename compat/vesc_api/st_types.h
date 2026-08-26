// Заглушка st_types.h для host-сборки.
//
// refloat-upstream/src/led_driver.h включает "st_types.h" ради типов TIM_TypeDef и
// DMA_Stream_TypeDef, которые используются только внутри led_driver.c (исключён из
// host-сборки). Достаточно непрозрачных типов совместимого размера.
//
// Тот же приём применим на ESP32: upstream-заголовок остаётся неизменным, а
// led_driver.c заменяется реализацией на RMT.
#pragma once

#include <stdint.h>

typedef struct { volatile uint32_t DIER; volatile uint32_t reserved[31]; } TIM_TypeDef;
typedef struct { volatile uint32_t CR; volatile uint32_t reserved[7]; } DMA_Stream_TypeDef;
