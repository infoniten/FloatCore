#include "fc_log_port.h"

#include "fc_platform.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <stdio.h>

static struct {
    FcLog ring;
    SemaphoreHandle_t mtx;
    TaskHandle_t drain;
    uint32_t printed;
    uint32_t max_format_us;
    char max_format_tag[FC_LOG_TAG_MAX];
    bool ready;

    bool boot_done;
    uint64_t boot_end_us;
} L;

// Задача-сливальщик. Живёт на ядре housekeeping и с низким приоритетом:
// печать не должна конкурировать ни с контуром, ни с приёмом CAN.
//
// Пауза 50 мс между проходами выбрана так, чтобы всплеск в полтора десятка
// строк вышел за секунду с небольшим, но при этом задача не будила процессор
// чаще, чем нужно. Задержка вывода на диагностику не влияет: у каждой записи
// есть собственная отметка времени, снятая в момент события, а не печати.
static void drain_task(void *arg) {
    (void) arg;
    for (;;) {
        FcLogEntry e;
        bool got;
        // Печать в UART на 115200 бод стоит около 9 мс на сто символов, и это
        // работа на ядре housekeeping. Раньше она не учитывалась нигде
        // (ТЗ v0.9H §8): её стоимость молча попадала в чужие измерения.
        fc_timing_exec_begin(FC_TIMING_LOG);
        do {
            xSemaphoreTake(L.mtx, portMAX_DELAY);
            got = fc_log_pop(&L.ring, &e);
            xSemaphoreGive(L.mtx);
            if (got) {
                printf("%s (%llu) %s: %s\n", fc_log_level_name((FcLogLevel) e.level),
                       (unsigned long long) (e.t_us / 1000ull), e.tag, e.msg);
                ++L.printed;
            }
        } while (got);
        fc_timing_exec_end(FC_TIMING_LOG);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void fc_log_port_init(void) {
    fc_log_init(&L.ring);
    L.mtx = xSemaphoreCreateMutex();
    if (!L.mtx) {
        return;
    }
    L.ready = true;
    xTaskCreatePinnedToCore(drain_task, "fc_log", 3072, NULL, FC_PRIO_LOG_DRAIN, &L.drain,
                            FC_CORE_HOUSEKEEPING);
}

void fc_log_port_emit(FcLogLevel level, const char *tag, const char *fmt, ...) {
    if (!L.ready) {
        // До поднятия кольца печатаем напрямую: это фаза загрузки, контура
        // ещё нет, и потерять сообщение об отказе инициализации хуже, чем
        // задержать её на миллисекунду.
        va_list ap;
        va_start(ap, fmt);
        printf("%s %s: ", fc_log_level_name(level), tag ? tag : "");
        vprintf(fmt, ap);
        printf("\n");
        va_end(ap);
        return;
    }

    uint64_t t0 = fc_uptime_us();
    va_list ap;
    va_start(ap, fmt);
    // Критическая секция короткая и ограниченная: форматирование в слот
    // известного размера. Ни одного обращения к устройству внутри неё нет.
    if (xSemaphoreTake(L.mtx, 0) == pdTRUE) {
        fc_log_push(&L.ring, level, tag, t0, fmt, ap);
        xSemaphoreGive(L.mtx);
    } else {
        // Мьютекс занят другим производителем. Ждать в контуре нельзя, а
        // тихо терять запись нельзя тем более — считаем как отброшенную.
        ++L.ring.dropped;
    }
    va_end(ap);

    uint32_t dt = (uint32_t) (fc_uptime_us() - t0);
    if (dt > L.max_format_us) {
        L.max_format_us = dt;
        snprintf(L.max_format_tag, sizeof(L.max_format_tag), "%s", tag ? tag : "?");
    }
}

FcLogPortStats fc_log_port_stats(void) {
    FcLogPortStats s = {0};
    s.emitted = L.ring.emitted;
    s.dropped = L.ring.dropped;
    s.truncated = L.ring.truncated;
    s.high_water = L.ring.high_water;
    s.pending = fc_log_pending(&L.ring);
    s.printed = L.printed;
    s.max_format_us = L.max_format_us;
    snprintf(s.max_format_tag, sizeof(s.max_format_tag), "%s", L.max_format_tag);
    return s;
}

void fc_log_port_reset_stats(void) {
    L.ring.emitted = 0;
    L.ring.dropped = 0;
    L.ring.truncated = 0;
    L.ring.high_water = 0;
    L.printed = 0;
    L.max_format_us = 0;
    L.max_format_tag[0] = '\0';
}

void fc_boot_phase_complete(uint64_t now_us) {
    if (!L.boot_done) {
        L.boot_done = true;
        L.boot_end_us = now_us;
    }
}

bool fc_boot_phase_done(void) {
    return L.boot_done;
}

uint64_t fc_boot_phase_end_us(void) {
    return L.boot_end_us;
}
