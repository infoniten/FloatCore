// Привязка ограниченного журнала к ESP32 (ТЗ v0.7C §6).
//
// Правило простое: из realtime-путей печатать нельзя, можно только положить
// запись в кольцо. Realtime-пути здесь — это
//
//   * задача fc_imu_rt (контур 500 Гц) и всё, что она вызывает, включая
//     icm20948_init() при переинициализации после серии отказов чтения;
//   * поток refloat_thd и всё, что зовёт Refloat, в том числе
//     VESC_IF->printf() — код upstream, который мы не правим;
//   * задача приёма CAN.
//
// Для них есть FC_LOGE/W/I. Для кода, который заведомо исполняется вне
// контура (загрузка, консоль, отчёты), обычный ESP_LOG* остаётся уместным:
// подменять его везде значило бы прятать печать там, где она безопасна.
#pragma once

#include "../../../compat/log/fc_log.h"

#include <stdbool.h>
#include <stdint.h>

/** Поднять кольцо и задачу-сливальщик. Вызывается один раз при загрузке. */
void fc_log_port_init(void);

/**
 * Положить запись. Не печатает, не ждёт, не выделяет память.
 * Безопасно из любой задачи; из обработчика прерывания — нельзя.
 */
void fc_log_port_emit(FcLogLevel level, const char *tag, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

#define FC_LOGE(tag, ...) fc_log_port_emit(FC_LOG_ERROR, (tag), __VA_ARGS__)
#define FC_LOGW(tag, ...) fc_log_port_emit(FC_LOG_WARN, (tag), __VA_ARGS__)
#define FC_LOGI(tag, ...) fc_log_port_emit(FC_LOG_INFO, (tag), __VA_ARGS__)

typedef struct {
    uint32_t emitted;
    uint32_t dropped;
    uint32_t truncated;
    uint32_t high_water;
    uint32_t pending;
    uint32_t printed;
    uint32_t max_format_us;      // худшая длительность вызова со стороны производителя
    char max_format_tag[FC_LOG_TAG_MAX];  // чей вызов оказался худшим
} FcLogPortStats;

FcLogPortStats fc_log_port_stats(void);
void fc_log_port_reset_stats(void);

// ------------------------------------------------- фаза загрузки (ТЗ §7)
//
// Загрузочный баннер и инициализация печатают в UART десятки строк, и это
// нормально: контур в этот момент ещё не несёт ответственности ни за что.
// Мешать эту фазу с установившимся режимом в одной статистике нельзя —
// иначе «пропуск дедлайна на старте» навсегда останется в цифрах и будет
// маскировать настоящие пропуски.
//
// Поэтому фаза объявляется явно. Будущий профиль MOTOR_CAPABLE обязан
// запрещать выход на мотор, пока фаза загрузки не закрыта.

/** Объявить, что загрузка и её печать завершены. Вызывается один раз. */
void fc_boot_phase_complete(uint64_t now_us);
bool fc_boot_phase_done(void);
uint64_t fc_boot_phase_end_us(void);
