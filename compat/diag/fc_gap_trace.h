// Трассировка длинных зазоров между семплами IMU (ТЗ v0.9A §3).
//
// ЗАЧЕМ. На v0.8B интервал между принятыми семплами достиг 110 мс, и
// объяснить его не удалось: переинициализации не было, дубликатов подряд не
// больше одного, худшая транзакция 13.8 мс, контур не опоздал ни разу.
// Счётчики говорят, что задача чтения ~80 мс не исполнялась, и больше ничего.
// Разбор в docs/imu_motor_interference.md §5.
//
// Чтобы следующий такой зазор был диагностируемым, а не наблюдаемым, рядом с
// ним нужно сохранить состояние всего остального: кто что делал в этот
// момент.
//
// ЧЕГО НЕЛЬЗЯ ДЕЛАТЬ. Печатать отсюда. Событие обнаруживается в задаче
// реального времени, а вывод ста символов на 115200 бод стоит около 9 мс при
// периоде контура 2 мс — трассировка сама создала бы то, что измеряет.
// Поэтому здесь только копирование чисел в кольцо; печать — по запросу
// человека, из консоли.
//
// Поля делятся на две группы. Дешёвые (счётчики в ОЗУ) снимаются НА МЕСТЕ, в
// момент события. Дорогие (запас стека, куча) требуют обхода памяти и
// снимаются позже, фоновой задачей, — для них есть отдельный признак, чтобы
// никто не принял значение «через секунду после» за значение «в момент».
#pragma once

#include <stdbool.h>
#include <stdint.h>

// Восемь событий. Больше не нужно: зазоры такого масштаба редки, а если их
// станет много, интересны будут первые — они объясняют причину, тогда как
// последние объясняют только последствия.
#define FC_GAP_TRACE_SLOTS 8u

// С чего начинаем ловить. Период контура 2000 мкс, поэтому 5 мс — это уже
// два пропущенных семпла подряд, а не дрожание планировщика.
#define FC_GAP_TRACE_DEFAULT_THRESHOLD_US 5000u

// Граница, выше которой зазор считается неприемлемым (ТЗ v0.9A §21).
#define FC_GAP_TRACE_UNACCEPTABLE_US 20000u

typedef struct {
    // --- момент события
    uint64_t timestamp_us;
    uint64_t prev_sample_us;
    uint32_t gap_us;
    uint32_t seq; // порядковый номер события, не семпла

    // --- состояние безопасности
    uint32_t supervisor_state;
    uint32_t supervisor_faults;
    uint32_t supervisor_latched;
    bool operator_armed;
    bool motor_backend_present;
    uint64_t motor_allowed;
    uint64_t motor_sent;
    uint64_t last_motor_command_us;
    uint32_t last_command_seq;
    float last_command_a;

    // --- шина
    uint64_t can_rx_frames;
    uint64_t can_tx_frames;
    uint32_t can_bus_errors;
    uint32_t can_err_tx;
    uint32_t can_err_rx;
    uint32_t can_bus_off;
    uint32_t node_age_us[2];

    // --- тракт семплов
    //
    // Появилось на v0.9A по результатам первого же пойманного зазора: шина
    // была исправна (failed=0), а интервал между ПРИНЯТЫМИ семплами вырос.
    // Разницу между «не прочиталось» и «прочиталось, но не принято» без этих
    // счётчиков увидеть нельзя, а она решающая.
    uint64_t pipe_polls;
    uint64_t pipe_accepted;
    uint64_t pipe_duplicates;
    uint64_t pipe_rejected;
    uint64_t pipe_suspected_skips;

    // --- шина датчика
    uint64_t i2c_reads_ok;
    uint64_t i2c_reads_failed;
    uint32_t i2c_recoveries;
    uint32_t i2c_last_transaction_us;

    // --- планировщик и память
    uint32_t control_missed;
    uint32_t control_late;
    uint32_t control_max_us;
    uint32_t refloat_missed;
    uint32_t refloat_max_us;
    uint32_t aux_missed;
    uint32_t aux_max_us;
    uint32_t log_queue_depth;
    uint32_t log_drops;
    uint32_t reset_reason;
    bool nvs_busy;
    bool nvs_busy_known; // false, если платформа этого не сообщает

    // --- мотор
    float measured_current[2];
    float erpm[2];

    // --- снято позже, фоновой задачей
    bool deferred_filled;
    uint32_t heap_free;
    uint32_t heap_min;
    uint32_t stack_imu;
    uint32_t stack_control;
    uint32_t stack_aux;
} FcGapEvent;

typedef struct {
    uint32_t threshold_us;
    uint32_t events_total;    // сколько превысило порог с начала
    uint32_t events_over_20ms;
    uint32_t dropped;         // кольцо переполнилось
    uint32_t max_gap_us;
    uint64_t max_gap_at_us;
    uint32_t stored;
} FcGapTraceStats;

void fc_gap_trace_init(uint32_t threshold_us);

/** Порог, с которого зазор считается событием. Ноль оставляет текущий. */
void fc_gap_trace_set_threshold(uint32_t threshold_us);
uint32_t fc_gap_trace_threshold(void);

/**
 * Зазор превысил порог? Дешёвая проверка для горячего пути: вызывающий
 * собирает тяжёлый снимок только если здесь true.
 */
bool fc_gap_trace_is_event(uint32_t gap_us);

/**
 * Сохранить снимок. Вызывается из задачи реального времени, поэтому только
 * копирует. При переполнении кольца сохраняются ПЕРВЫЕ события: они
 * объясняют причину, последние объясняют последствия.
 */
void fc_gap_trace_capture(const FcGapEvent *e);

uint32_t fc_gap_trace_count(void);
const FcGapEvent *fc_gap_trace_get(uint32_t index);
FcGapTraceStats fc_gap_trace_stats(void);
void fc_gap_trace_reset(void);

/**
 * Дописать дорогие поля в событие, у которого их ещё нет. Возвращает индекс
 * или -1, если дописывать нечего. Вызывается фоновой задачей.
 */
int fc_gap_trace_next_unfilled(void);
void fc_gap_trace_fill_deferred(uint32_t index, uint32_t heap_free, uint32_t heap_min,
                                uint32_t stack_imu, uint32_t stack_control, uint32_t stack_aux);
