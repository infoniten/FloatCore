// Шина CAN на ESP32: приём всегда, передача — только read-only запросы
// (ТЗ v0.7A §5, §7; ТЗ v0.7B §3, §6).
//
// Модуль владеет контроллером TWAI. Режим контроллера — не настройка, а
// свойство профиля сборки:
//
//   FLOATCORE_CAN_PASSIVE       TWAI_MODE_LISTEN_ONLY. Передатчик отключён
//                               аппаратно; ни одной функции отправки в
//                               сборке нет.
//   FLOATCORE_CAN_ACTIVE_DIAG   TWAI_MODE_NORMAL. Передавать можно ровно то,
//                               что умеет собрать compat/can/fc_can_diag.c —
//                               перечисление намерений, а не произвольный
//                               кадр.
//
// ФУНКЦИИ «ОТПРАВИТЬ ПРОИЗВОЛЬНЫЙ КАДР» ЗДЕСЬ НЕТ НИ В ОДНОМ ПРОФИЛЕ. Даже в
// диагностическом единственный вход в передачу принимает FcCanDiagRequest и
// номер получателя; функция, принимающая (идентификатор, байты), объявлена
// static внутри .c и не видна ни одному другому модулю. Негативные тесты
// компиляции проверяют оба профиля.
//
// Три независимые гарантии, что в пассивном профиле контроллер молчит:
//
//   1. Аппаратная. twai_ll_set_mode() выставляет mode_reg.lom — бит Listen
//      Only Mode контроллера, совместимого с SJA1000. В этом режиме
//      передатчик отключён: ни кадров, ни битов подтверждения, ни error
//      frames контроллер не выдаёт.
//      ~/esp/esp-idf/components/hal/esp32/include/hal/twai_ll.h:282-286
//
//   2. Драйверная. twai_transmit() возвращает ESP_ERR_NOT_SUPPORTED сразу,
//      ещё до обращения к железу, если режим LISTEN_ONLY.
//      ~/esp/esp-idf/components/driver/twai/twai.c:718-721
//
//   3. Контракт заголовка ESP-IDF: «The TWAI controller will not influence the
//      bus (No transmissions or acknowledgments) but can receive messages».
//      components/hal/include/hal/twai_types_deprecated.h:122
//
// Режим задаётся одной константой и нигде не меняется: восстановление шины
// (twai_initiate_recovery) режим не трогает, а повторная установка драйвера
// выполняется тем же кодом с той же константой.
#pragma once

#include "../../../compat/safety/fc_build_profile.h"
#include "../../../compat/can/fc_can_diag.h"
#include "../../../compat/can/fc_can_health.h"

#include <stdbool.h>
#include <stdint.h>

#if !FC_CAN_TX_AVAILABLE
// Ловушка компиляции (ТЗ v0.7A §6).
//
// Мало того, что функций передачи не существует: «не существует» даёт при
// вызове невнятную ошибку о неявном объявлении, которую легко принять за
// опечатку. Отравленные идентификаторы дают прямое сообщение
// «attempt to use poisoned identifier», и оно указывает на настоящую причину.
//
// Отравляются только НАШИ имена. twai_transmit сюда не входит намеренно: его
// объявляет driver/twai.h, и отравление сломало бы сам заголовок ESP-IDF.
// Отсутствие twai_transmit в образе доказывается иначе — таблицей символов в
// tools/esp32_smoke.sh.
#pragma GCC poison fc_can_transmit fc_can_send fc_can_send_frame fc_can_write
#endif

#if FC_CAN_RX_AVAILABLE

// Сколько различных идентификаторов держать в таблице. Шина двух половин
// FSESC при 50 Гц статуса даёт единицы идентификаторов; 32 — с запасом на
// неожиданных участников, которых как раз и надо заметить.
#define FC_CAN_MAX_IDS 32
// Кольцо последних кадров для разбора руками. Намеренно небольшое: печать
// каждого кадра в UART сломала бы тайминг, а хвост из 32 штук помогает
// понять структуру трафика.
#define FC_CAN_RING 32

typedef struct {
    uint32_t id;
    bool extended;
    uint64_t count;
    uint32_t last_dlc;
    uint64_t last_us;
    uint8_t last_data[8];
} FcCanIdStat;

typedef struct {
    uint32_t id;
    bool extended;
    bool rtr;
    uint8_t dlc;
    uint8_t data[8];
    uint64_t t_us;
} FcCanFrame;

typedef struct {
    uint64_t frames_total;
    uint64_t frames_std;
    uint64_t frames_ext;
    uint64_t frames_rtr;
    uint64_t dlc_hist[9];
    uint64_t ids_overflow;   // различных идентификаторов больше, чем помещается
    uint32_t id_count;
    FcCanIdStat ids[FC_CAN_MAX_IDS];

    // Состояние контроллера по twai_get_status_info().
    uint32_t bus_state;
    uint32_t msgs_to_rx;
    uint32_t rx_missed;      // кадры, не влезшие в очередь драйвера
    uint32_t rx_overrun;     // переполнение аппаратного буфера
    uint32_t bus_error_count;
    uint32_t arb_lost_count;
    uint32_t tx_failed_count;
    uint32_t tx_error_counter;
    uint32_t rx_error_counter;

    uint32_t bus_off_count;  // сколько раз контроллер уходил в BUS_OFF
    uint32_t recoveries;     // сколько раз запускалось восстановление

    uint64_t started_us;
    uint64_t last_frame_us;
    uint64_t receive_errors;  // twai_receive вернул не ESP_OK и не таймаут
} FcCanStats;

/** Поднять TWAI в listen-only и запустить приём. */
bool fc_can_bus_start(void);
void fc_can_bus_stop(void);
bool fc_can_bus_running(void);

FcCanStats fc_can_bus_stats(void);
/** Последние кадры, новейший последним. Возвращает сколько скопировано. */
uint32_t fc_can_bus_ring(FcCanFrame *out, uint32_t max);
void fc_can_bus_reset_stats(void);
const char *fc_can_bus_state_name(uint32_t state);
uint32_t fc_can_bus_stack_watermark(void);

/** Имя режима контроллера, как он реально установлен драйверу. */
const char *fc_can_bus_mode_name(void);

// --------------------------------------------- здоровье узлов (ТЗ v0.7B §12)
//
// Наблюдение, а не управление: модель узлов заполняется из принимаемых
// кадров и никуда не подключена. Motor Gate её не видит и видеть не должен.
//
// Возвращается снимок, а не указатель: доступность пересчитывается по времени
// в момент запроса. Иначе узел, замолчавший после остановки приёма, так и
// остался бы «доступным» — пересчитывать его было бы некому.
FcCanHealth fc_can_bus_health(void);

#if FC_CAN_DIAG_TX_AVAILABLE

typedef struct {
    uint64_t requests;        // сколько запросов ушло в шину
    uint64_t tx_failures;     // twai_transmit вернул не ESP_OK
    uint64_t throttled;       // отклонено ограничителем темпа
    uint64_t responses;       // собранных ответов
    uint64_t timeouts;        // ответ не пришёл за отведённое время
    uint64_t crc_errors;
    uint64_t build_rejected;  // белый список отверг запрос
    uint32_t last_rtt_us;
    uint32_t max_rtt_us;
} FcCanDiagStats;

/**
 * Отправить один диагностический запрос и дождаться ответа.
 *
 * Единственная точка передачи во всей прошивке. Принимает намерение, а не
 * кадр: собрать произвольный пакет через неё нельзя.
 *
 * Блокирующая и вызывается только из консоли или housekeeping — в
 * realtime-путь она не заходит.
 *
 * payload/len_out — тело ответа без обёртки CAN; для PING это
 * [номер ответившего][тип железа].
 */
bool fc_can_bus_diag_request(FcCanDiagRequest req, uint8_t target_id, uint32_t timeout_ms,
                             uint8_t *payload, uint16_t payload_max, uint16_t *len_out);

FcCanDiagStats fc_can_bus_diag_stats(void);
void fc_can_bus_diag_reset_stats(void);

#endif  // FC_CAN_DIAG_TX_AVAILABLE

#endif  // FC_CAN_RX_AVAILABLE
