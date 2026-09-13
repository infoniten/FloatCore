// Диагностические запросы к VESC по CAN — только чтение (ТЗ v0.7B §3, §4).
//
// ЗДЕСЬ НЕТ ФУНКЦИИ «ОТПРАВИТЬ ПРОИЗВОЛЬНЫЙ КАДР». Это главное свойство слоя.
// Наружу выставлено перечисление намерений (FcCanDiagRequest), а не пара
// (идентификатор, данные): вызывающий код физически не может назвать тип
// пакета, которого нет в перечислении. Обобщённого can_send(id, data) не
// существует нигде в сборке — ни публично, ни внутри.
//
// Почему белый список проверяется на уровне COMM, а не CAN. Пакет
// CAN_PACKET_PROCESS_SHORT_BUFFER сам по себе безобиден, но он несёт внутри
// обычный пакет COMM_*, который целевой ESC отдаёт в
// commands_process_packet() (bldc release_6_06, comm/comm_can.c:1749-1788).
// Через него одинаково проходит и запрос версии прошивки, и запись
// конфигурации. Поэтому номер COMM задаётся таблицей внутри этого модуля и
// никогда не приходит от вызывающего.
//
// Доказательство белого списка — на этапе компиляции. Макросы ниже
// раскрываются в константные выражения, и _Static_assert проверяет каждый
// элемент таблицы: тип пакета не моторный, номер COMM входит в список
// доказанно read-only. Это проверяется в каждой единице трансляции, которая
// включает заголовок, а не один раз в тесте.
#pragma once

#include "../safety/fc_build_profile.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Ловушка компиляции: имена, которых нет и не должно появиться. Обращение к
// любому из них даёт «attempt to use a poisoned identifier» вместо невнятной
// ошибки о неявном объявлении. Отравляются во всех профилях, включая
// диагностический: в нём тоже нет ни обобщённой отправки, ни моторных команд.
#pragma GCC poison fc_can_diag_send_raw fc_can_diag_transmit_eid fc_can_diag_send_comm
#pragma GCC poison fc_can_set_current fc_can_set_duty fc_can_set_rpm fc_can_set_pos
#pragma GCC poison fc_can_set_current_brake fc_can_set_handbrake

#if FC_CAN_DIAG_TX_AVAILABLE

// ------------------------------------------------- номера из bldc/datatypes.h

#define FC_CAN_PACKET_PING 17u
#define FC_CAN_PACKET_PONG 18u
#define FC_CAN_PACKET_FILL_RX_BUFFER 5u
#define FC_CAN_PACKET_FILL_RX_BUFFER_LONG 6u
#define FC_CAN_PACKET_PROCESS_RX_BUFFER 7u
#define FC_CAN_PACKET_PROCESS_SHORT_BUFFER 8u

#define FC_COMM_FW_VERSION 0u
#define FC_COMM_GET_VALUES 4u
#define FC_COMM_GET_MCCONF 14u
#define FC_COMM_GET_APPCONF 17u

// Константные предикаты для _Static_assert. Дублируют логику
// fc_vesc_can_is_motor_packet() и fc_vesc_comm_is_read_only(), и это
// намеренно: функцию нельзя вызвать на этапе компиляции, а проверка нужна
// именно там. Тест tests/can сверяет обе формы между собой, поэтому
// разъехаться незаметно они не могут.
#define FC_CAN_TYPE_IS_MOTOR(t)                                                                    \
    ((t) == 0u || (t) == 1u || (t) == 2u || (t) == 3u || (t) == 4u || (t) == 10u || (t) == 11u ||  \
     (t) == 12u || (t) == 13u)

#define FC_COMM_IS_READ_ONLY(c)                                                                    \
    ((c) == 0u || (c) == 4u || (c) == 14u || (c) == 15u || (c) == 17u || (c) == 18u || (c) == 50u)

/**
 * Разрешённые запросы. Расширять этот список можно только вместе с
 * доказательством по исходникам bldc, что ветка обработчика не меняет
 * состояние ESC.
 */
typedef enum {
    FC_CAN_DIAG_PING = 0,   // CAN_PACKET_PING, ответ PONG
    FC_CAN_DIAG_FW_VERSION, // COMM_FW_VERSION
    FC_CAN_DIAG_VALUES,     // COMM_GET_VALUES
    FC_CAN_DIAG_MCCONF,     // COMM_GET_MCCONF
    FC_CAN_DIAG_APPCONF,    // COMM_GET_APPCONF
    FC_CAN_DIAG_REQUEST_COUNT
} FcCanDiagRequest;

// Проверка белого списка на этапе компиляции.
_Static_assert(!FC_CAN_TYPE_IS_MOTOR(FC_CAN_PACKET_PING), "PING не может быть моторным пакетом");
_Static_assert(!FC_CAN_TYPE_IS_MOTOR(FC_CAN_PACKET_PROCESS_SHORT_BUFFER),
               "PROCESS_SHORT_BUFFER не может быть моторным пакетом");
_Static_assert(FC_COMM_IS_READ_ONLY(FC_COMM_FW_VERSION), "COMM_FW_VERSION должен быть read-only");
_Static_assert(FC_COMM_IS_READ_ONLY(FC_COMM_GET_VALUES), "COMM_GET_VALUES должен быть read-only");
_Static_assert(FC_COMM_IS_READ_ONLY(FC_COMM_GET_MCCONF), "COMM_GET_MCCONF должен быть read-only");
_Static_assert(FC_COMM_IS_READ_ONLY(FC_COMM_GET_APPCONF), "COMM_GET_APPCONF должен быть read-only");

/**
 * Идентификатор узла FloatCore на шине.
 *
 * Выбран так, чтобы не совпасть ни с одной половиной Dual FSESC (118 и 100,
 * прочитаны в v0.7A) и не быть широковещательным адресом 255. ESC отвечает
 * именно на этот номер: PONG уходит на data8[0] запроса, а ответ на
 * COMM-запрос — через comm_can_send_buffer(rx_buffer_last_id, …).
 */
#define FC_CAN_SELF_ID 50u
_Static_assert(FC_CAN_SELF_ID != 118u && FC_CAN_SELF_ID != 100u && FC_CAN_SELF_ID != 255u,
               "идентификатор FloatCore обязан отличаться от половин FSESC и от broadcast");

typedef struct {
    uint32_t eid;
    uint8_t data[8];
    uint8_t len;
} FcCanDiagFrame;

typedef enum {
    FC_CAN_DIAG_BUILD_OK = 0,
    FC_CAN_DIAG_BUILD_UNKNOWN_REQUEST, // номера нет в белом списке
    FC_CAN_DIAG_BUILD_BROADCAST,       // адрес 255 запрещён: отвечать будут все
    FC_CAN_DIAG_BUILD_SELF,            // запрос самому себе бессмыслен
} FcCanDiagBuildResult;

/**
 * Собрать кадр запроса. Единственный способ получить кадр для передачи.
 *
 * Цель адресуется поимённо: широковещательный 255 отвергается, потому что на
 * него ответят обе половины сразу, и различить ответы будет нечем.
 */
FcCanDiagBuildResult fc_can_diag_build(FcCanDiagRequest req, uint8_t self_id, uint8_t target_id,
                                       FcCanDiagFrame *out);

const char *fc_can_diag_request_name(FcCanDiagRequest req);
/** Номер COMM внутри запроса или 0xFF, если запрос его не использует (PING). */
uint8_t fc_can_diag_request_comm_id(FcCanDiagRequest req);
/** Тип пакета CAN, которым отправляется запрос. */
uint32_t fc_can_diag_request_packet_type(FcCanDiagRequest req);

// ------------------------------------------------------------ сборка ответа
//
// Короткий ответ (<= 6 байт полезной нагрузки) приходит одним
// PROCESS_SHORT_BUFFER. Длинный — чередой FILL_RX_BUFFER / FILL_RX_BUFFER_LONG
// и завершается PROCESS_RX_BUFFER, где лежат длина и CRC16 всего ответа
// (bldc release_6_06, comm/comm_can.c:443-503). CRC проверяется всегда:
// принять ответ, не сойдясь по контрольной сумме, значит поверить мусору.

#define FC_CAN_DIAG_RX_MAX 512u

typedef struct {
    uint8_t self_id;

    uint8_t asm_buf[FC_CAN_DIAG_RX_MAX]; // сборка длинного ответа
    uint16_t asm_filled;

    uint8_t payload[FC_CAN_DIAG_RX_MAX]; // готовый ответ
    uint16_t payload_len;
    uint8_t from_id;
    bool complete;

    uint32_t pongs;
    uint32_t short_replies;
    uint32_t long_replies;
    uint32_t crc_errors;
    uint32_t overflows;
    uint32_t foreign; // кадры, адресованные не нам
} FcCanDiagRx;

typedef enum {
    FC_CAN_DIAG_RX_IGNORED = 0, // не наш кадр или не относится к ответу
    FC_CAN_DIAG_RX_PARTIAL,     // фрагмент принят, ответ ещё не собран
    FC_CAN_DIAG_RX_COMPLETE,    // ответ собран и проверен
    FC_CAN_DIAG_RX_CRC_ERROR,
    FC_CAN_DIAG_RX_OVERFLOW,
} FcCanDiagRxResult;

void fc_can_diag_rx_init(FcCanDiagRx *rx, uint8_t self_id);
/** Скормить принятый кадр. Возврат говорит, что с ним стало. */
FcCanDiagRxResult fc_can_diag_rx_frame(FcCanDiagRx *rx, uint32_t eid, const uint8_t *data,
                                       uint8_t len);
/** Забыть недособранный ответ — например, по таймауту запроса. */
void fc_can_diag_rx_abort(FcCanDiagRx *rx);

// -------------------------------------------------------- ограничение темпа
//
// v0.7B не превращается в цикл опроса: периодический STATUS остаётся основным
// источником телеметрии, а диагностический запрос — редкое событие
// (ТЗ v0.7B §10). Ограничитель отделён от транспорта, чтобы его можно было
// проверить на host без платы.

#define FC_CAN_DIAG_MIN_INTERVAL_US 200000u // не чаще 5 запросов в секунду

typedef struct {
    uint64_t last_us;
    bool armed; // был ли хоть один запрос
    uint32_t min_interval_us;
    uint32_t allowed;
    uint32_t throttled;
} FcCanDiagRate;

void fc_can_diag_rate_init(FcCanDiagRate *r, uint32_t min_interval_us);
bool fc_can_diag_rate_allow(FcCanDiagRate *r, uint64_t now_us);

#endif // FC_CAN_DIAG_TX_AVAILABLE
