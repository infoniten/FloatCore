// Здоровье узлов VESC на шине CAN (ТЗ v0.7B §12).
//
// ЧЕГО ЭТОТ МОДУЛЬ НЕ ДЕЛАЕТ. Он не управляет ничем. Он не знает про Motor
// Gate, не вызывает его и не может быть вызван из него. Это наблюдатель:
// считает, когда узел в последний раз подавал признаки жизни, и отвечает на
// вопрос «жив ли». Ни одно значение отсюда не открывает выход на мотор.
//
// Направление зависимости выбрано именно так намеренно. Соблазн «если оба
// VESC здоровы — можно ехать» создал бы новый путь к моторной команде, и
// проверять пришлось бы уже не одно место (Motor Gate), а два. Поэтому
// Supervisor вправе читать это состояние как ещё один вход, но отсутствие
// VESC обязано только запрещать, никогда не разрешать.
//
// Модуль платформенно-нейтрален и не зависит от профиля CAN: он полезен и в
// пассивном профиле, где узлы видны только по периодическому STATUS.
#pragma once

#include <stdbool.h>
#include <stdint.h>

// Половин Dual FSESC две; запас на будущее небольшой и осознанный — таблица
// сканируется линейно, и её незачем делать больше наблюдаемой шины.
#define FC_CAN_MAX_NODES 4

// Через сколько молчания узел считается недоступным.
//
// Периодический STATUS идёт 50 Гц (appconf.can_status_rate_1, прочитано в
// v0.7A), то есть каждые 20 мс. Полсекунды — это 25 подряд пропущенных
// кадров: достаточно, чтобы не реагировать на единичную потерю, и достаточно
// быстро, чтобы обрыв шины не остался незамеченным.
#define FC_CAN_NODE_STALE_US 500000u

typedef struct {
    uint8_t id;
    bool known;               // узел объявлен ожидаемым или уже наблюдался
    bool expected;            // объявлен заранее: его отсутствие — проблема
    bool healthy;             // подавал признаки жизни недавно
    bool ever_seen;
    uint64_t last_seen_us;    // любой кадр от узла
    uint64_t last_status_us;  // периодический STATUS
    uint64_t last_diag_us;    // ответ на диагностический запрос
    uint32_t statuses;
    uint32_t diag_responses;
    uint32_t diag_timeouts;
    uint32_t health_drops;    // сколько раз переходил в недоступен
} FcCanNode;

typedef struct {
    FcCanNode nodes[FC_CAN_MAX_NODES];
    uint32_t count;
    uint32_t stale_us;
    uint32_t table_overflow; // узлов на шине больше, чем помещается
} FcCanHealth;

void fc_can_health_init(FcCanHealth *h, uint32_t stale_us);

/** Объявить узел ожидаемым: его отсутствие — не «нет такого», а «пропал». */
void fc_can_health_expect(FcCanHealth *h, uint8_t id);

void fc_can_health_on_status(FcCanHealth *h, uint8_t id, uint64_t now_us);
void fc_can_health_on_diag_response(FcCanHealth *h, uint8_t id, uint64_t now_us);
void fc_can_health_on_diag_timeout(FcCanHealth *h, uint8_t id);

/** Пересчитать доступность по времени. Вызывается периодически. */
void fc_can_health_tick(FcCanHealth *h, uint64_t now_us);

const FcCanNode *fc_can_health_node(const FcCanHealth *h, uint8_t id);

/** Все объявленные ожидаемыми узлы доступны. Ожидаемых нет — false. */
bool fc_can_health_all_expected_healthy(const FcCanHealth *h);
