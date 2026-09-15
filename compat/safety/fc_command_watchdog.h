// Политика сторожевого таймера команд мотору (ТЗ v0.7C §4).
//
// НА ЭТОМ ЭТАПЕ МОТОРА НЕТ. Модуль ничего не отправляет и ни к чему не
// подключён: он описывает и проверяет политику, по которой будущая сборка
// MOTOR_CAPABLE обязана будет работать. Проверять её на host, пока ошибка
// стоит одну неудачную сборку, а не одно падение с доски, — единственный
// разумный момент.
//
// ------------------------------------------------------------------ цепочка
//
// Что происходит, когда FloatCore перестаёт слать команды. Разобрано по
// исходникам bldc release_6_06, файл timeout.c:
//
//   1. Каждый обработчик CAN_PACKET_SET_* и COMM_SET_* зовёт timeout_reset(),
//      то есть запоминает момент последней команды (comm_can.c:1590 и далее).
//      Периодический STATUS таймер НЕ сбрасывает: это широковещательная
//      телеметрия ESC, а не признак того, что контроллер жив.
//   2. Отдельный поток Timeout просыпается КАЖДЫЕ 10 мс (timeout.c, хвост
//      функции: chThdSleepMilliseconds(10)). Отсюда дискретность политики:
//      значения меньше 20 мс на этом железе смысла не имеют.
//   3. Если с последней команды прошло больше timeout_msec, поток вызывает
//      mc_interface_set_brake_current(timeout_brake_current) для ОБЕИХ
//      половин и поднимает has_timeout.
//   4. timeout_msec == 0 отключает проверку ЦЕЛИКОМ (условие
//      `timeout_msec != 0 &&` в timeout.c). Ноль — не «мгновенно», а
//      «никогда», и это ровно та ошибка, которую легко сделать.
//
// Отдельно: независимый аппаратный IWDG на 12 мс следит за потоками FOC и CAN
// самого ESC и перезагружает его микроконтроллер, если они встанут. К потере
// связи с FloatCore он отношения не имеет.
//
// ------------------------------------------------- почему торможение = 0
//
// timeout_brake_current на стенде равен 0.0, и менять это не следует.
// Самобалансирующаяся доска при потере команд должна перейти в свободный
// выбег, а не затормозить: торможение под ногами — это бросок вперёд.
// Значение 0.0 означает «отпустить», и для этого применения оно верное. Но
// оно ОБЯЗАНО быть осознанным, а не унаследованным: отсюда этот абзац.
#pragma once

#include <stdbool.h>
#include <stdint.h>

// Дискретность потока Timeout на стороне ESC, timeout.c.
#define FC_ESC_TIMEOUT_TICK_US 10000u

typedef struct {
    uint32_t command_period_us;     // как часто FloatCore шлёт ток
    uint32_t tolerated_misses;      // сколько команд подряд позволено потерять
    uint32_t esc_timeout_us;        // timeout_msec на VESC
    uint32_t supervisor_timeout_us; // когда FloatCore сам прекращает слать
    uint32_t esc_tick_us;           // дискретность потока Timeout на ESC
} FcCommandWatchdogPolicy;

/**
 * Проверить политику на внутреннюю непротиворечивость.
 *
 * Возвращает false и заполняет why причиной первого нарушения. Проверяются не
 * вкусовые предпочтения, а вещи, каждая из которых уже кого-нибудь роняла.
 */
bool fc_command_watchdog_policy_valid(const FcCommandWatchdogPolicy *p, const char **why);

/** Худшее время удержания последней команды при полном отказе FloatCore. */
uint32_t fc_command_watchdog_worst_case_us(const FcCommandWatchdogPolicy *p);

// ------------------------------------------------ состояние потока команд
//
// Наблюдатель за собственным потоком команд FloatCore. Он не отправляет
// ничего и не разрешает ничего: он отвечает на вопрос «идут ли команды
// вовремя», и в будущей сборке его ответ будет ОДНИМ ИЗ запрещающих входов.
//
// Направление важно: LOST обязан запрещать выход, но возврат в OK НЕ обязан
// его разрешать. Разрешение — исключительно дело Supervisor и Motor Gate.

typedef enum {
    FC_CMD_STREAM_IDLE = 0, // ни одной команды ещё не было
    FC_CMD_STREAM_OK,       // команды идут в срок
    FC_CMD_STREAM_LATE,     // пропуски есть, но в пределах допуска
    FC_CMD_STREAM_LOST,     // допуск исчерпан: ESC вот-вот снимет тягу сам
} FcCommandStreamState;

typedef struct {
    FcCommandWatchdogPolicy policy;
    uint64_t last_command_us;
    bool armed;
    FcCommandStreamState state;
    uint32_t transitions_to_late;
    uint32_t transitions_to_lost;
    uint32_t max_gap_us;
} FcCommandStream;

void fc_command_stream_init(FcCommandStream *s, const FcCommandWatchdogPolicy *p);
void fc_command_stream_sent(FcCommandStream *s, uint64_t now_us);
FcCommandStreamState fc_command_stream_poll(FcCommandStream *s, uint64_t now_us);
const char *fc_command_stream_state_name(FcCommandStreamState st);
