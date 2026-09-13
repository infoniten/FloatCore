// Пассивный приём CAN (ТЗ v0.7A §5, §7).
//
// ЗДЕСЬ НЕТ И НЕ ДОЛЖНО БЫТЬ ФУНКЦИИ ПЕРЕДАЧИ. Это не оговорка в комментарии,
// а свойство сборки: в профиле FLOATCORE_CAN_PASSIVE никакой функции отправки
// кадра не объявлено ни здесь, ни где-либо ещё, поэтому вызвать её нельзя —
// код просто не соберётся. Негативный тест компиляции это проверяет
// (tests/can/negative_tx.c).
//
// Три независимые гарантии, что контроллер молчит на шине:
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
// Режим задаётся константой при установке драйвера и нигде не меняется:
// восстановление шины (twai_initiate_recovery) режим не трогает, а повторная
// установка драйвера в этой сборке выполняется тем же кодом с той же
// константой.
#pragma once

#include "../../../compat/safety/fc_build_profile.h"

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

    uint64_t started_us;
    uint64_t last_frame_us;
    uint64_t receive_errors;  // twai_receive вернул не ESP_OK и не таймаут
} FcCanStats;

/** Поднять TWAI в listen-only и запустить приём. */
bool fc_can_passive_start(void);
void fc_can_passive_stop(void);
bool fc_can_passive_running(void);

FcCanStats fc_can_passive_stats(void);
/** Последние кадры, новейший последним. Возвращает сколько скопировано. */
uint32_t fc_can_passive_ring(FcCanFrame *out, uint32_t max);
void fc_can_passive_reset_stats(void);
const char *fc_can_bus_state_name(uint32_t state);
uint32_t fc_can_passive_stack_watermark(void);

#endif  // FC_CAN_RX_AVAILABLE
