// Описание шины к контроллерам мотора (ТЗ v0.6A §13, уточнено в v0.7A).
//
// Здесь речь о ТРАНСПОРТЕ КОМАНД МОТОРУ, и его по-прежнему нет. С v0.7A в
// сборке появился приём CAN (fc_can_bus.c), и путать эти две вещи нельзя:
// слушать шину и уметь что-то на неё отправить — разные способности, и вторая
// отсутствует как код.
//
// Проверяется таблицей символов в tools/esp32_smoke.sh: символов передачи в
// образе нет.

#include "fc_platform.h"

#include "../../../compat/safety/fc_build_profile.h"

const char *fc_can_backend_name(void) {
#if FC_CAN_TX_AVAILABLE
    return "ОШИБКА: передача в CAN разрешена профилем";
#elif FC_CAN_DIAG_TX_AVAILABLE
    // Передача есть, но только read-only запросы из белого списка. Называть
    // это «RX only» было бы неправдой, а «транспорт есть» — опасной
    // неправдой: команду мотору через этот путь собрать нечем.
    return "RX + read-only diag TX (" FC_CAN_PROFILE_NAME "), транспорта команд мотору нет";
#elif FC_CAN_RX_AVAILABLE
    return "RX only (" FC_CAN_PROFILE_NAME "), транспорта команд мотору нет";
#else
    return "unavailable (TWAI не собран, трансивер не подключён)";
#endif
}
