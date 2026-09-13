// Профили сборки FloatCore (ТЗ v0.6A §13, §14).
//
// Профиль — не настройка во время работы, а свойство собранного двоичного
// файла. Смысл в том, чтобы опасные пути не существовали, а не были
// «выключены флагом»: выключенный флаг можно переключить, отсутствующий код
// переключить нельзя.
//
//   FLOATCORE_LAB_SAFE       лабораторная сборка. Единственная, которую
//                            разрешено прошивать на этапах v0.5–v0.6x.
//   FLOATCORE_MOTOR_CAPABLE  задел под будущее. Существует как структура,
//                            собираться на живой плате не должен.
//
// Что LAB_SAFE гарантирует на этапе компиляции:
//   * fc_motor_gate_set_backend() не объявлена и не определена — зарегистрировать
//     backend физической отправки нечем;
//   * fc_supervisor_motor_output_permitted() — константа false, компилятор
//     видит это и выкидывает недостижимые ветки;
//   * драйвер CAN/TWAI не включается в сборку ни одним компонентом;
//   * опасные диагностические команды не компилируются.
//
// Проверка не на слово: tools/esp32_smoke.sh ищет в .elf символы TX и
// safety-bypass. Их отсутствие — часть прогона, а не обещание.
#pragma once

#if defined(FLOATCORE_LAB_SAFE) && defined(FLOATCORE_MOTOR_CAPABLE)
#error "FLOATCORE_LAB_SAFE и FLOATCORE_MOTOR_CAPABLE взаимоисключающи"
#endif

#if !defined(FLOATCORE_LAB_SAFE) && !defined(FLOATCORE_MOTOR_CAPABLE)
#error "Профиль сборки не задан: определите FLOATCORE_LAB_SAFE или FLOATCORE_MOTOR_CAPABLE"
#endif

#ifdef FLOATCORE_LAB_SAFE

#define FC_PROFILE_NAME "LAB_SAFE"
// Backend физической отправки отсутствует как код.
#define FC_MOTOR_BACKEND_AVAILABLE 0
// Драйвер шины к контроллерам мотора не собирается.
#define FC_CAN_TX_AVAILABLE 0
// Команды, способные ослабить безопасность или вызвать выход на мотор.
#define FC_DANGEROUS_DIAGNOSTICS 0
// Команды, которые меняют состояние, но физически безопасны: они нужны
// для проверки самих механизмов безопасности (watchdog, panic, persistence).
#define FC_LAB_DIAGNOSTICS 1

#else  // FLOATCORE_MOTOR_CAPABLE

#define FC_PROFILE_NAME "MOTOR_CAPABLE"
#define FC_MOTOR_BACKEND_AVAILABLE 1
#define FC_CAN_TX_AVAILABLE 1
#define FC_DANGEROUS_DIAGNOSTICS 0
#define FC_LAB_DIAGNOSTICS 0

// Профиль существует только как архитектурный задел. Прошивать его на плате
// нельзя до отдельного этапа: ни supervisor с полным набором входов, ни CAN,
// ни проверки ESC ещё не написаны (docs/esp32_safety.md, раздел
// «Requirements before enabling motor output»).
#ifndef FLOATCORE_MOTOR_CAPABLE_I_KNOW_WHAT_I_AM_DOING
#error "MOTOR_CAPABLE — незавершённый профиль. Собирать и прошивать его запрещено."
#endif

#endif

// ------------------------------------ профиль шины CAN (v0.7A, v0.7B)
//
// Ортогонален профилю сборки и задаётся отдельно. Смысл тот же: не «флаг,
// который можно переключить», а наличие или отсутствие кода.
//
//   (не задан)               CAN отсутствует полностью, как было до v0.7A.
//   FLOATCORE_CAN_PASSIVE    слушать шину и только слушать. Контроллер TWAI
//                            поднимается в listen-only, ни одной функции
//                            передачи в сборке нет.
//   FLOATCORE_CAN_ACTIVE_DIAG  v0.7B: TWAI в normal mode, разрешена передача
//                            СТРОГО перечисленных read-only запросов. Ни
//                            одного типа пакета, способного повлиять на
//                            мотор или изменить конфигурацию VESC, в этом
//                            профиле собрать нельзя.
//
// Три уровня передачи различаются намеренно и НЕ сводятся к одному флагу:
//
//   FC_CAN_RX_AVAILABLE       приём кадров
//   FC_CAN_DIAG_TX_AVAILABLE  передача read-only запросов из белого списка
//   FC_CAN_TX_AVAILABLE       транспорт команд мотору — только MOTOR_CAPABLE
//
// Именно поэтому ACTIVE_DIAG не переиспользует FC_CAN_TX_AVAILABLE: этот
// макрос означает «мотору можно послать команду», и он обязан оставаться
// нулём весь v0.7B. Отравление имён ниже привязано к нему же, поэтому в
// диагностическом профиле обобщённые функции отправки остаются запрещёнными.
#if defined(FLOATCORE_CAN_PASSIVE) && defined(FLOATCORE_CAN_ACTIVE_DIAG)
#error "FLOATCORE_CAN_PASSIVE и FLOATCORE_CAN_ACTIVE_DIAG взаимоисключающи"
#endif

#if defined(FLOATCORE_CAN_PASSIVE) && FC_CAN_TX_AVAILABLE
#error "FLOATCORE_CAN_PASSIVE несовместим с профилем, где разрешена передача в CAN"
#endif

#if defined(FLOATCORE_CAN_ACTIVE_DIAG) && FC_CAN_TX_AVAILABLE
#error "FLOATCORE_CAN_ACTIVE_DIAG несовместим с транспортом команд мотору"
#endif

#if defined(FLOATCORE_CAN_ACTIVE_DIAG) && FC_MOTOR_BACKEND_AVAILABLE
#error "FLOATCORE_CAN_ACTIVE_DIAG несовместим с backend-ом выхода на мотор"
#endif

#ifdef FLOATCORE_CAN_PASSIVE
#define FC_CAN_PROFILE_NAME "PASSIVE (listen-only)"
#define FC_CAN_RX_AVAILABLE 1
#define FC_CAN_DIAG_TX_AVAILABLE 0
#elif defined(FLOATCORE_CAN_ACTIVE_DIAG)
#define FC_CAN_PROFILE_NAME "ACTIVE_DIAG (normal mode, read-only requests)"
#define FC_CAN_RX_AVAILABLE 1
#define FC_CAN_DIAG_TX_AVAILABLE 1
#else
#define FC_CAN_PROFILE_NAME "ABSENT"
#define FC_CAN_RX_AVAILABLE 0
#define FC_CAN_DIAG_TX_AVAILABLE 0
#endif
