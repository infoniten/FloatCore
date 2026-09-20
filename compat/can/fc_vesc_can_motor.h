// Сериализация моторной команды VESC по CAN (ТЗ v0.9A §4).
//
// ЕДИНСТВЕННОЕ место во всём репозитории, где моторная команда превращается
// в байты. В профиле LAB_SAFE этот файл компилируется в пустую единицу
// трансляции: ни одного символа, а не «функция, которую не зовут».
//
// ЧЕГО ЗДЕСЬ НЕТ И НЕ БУДЕТ. Решений о безопасности. Этот слой не знает про
// supervisor, свежесть, здоровье узлов и вооружение; он получает уже
// проверенную величину и превращает её в кадр. Обратное — сериализатор,
// который «на всякий случай» сам подрезает ток, — опаснее, чем кажется: он
// молча превращает ошибку расчёта в допустимую команду, и ни один тест
// уровнем выше этого не заметит.
//
// По той же причине здесь НЕ зажимаются NaN и Inf: они обязаны быть
// отвергнуты выше (Motor Gate, координатор). Если такое значение дошло сюда,
// это ошибка архитектуры, и прятать её округлением нельзя.
//
// Разрешён РОВНО ОДИН тип пакета — CAN_PACKET_SET_CURRENT. Ни duty, ни rpm,
// ни position: Refloat управляет током, остальное на этом этапе не нужно, а
// каждый лишний тип — это ещё один способ подать на мотор что-то неожиданное.
#pragma once

#include "../safety/fc_build_profile.h"

#include <stdbool.h>
#include <stdint.h>

#if FC_CAN_TX_AVAILABLE

// Номер из bldc/datatypes.h, перечисление CAN_PACKET_ID.
#define FC_CAN_PACKET_SET_CURRENT 1u

// Тип обязан быть моторным — это не самоочевидно, а проверяется: таблица
// номеров живёт в fc_vesc_can.c, и расхождение с ней означало бы, что мы
// шлём не то, что думаем.
_Static_assert(FC_CAN_PACKET_SET_CURRENT == 1u, "CAN_PACKET_SET_CURRENT = 1 в bldc");

typedef struct {
    uint32_t eid;
    uint8_t data[8];
    uint8_t len;
} FcMotorFrame;

typedef enum {
    FC_MOTOR_FRAME_OK = 0,
    FC_MOTOR_FRAME_BAD_VALUE,  // NaN или Inf — сюда такое доходить не должно
    FC_MOTOR_FRAME_BAD_TARGET, // broadcast 255 или наш собственный адрес
} FcMotorFrameResult;

/**
 * Собрать кадр CAN_PACKET_SET_CURRENT.
 *
 *     eid  = controller_id | (1 << 8)
 *     data = int32 big-endian, миллиамперы
 *
 * Формат взят из bldc/comm/comm_can.c: comm_can_set_current() умножает ток на
 * 1000 и кладёт buffer_append_int32, то есть старшим байтом вперёд.
 *
 * Широковещательный адрес 255 отвергается отдельно: на него ответили бы обе
 * половины, и «команда одной половине» перестала бы существовать как понятие.
 */
FcMotorFrameResult fc_vesc_can_motor_build_current(uint8_t target_id, float amps,
                                                   FcMotorFrame *out);

/** Обратный разбор — для тестов и для разбора собственного трафика. */
bool fc_vesc_can_motor_decode_current(const FcMotorFrame *f, uint8_t *target_id, float *amps);

#endif // FC_CAN_TX_AVAILABLE
