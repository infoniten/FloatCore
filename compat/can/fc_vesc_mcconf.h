// Разбор пределов тока из ответа COMM_GET_MCCONF (ТЗ v0.9E §7, §10).
//
// ЗАЧЕМ ОТДЕЛЬНЫЙ МОДУЛЬ. До v0.9E прошивка сообщала Refloat выдуманное
// число: fc_vesc_if.c держал l_current_max = 10 А с комментарием «реальные
// придут по CAN на следующем этапе». Этап наступил, половины настроены на
// +5 / −3 А, их конфигурация читается по CAN — но в зеркало не попадала, и
// Refloat считал, что ему можно вдвое больше, чем разрешает железо.
//
// ЧТО ЗДЕСЬ РАЗБИРАЕТСЯ. Только пределы тока, и только они. Полный
// сериализатор mcconf на плате не нужен и был бы опасен: чем больше полей
// мы беремся толковать, тем больше мест, где толкование разойдётся с
// прошивкой ESC.
//
// ЗАЩИТА ОТ ЧУЖОЙ РАСКЛАДКИ. Первые четыре байта конфигурации — сигнатура
// раскладки. Если она не та, разбор отвергается целиком: считать смещения
// от чужой структуры хуже, чем не считать вовсе.
#pragma once

#include <stdbool.h>
#include <stdint.h>

// Сигнатура mcconf прошивки 6.6 (release_6_06), проверена на обеих половинах
// при снятии резервных копий (backups/vesc/README.md).
#define FC_MCCONF_SIGNATURE 0x2efd0142u

/**
 * Моментная постоянная по соглашению VESC: Kt = 1.5 · пар полюсов · λ.
 *
 * Та же формула, что в refloat-upstream/src/motor_data.c:106 и в
 * lib/utils.h:38. Держать её здесь отдельной функцией нужно затем, чтобы
 * согласие с Refloat проверялось тестом, а не совпадением по памяти.
 */
float fc_vesc_torque_constant(uint8_t poles, float flux_linkage);

/** buffer_get_float32_auto из bldc/util/buffer.c — нестандартная упаковка float. */
float fc_vesc_float32_auto(const uint8_t *b);

typedef struct {
    float current_max;     // l_current_max, А
    float current_min;     // l_current_min, А (отрицательное)
    float in_current_max;  // l_in_current_max, А
    float in_current_min;  // l_in_current_min, А (отрицательное)

    // Параметры мотора. Нужны не для управления током, а для того, чтобы
    // Refloat знал НАСТОЯЩУЮ моментную постоянную: он переводит свой момент
    // в ток через неё, и ошибка здесь умножает запрашиваемый ток ровно во
    // столько же раз (docs/control_scale_audit.md).
    float flux_linkage;    // foc_motor_flux_linkage, Вб
    uint8_t motor_poles;   // si_motor_poles

    bool valid;
} FcMcconfLimits;

/**
 * Разобрать пределы из ПОЛЕЗНОЙ НАГРУЗКИ ответа COMM_GET_MCCONF.
 *
 * payload — байты ответа БЕЗ первого байта с номером пакета COMM, то есть
 * начиная с сигнатуры. len — их количество.
 *
 * Возвращает valid = false, если сигнатура не совпала или длины не хватает.
 */
FcMcconfLimits fc_vesc_mcconf_limits(const uint8_t *payload, uint16_t len);

/**
 * Безопасное пересечение пределов двух половин.
 *
 * Положительные берутся по минимуму, отрицательные — по максимуму (то есть
 * по наименьшему модулю). Предполагать симметрию нельзя: на этой машине
 * +5 и −3, и «±5» было бы ошибкой в полтора раза по торможению.
 */
FcMcconfLimits fc_vesc_mcconf_intersect(const FcMcconfLimits *a, const FcMcconfLimits *b);
