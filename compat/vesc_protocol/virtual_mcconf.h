// Virtual mcConfig — read-only проекция состояния FloatCore в формат
// Motor Configuration VESC.
//
// Это НЕ конфигурация мотора. Собственных параметров здесь нет и быть не может:
// значения вычисляются при каждом запросе из FloatCore Config
// (compat/config/floatcore_limits.h). Запись невозможна — обработчика
// COMM_SET_MCCONF не существует.
//
// Нужен ровно для одного: Refloat QML читает `VescIf.mcConfig()`, чтобы взять
// пределы для шкал и порогов. Без этого VESC Tool подставляет туда собственные
// значения по умолчанию, и шкалы не соответствуют реальным ограничениям.
//
// Полный аудит обращений — docs/virtual_mcconfig.md.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Способ кодирования параметра (ConfigParams::setParamSerial в VESC Tool). */
typedef enum {
    MC_KIND_DOUBLE16,       // int16, значение × scale
    MC_KIND_DOUBLE32,       // int32, значение × scale
    MC_KIND_DOUBLE32_AUTO,  // формат buffer_append_float32_auto
    MC_KIND_U8,
    MC_KIND_I8,
    MC_KIND_U16,
    MC_KIND_I16,
    MC_KIND_U32,
    MC_KIND_I32,
    MC_KIND_BYTE,  // enum / bool / bitfield — всегда один байт
} McParamKind;

typedef struct {
    const char *name;
    uint8_t kind;
    float scale;
    float def;  // значение по умолчанию из схемы VESC Tool
} McParam;

typedef struct {
    const char *version;   // версия VESC Tool, например "6.06"
    uint32_t signature;    // ConfigParams::getSignature() этой схемы
    const McParam *params;
    uint16_t param_count;
    uint16_t blob_size;    // размер сериализованной конфигурации вместе с сигнатурой
} McConfSchema;

/**
 * Проецируемые значения. Ровно те параметры, которые читает Refloat QML,
 * плюс две пары «end», без которых пороги в UI выглядели бы противоречиво.
 *
 * Структура заполняется на каждый запрос и нигде не сохраняется.
 */
typedef struct {
    // Читаются Refloat QML (ui.qml.in:243-249)
    int si_battery_cells;
    float l_temp_motor_start;
    float l_temp_fet_start;
    float l_current_min;
    float l_current_max;
    float l_in_current_min;
    float l_in_current_max;

    // Не читаются QML напрямую, но должны быть согласованы с *_start:
    // иначе в VESC Tool окажется end < start.
    float l_temp_motor_end;
    float l_temp_fet_end;
} VirtualMcConfValues;

// --- доступные схемы -------------------------------------------------------

extern const McConfSchema mcconf_schema_6_06;
extern const McConfSchema mcconf_schema_7_01;

size_t virtual_mcconf_schema_count(void);
const McConfSchema *virtual_mcconf_schema_at(size_t index);

/** Поиск по строке версии ("6.06"). NULL, если такой схемы нет. */
const McConfSchema *virtual_mcconf_schema_by_version(const char *version);

/** Схема по умолчанию — самая новая из известных. */
const McConfSchema *virtual_mcconf_default_schema(void);

// --- кодирование -----------------------------------------------------------

/**
 * Собрать payload ответа COMM_GET_MCCONF / COMM_GET_MCCONF_DEFAULT.
 *
 * `values` == NULL или `is_default` == true → отдаются значения по умолчанию
 * из схемы VESC Tool без проекции FloatCore.
 *
 * Возвращает длину или 0, если не хватило места.
 */
size_t virtual_mcconf_encode(
    const McConfSchema *schema, const VirtualMcConfValues *values, bool is_default, uint8_t *out,
    size_t cap
);

/** Кодировщик формата buffer_append_float32_auto (bldc/util/buffer.c). */
uint32_t virtual_mcconf_float32_auto(float value);
