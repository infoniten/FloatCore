#include "virtual_mcconf.h"

#include "commands.h"
#include "vesc_buffer.h"

#include <math.h>
#include <string.h>

static const McConfSchema *const SCHEMAS[] = {
    &mcconf_schema_7_01,
    &mcconf_schema_6_06,
};

size_t virtual_mcconf_schema_count(void) {
    return sizeof(SCHEMAS) / sizeof(SCHEMAS[0]);
}

const McConfSchema *virtual_mcconf_schema_at(size_t index) {
    return index < virtual_mcconf_schema_count() ? SCHEMAS[index] : NULL;
}

const McConfSchema *virtual_mcconf_schema_by_version(const char *version) {
    if (!version) {
        return NULL;
    }
    for (size_t i = 0; i < virtual_mcconf_schema_count(); ++i) {
        if (strcmp(SCHEMAS[i]->version, version) == 0) {
            return SCHEMAS[i];
        }
    }
    return NULL;
}

const McConfSchema *virtual_mcconf_default_schema(void) {
    return SCHEMAS[0];
}

uint32_t virtual_mcconf_float32_auto(float number) {
    // Точная копия buffer_append_float32_auto из bldc/util/buffer.c.
    if (fabsf(number) < 1.5e-38f) {
        number = 0.0f;
    }

    int e = 0;
    float sig = frexpf(number, &e);
    float sig_abs = fabsf(sig);
    uint32_t sig_i = 0;

    if (sig_abs >= 0.5f) {
        sig_i = (uint32_t) ((sig_abs - 0.5f) * 2.0f * 8388608.0f);
        e += 126;
    }

    uint32_t res = ((uint32_t) (e & 0xFF) << 23) | (sig_i & 0x7FFFFF);
    if (sig < 0) {
        res |= 1u << 31;
    }
    return res;
}

/**
 * Подстановка проекции FloatCore вместо значения по умолчанию.
 *
 * Возвращает false, если параметр не проецируется — тогда берётся default
 * из схемы VESC Tool. Так реализуется требование «минимальная модель»:
 * реализовано только то, что действительно используется.
 */
static bool project(const char *name, const VirtualMcConfValues *v, float *out) {
    if (strcmp(name, "si_battery_cells") == 0) {
        *out = (float) v->si_battery_cells;
    } else if (strcmp(name, "l_temp_motor_start") == 0) {
        *out = v->l_temp_motor_start;
    } else if (strcmp(name, "l_temp_motor_end") == 0) {
        *out = v->l_temp_motor_end;
    } else if (strcmp(name, "l_temp_fet_start") == 0) {
        *out = v->l_temp_fet_start;
    } else if (strcmp(name, "l_temp_fet_end") == 0) {
        *out = v->l_temp_fet_end;
    } else if (strcmp(name, "l_current_min") == 0) {
        *out = v->l_current_min;
    } else if (strcmp(name, "l_current_max") == 0) {
        *out = v->l_current_max;
    } else if (strcmp(name, "l_in_current_min") == 0) {
        *out = v->l_in_current_min;
    } else if (strcmp(name, "l_in_current_max") == 0) {
        *out = v->l_in_current_max;
    } else {
        return false;
    }
    return isfinite(*out);
}

size_t virtual_mcconf_encode(
    const McConfSchema *schema, const VirtualMcConfValues *values, bool is_default, uint8_t *out,
    size_t cap
) {
    if (!schema || cap < (size_t) schema->blob_size + 1u) {
        return 0;
    }

    size_t ind = 0;
    vb_append_uint8(out, is_default ? COMM_GET_MCCONF_DEFAULT : COMM_GET_MCCONF, &ind);
    vb_append_uint32(out, schema->signature, &ind);

    const bool projected = values != NULL && !is_default;

    for (uint16_t i = 0; i < schema->param_count; ++i) {
        const McParam *p = &schema->params[i];

        float value = p->def;
        if (projected) {
            float v;
            if (project(p->name, values, &v)) {
                value = v;
            }
        }

        switch ((McParamKind) p->kind) {
        case MC_KIND_DOUBLE16:
            vb_append_float16(out, value, p->scale, &ind);
            break;
        case MC_KIND_DOUBLE32:
            vb_append_float32(out, value, p->scale, &ind);
            break;
        case MC_KIND_DOUBLE32_AUTO:
            vb_append_uint32(out, virtual_mcconf_float32_auto(value), &ind);
            break;
        case MC_KIND_U8:
        case MC_KIND_I8:
        case MC_KIND_BYTE:
            vb_append_uint8(out, (uint8_t) (int32_t) lrintf(value), &ind);
            break;
        case MC_KIND_U16:
        case MC_KIND_I16:
            vb_append_uint16(out, (uint16_t) (int32_t) lrintf(value), &ind);
            break;
        case MC_KIND_U32:
        case MC_KIND_I32:
            vb_append_uint32(out, (uint32_t) (int64_t) llrintf(value), &ind);
            break;
        }
    }

    return ind;
}
