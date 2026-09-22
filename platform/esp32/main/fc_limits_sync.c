#include "fc_limits_sync.h"

#include "fc_can_bus.h"
#include "fc_log_port.h"
#include "fc_platform.h"

#include "../../../compat/config/floatcore_limits.h"
#include "../../../compat/diag/fc_shadow.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <string.h>

#define TAG "limits"

#define ID_A 118u
#define ID_B 100u

static FcLimitsSyncStatus ST;

#if FC_CAN_DIAG_TX_AVAILABLE

static bool read_half(uint8_t id, FcMcconfLimits *out) {
    static uint8_t buf[512];
    uint16_t len = 0;

    // Повторы с выдержкой. Диагностический слой держит минимальный интервал
    // между запросами в 200 мс (FC_CAN_DIAG_MIN_INTERVAL_US, заведён на
    // v0.7B, чтобы не заливать чужую шину), поэтому два запроса подряд к
    // разным половинам штатно отвергаются как «придержано». Синхронизация
    // происходит редко, и ждать здесь дешевле, чем ослаблять ограничитель.
    bool ok = false;
    for (int attempt = 0; attempt < 5 && !ok; ++attempt) {
        if (attempt) {
            vTaskDelay(pdMS_TO_TICKS(250));
        }
        ok = fc_can_bus_diag_request(FC_CAN_DIAG_MCCONF, id, 500, buf, sizeof buf, &len);
    }
    if (!ok || len < 2) {
        return false;
    }
    // Первый байт ответа — номер пакета COMM; конфигурация начинается со
    // второго, и первые её четыре байта — сигнатура раскладки.
    *out = fc_vesc_mcconf_limits(buf + 1, (uint16_t) (len - 1));
    return out->valid;
}

bool fc_limits_sync(void) {
    ++ST.attempts;

    FcMcconfLimits a, b;
    memset(&a, 0, sizeof a);
    memset(&b, 0, sizeof b);

    bool ok_a = read_half(ID_A, &a);
    bool ok_b = read_half(ID_B, &b);
    ST.half[0] = a;
    ST.half[1] = b;

    if (!ok_a || !ok_b) {
        // Частичного применения нет намеренно: пределы одной половины ничего
        // не говорят о второй, а обе половины крутят один вал.
        FC_LOGW(TAG, "пределы не синхронизированы: 118 %s, 100 %s", ok_a ? "ок" : "НЕТ",
                ok_b ? "ок" : "НЕТ");
        return false;
    }

    ST.common = fc_vesc_mcconf_intersect(&a, &b);
    if (!ST.common.valid) {
        return false;
    }

    // Неразобранные поля помечаются NAN, а не нулём. Агрегация пропускает
    // неконечные значения (floatcore_limits.c:83), поэтому температурные
    // пороги и предел скважности останутся за источником FloatCore. Ноль
    // здесь означал бы «порог перегрева 0 °C» и обнулил бы защиту при
    // выборе минимума.
    FcSourceLimits la = {
        .present = true,
        .current_max = a.current_max,
        .current_min = a.current_min,
        .in_current_max = a.in_current_max,
        .in_current_min = a.in_current_min,
        .temp_fet_start = NAN,
        .temp_fet_end = NAN,
        .temp_motor_start = NAN,
        .temp_motor_end = NAN,
        .max_duty = NAN,
    };
    FcSourceLimits lb = la;
    lb.current_max = b.current_max;
    lb.current_min = b.current_min;
    lb.in_current_max = b.in_current_max;
    lb.in_current_min = b.in_current_min;

    floatcore_limits_set_esc(0, &la);
    floatcore_limits_set_esc(1, &lb);
    fc_vesc_if_refresh_limits();
    // Эквивалентное потокосцепление ПАРЫ: координатор посылает одну и ту же
    // величину обеим половинам, поэтому момент на валу — сумма, и Refloat
    // должен видеть постоянную пары, а не одного мотора. Иначе в контуре
    // остаётся скрытый множитель два.
    fc_vesc_if_set_motor_params(ST.common.flux_linkage, ST.common.motor_poles);
    // Теневой разбор считает упор в предел ESC по фактическому значению, а
    // не по предположению.
    fc_shadow_set_esc_limit(ST.common.current_max);

    ST.applied = true;
    ++ST.successes;
    ST.last_sync_us = fc_uptime_us();
    FC_LOGI(TAG, "Kt пары %.4f Н·м/А (lambda %.6f, полюсов %u)",
            (double) fc_vesc_torque_constant(ST.common.motor_poles, ST.common.flux_linkage),
            (double) ST.common.flux_linkage, ST.common.motor_poles);
    FC_LOGI(TAG, "пределы с ESC: 118 %+.1f/%+.1f, 100 %+.1f/%+.1f, общий %+.1f/%+.1f А",
            (double) a.current_max, (double) a.current_min, (double) b.current_max,
            (double) b.current_min, (double) ST.common.current_max,
            (double) ST.common.current_min);
    return true;
}

#else

bool fc_limits_sync(void) {
    ++ST.attempts;
    return false;
}

#endif

FcLimitsSyncStatus fc_limits_sync_status(void) {
    return ST;
}
