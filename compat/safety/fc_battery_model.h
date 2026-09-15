// Проверка непротиворечивости модели батареи (ТЗ v0.7C §2, §3).
//
// Поводом послужило расхождение, найденное на стенде: `si_battery_cells = 3`
// при отсечках 34.0/31.0 В и regen-отсечках 41.5/42.0 В. Первое описывает
// батарею 3S, вторые — 10S. Обе цифры лежали рядом много месяцев и ни одна
// проверка их не сравнивала, потому что сравнивать было нечему.
//
// Что важно понимать про роли:
//
//   * Защиты САМОЙ прошивки ESC числом ячеек НЕ пользуются. Они работают в
//     абсолютных вольтах: `l_battery_cut_start/end` и
//     `l_battery_regen_cut_start/end` (bldc mc_interface.c:2440-2458).
//     Неверное число ячеек их не ослабляет.
//   * `si_battery_cells` в прошивке ESC используется ровно в одном месте —
//     оценке остатка заряда `mc_interface_get_battery_level()`
//     (mc_interface.c:1560-1580). Это информационный параметр.
//   * А вот у REFLOAT он влияет на безопасность. Пороги отката по напряжению
//     заданы НА ЯЧЕЙКУ, и Refloat умножает их на число ячеек:
//
//         motor_data.c:79-88
//         uint8_t battery_cells = VESC_IF->get_cfg_int(CFG_PARAM_si_battery_cells);
//         if (lv_threshold < 10) lv_threshold *= battery_cells;
//         if (hv_threshold < 10) hv_threshold *= battery_cells;
//
//     При значениях по умолчанию (3.0 и 4.3 В на ячейку) ошибка в числе
//     ячеек сдвигает пороги отката в разы. Для 10S-батареи значение 3 дало бы
//     порог низкого напряжения 9 В — то есть откат не сработает никогда, а
//     значение 15 дало бы 45 В, то есть откат будет включён всегда.
//
// Отсюда правило модуля: число ячеек и абсолютные отсечки должны описывать
// одну и ту же батарею, и это проверяемо.
#pragma once

#include <stdbool.h>

// Разумное окно напряжения на ячейку Li-ion для порогов отсечки. Границы
// широкие намеренно: задача — ловить расхождение в разы, а не спорить о
// десятых долях вольта.
#define FC_CELL_CUTOFF_MIN_V 2.5f
#define FC_CELL_CUTOFF_MAX_V 3.9f
#define FC_CELL_FULL_MAX_V 4.25f

typedef struct {
    int cells;

    // Абсолютные вольты из mcconf ESC.
    float cut_start;
    float cut_end;
    float regen_cut_start;
    float regen_cut_end;
    float max_vin;
    float min_vin;

    // Пороги отката Refloat. Значение меньше 10 трактуется как «на ячейку» —
    // ровно так же, как это делает сам Refloat.
    float refloat_tiltback_lv;
    float refloat_tiltback_hv;
} FcBatteryModel;

/**
 * Проверить модель целиком. false и причина первого нарушения в why.
 *
 * Умолчание — «неверна»: проверяются не только очевидные инверсии порогов, но
 * и согласие числа ячеек с абсолютными отсечками.
 */
bool fc_battery_model_valid(const FcBatteryModel *m, const char **why);

/** Оценка числа ячеек по regen-отсечке: она ставится на полный заряд. */
int fc_battery_cells_from_regen_cutoff(const FcBatteryModel *m);

/** Эффективный порог Refloat в вольтах, с учётом правила «меньше 10 — на ячейку». */
float fc_battery_refloat_lv_volts(const FcBatteryModel *m);
float fc_battery_refloat_hv_volts(const FcBatteryModel *m);
