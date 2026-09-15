#include "fc_battery_model.h"

#include <math.h>
#include <stddef.h>

int fc_battery_cells_from_regen_cutoff(const FcBatteryModel *m) {
    // Regen-отсечка ставится на полный заряд, то есть примерно 4.2 В на
    // ячейку. Это самая надёжная опорная точка: нижние отсечки владелец
    // двигает по вкусу, верхнюю — практически никогда, её задаёт химия.
    if (m->regen_cut_end <= 0.0f) {
        return 0;
    }
    float est = m->regen_cut_end / 4.2f;
    return (int) (est + 0.5f);
}

float fc_battery_refloat_lv_volts(const FcBatteryModel *m) {
    // То же правило, что в refloat-upstream/src/motor_data.c:82-84.
    if (m->refloat_tiltback_lv < 10.0f && m->cells > 0) {
        return m->refloat_tiltback_lv * (float) m->cells;
    }
    return m->refloat_tiltback_lv;
}

float fc_battery_refloat_hv_volts(const FcBatteryModel *m) {
    if (m->refloat_tiltback_hv < 10.0f && m->cells > 0) {
        return m->refloat_tiltback_hv * (float) m->cells;
    }
    return m->refloat_tiltback_hv;
}

bool fc_battery_model_valid(const FcBatteryModel *m, const char **why) {
    const char *dummy = NULL;
    if (!why) {
        why = &dummy;
    }
    *why = NULL;

    if (m->cells <= 0) {
        *why = "число ячеек не задано";
        return false;
    }
    if (!(m->cut_end < m->cut_start)) {
        *why = "нижняя отсечка не ниже начала снижения тока";
        return false;
    }
    if (!(m->regen_cut_start < m->regen_cut_end)) {
        *why = "regen-отсечка задана в обратном порядке";
        return false;
    }
    if (!(m->min_vin < m->cut_end)) {
        *why = "аппаратный минимум напряжения не ниже нижней отсечки";
        return false;
    }
    if (!(m->max_vin > m->regen_cut_end)) {
        *why = "аппаратный максимум напряжения не выше regen-отсечки";
        return false;
    }

    // Главная проверка: согласуется ли число ячеек с абсолютными вольтами.
    float per_cell_cut = m->cut_start / (float) m->cells;
    if (per_cell_cut < FC_CELL_CUTOFF_MIN_V || per_cell_cut > FC_CELL_CUTOFF_MAX_V) {
        *why = "отсечка в пересчёте на ячейку вне разумного окна — число ячеек "
               "не описывает ту же батарею, что и вольты";
        return false;
    }
    float per_cell_full = m->regen_cut_end / (float) m->cells;
    if (per_cell_full > FC_CELL_FULL_MAX_V) {
        *why = "regen-отсечка в пересчёте на ячейку выше предела химии Li-ion";
        return false;
    }

    // Требование самого Refloat, дословно из описания параметра
    // tiltback_lv в settings.xml: «Make sure your Voltage Cutoff Start and End
    // are below this threshold». Откат должен предупреждать РАНЬШЕ, чем
    // прошивка начнёт срезать ток, иначе предупреждения не будет вовсе —
    // сначала пропадёт тяга.
    float lv = fc_battery_refloat_lv_volts(m);
    if (!(lv > m->cut_start)) {
        *why = "порог отката Refloat не выше отсечки ESC: тяга пропадёт раньше "
               "предупреждения";
        return false;
    }

    float hv = fc_battery_refloat_hv_volts(m);
    if (!(hv < m->max_vin)) {
        *why = "порог отката по высокому напряжению не ниже аппаратного максимума";
        return false;
    }
    if (!(hv > lv)) {
        *why = "порог высокого напряжения не выше порога низкого";
        return false;
    }

    return true;
}
