// Footpad ADC на ESP32, этап v0.5 (ТЗ §9).
//
// Физические footpad-датчики НЕ подключены. Задача этого модуля — гарантировать,
// что Refloat видит доску как НЕ активированную.
//
// Обоснование выбранного значения (refloat-upstream/src/footpad_sensor.c:31-32):
//
//     bool adc1_on = config->fault_adc1 == 0.0f || adc1 > config->fault_adc1;
//
// то есть датчик считается нажатым, если напряжение ВЫШЕ порога
// `fault_adc1`/`fault_adc2`. Значения по умолчанию в settings.xml — 2.00 В для
// обоих каналов. Отдаём 0.0 В: строго ниже порога при любом его допустимом
// значении > 0, и это же значение соответствует физическому «нет входа с
// подтяжкой к земле».
//
// ВНИМАНИЕ, отдельный случай: если порог выставлен в 0, Refloat считает канал
// отключённым и трактует его как ПОСТОЯННО НАЖАТЫЙ (первая ветка условия).
// Никакое значение ADC этого не отменяет — поэтому 0 В достаточно, только пока
// конфигурация содержит ненулевые пороги. Проверка этого факта делается
// отдельно, в fc_safety_check_footpad_config() (см. app_main.c), и её результат
// печатается в баннере.
//
// Возврат -1.0 В (VESC: «пина нет на железе») здесь НЕ используется намеренно:
// это значение тоже ниже порога, но оно означало бы «канал отсутствует», а мы
// хотим сообщить «канал есть и он отпущен».

#include "fc_platform.h"

#include "../../../compat/safety/fc_build_profile.h"

// VESC_PIN_ADC1 = 7, VESC_PIN_ADC2 = 8 (vesc_c_if.h). Числа продублированы,
// чтобы не включать сюда заголовок SDK: этот файл видят и модули без него.
#define FC_VESC_PIN_ADC1 7
#define FC_VESC_PIN_ADC2 8

#define FC_ADC_SAFE_VOLTAGE 0.0f

// Имитация нажатых футпадов — только для лабораторной диагностики (ТЗ v0.6D §16).
//
// Зачем понадобилась. Refloat запрашивает балансировочный ток только в
// состоянии RUNNING, а войти в него без нажатых футпадов он не может. Между
// тем главную проверку этапа — «наклон доски в одну сторону даёт ток в
// восстанавливающую сторону» — нельзя провести по одному лишь pitch: знак
// обязан быть подтверждён на запрошенном токе.
//
// Почему это безопасно. Имитация не касается пути к мотору вовсе: она меняет
// только напряжение, которое видит Refloat. Запрошенный ток по-прежнему
// проходит Motor Gate, а backend физической отправки в профиле LAB_SAFE не
// существует как код — physically_sent обязан остаться нулём и при активном
// RUNNING. Именно это и проверяется в ходе теста.
//
// Компилируется только при FC_LAB_DIAGNOSTICS. В профиле мотора этого кода
// в двоичном файле нет.
#if FC_LAB_DIAGNOSTICS
static float g_sim_voltage = FC_ADC_SAFE_VOLTAGE;
static bool g_sim_active;

void fc_adc_simulate_footpads(bool on, float voltage) {
    g_sim_active = on;
    g_sim_voltage = on ? voltage : FC_ADC_SAFE_VOLTAGE;
}

bool fc_adc_simulation_active(void) {
    return g_sim_active;
}
#endif

float fc_adc_safe_voltage(void) {
    return FC_ADC_SAFE_VOLTAGE;
}

float fc_adc_read(int vesc_pin) {
    if (vesc_pin == FC_VESC_PIN_ADC1 || vesc_pin == FC_VESC_PIN_ADC2) {
#if FC_LAB_DIAGNOSTICS
        if (g_sim_active) {
            return g_sim_voltage;
        }
#endif
        return FC_ADC_SAFE_VOLTAGE;
    }
    return -1.0f;  // остальные пины на плате не заведены
}
