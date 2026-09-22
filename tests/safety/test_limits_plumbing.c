// Пределы тока: разбор с ESC и иерархия источников (ТЗ v0.9E §7, §8, §10, §18).
//
// До v0.9E прошивка сообщала Refloat выдуманные 10 А. Эти тесты закрепляют
// то, что сломалось бы молча: раскладку полей, отказ при чужой сигнатуре и
// безопасное пересечение двух НЕсимметричных половин.

#include "../../compat/can/fc_vesc_mcconf.h"
#include "../../compat/config/floatcore_limits.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

void fc_test_check(bool ok, const char *what);
void fc_test_note(const char *fmt, ...);
#define check fc_test_check
#define note fc_test_note

// Синтетическое ТЕЛО конфигурации: сигнатура и нужные поля на настоящих
// смещениях, остальное — нули. Числа взяты с живых половин
// (backups/vesc/v0.8b-post-detection), смещения — из раскладки
// confgenerator.c той же прошивки.
#define BODY_LEN 483u

static uint8_t BODY[BODY_LEN];

static void put_f32_auto(uint8_t *p, uint32_t raw) {
    p[0] = (uint8_t) (raw >> 24);
    p[1] = (uint8_t) (raw >> 16);
    p[2] = (uint8_t) (raw >> 8);
    p[3] = (uint8_t) raw;
}

static void build_body(void) {
    memset(BODY, 0, sizeof BODY);
    put_f32_auto(BODY + 0, 0x2efd0142u);   // сигнатура раскладки
    put_f32_auto(BODY + 8, 0x40a00000u);   // l_current_max    = +5.0
    put_f32_auto(BODY + 12, 0xc0400000u);  // l_current_min    = -3.0
    put_f32_auto(BODY + 16, 0x40a00000u);  // l_in_current_max = +5.0
    put_f32_auto(BODY + 20, 0xbf800000u);  // l_in_current_min = -1.0
    put_f32_auto(BODY + 169, 0x3ca08312u); // foc_motor_flux_linkage = 0.0196
    BODY[447] = 30;                        // si_motor_poles
}

static bool near(float a, float b) {
    float d = a - b;
    return d > -0.001f && d < 0.001f;
}

static void test_parse(void) {
    note("РАЗБОР ПРЕДЕЛОВ С ESC");

    build_body();
    FcMcconfLimits l = fc_vesc_mcconf_limits(BODY, sizeof BODY);
    check(l.valid, "тело реальной половины разобрано");
    check(near(l.current_max, 5.0f), "l_current_max = +5.0 А");
    check(near(l.current_min, -3.0f), "l_current_min = -3.0 А");
    check(near(l.in_current_max, 5.0f), "l_in_current_max = +5.0 А");
    check(near(l.in_current_min, -1.0f), "l_in_current_min = -1.0 А");
    check(!near(l.current_max, -l.current_min), "пределы НЕсимметричны: +5 против -3");
    check(near(l.flux_linkage, 0.0196f), "потокосцепление 0.0196 Вб");
    check(l.motor_poles == 30, "полюсов 30");
    check(near(fc_vesc_torque_constant(l.motor_poles, l.flux_linkage), 0.441f),
          "Kt одного мотора 0.441 Н·м/А = 1.5 x 15 x 0.0196");

    // Сдвиг раскладки на один байт — ровно та ошибка, которую я допустил,
    // взяв смещения из кадра вместо тела. Сигнатура при этом совпадает, и
    // поймать это может только проверка правдоподобия.
    static uint8_t shifted[BODY_LEN + 1];
    shifted[0] = 0x0e;
    memcpy(shifted + 1, BODY, BODY_LEN);
    FcMcconfLimits bad = fc_vesc_mcconf_limits(shifted, sizeof shifted);
    check(!bad.valid, "сдвиг раскладки на байт отвергнут, а не принят как 'другие токи'");

    // Чужая сигнатура — отказ целиком.
    static uint8_t alien[BODY_LEN];
    memcpy(alien, BODY, BODY_LEN);
    alien[0] = 0xff;
    check(!fc_vesc_mcconf_limits(alien, sizeof alien).valid, "чужая сигнатура отвергнута");

    // Короткий ответ.
    check(!fc_vesc_mcconf_limits(BODY, 10).valid, "усечённый ответ отвергнут");
    check(!fc_vesc_mcconf_limits(NULL, 100).valid, "пустой указатель отвергнут");
}

static void test_intersection(void) {
    note("БЕЗОПАСНОЕ ПЕРЕСЕЧЕНИЕ ДВУХ ПОЛОВИН");

    FcMcconfLimits a = {.current_max = 5.0f,
                        .current_min = -3.0f,
                        .in_current_max = 5.0f,
                        .in_current_min = -1.0f,
                        .flux_linkage = 0.0196f,
                        .motor_poles = 30,
                        .valid = true};
    FcMcconfLimits b = a;

    FcMcconfLimits same = fc_vesc_mcconf_intersect(&a, &b);
    check(same.valid && near(same.current_max, 5.0f) && near(same.current_min, -3.0f),
          "одинаковые половины дают те же пределы");

    // Момент СКЛАДЫВАЕТСЯ: координатор посылает одну величину обеим, и на
    // валу получается сумма. Скрытого множителя два быть не должно.
    check(near(same.flux_linkage, 0.0392f), "потокосцепление пары = сумма");
    check(near(fc_vesc_torque_constant(same.motor_poles, same.flux_linkage), 0.882f),
          "Kt пары вдвое больше одиночного");

    FcMcconfLimits other_motor = a;
    other_motor.motor_poles = 14;
    check(!fc_vesc_mcconf_intersect(&a, &other_motor).valid,
          "разное число полюсов: складывать нечего, пересечение отвергнуто");

    // Половины различаются: берём наименее разрешающее с КАЖДОЙ стороны.
    b.current_max = 4.0f;  // строже по разгону
    b.current_min = -2.0f; // строже по торможению
    FcMcconfLimits mix = fc_vesc_mcconf_intersect(&a, &b);
    check(near(mix.current_max, 4.0f), "положительный предел по минимуму");
    check(near(mix.current_min, -2.0f), "отрицательный предел по НАИМЕНЬШЕМУ МОДУЛЮ");

    // Асимметрия сохраняется: предполагать ±5 было бы ошибкой в полтора раза
    // по торможению.
    check(!near(mix.current_max, -mix.current_min),
          "асимметрия не сглаживается: разгон и торможение остаются разными");

    FcMcconfLimits unknown = {0};
    check(!fc_vesc_mcconf_intersect(&a, &unknown).valid,
          "пересечение с непрочитанной половиной не определено");
    check(!fc_vesc_mcconf_intersect(NULL, &b).valid, "пустой указатель отвергнут");
}

static void test_hierarchy(void) {
    note("ИЕРАРХИЯ ИСТОЧНИКОВ (ТЗ v0.9E §8)");

    floatcore_limits_init();
    // До чтения ESC действует только собственный предел FloatCore.
    float before = fc_effective_current_max();
    check(before > 0.0f, "до чтения ESC предел берётся у FloatCore");

    FcSourceLimits esc = {
        .present = true,
        .current_max = 5.0f,
        .current_min = -3.0f,
        .in_current_max = 5.0f,
        .in_current_min = -1.0f,
    };
    floatcore_limits_set_esc(0, &esc);
    floatcore_limits_set_esc(1, &esc);

    check(near(fc_effective_current_max(), 5.0f), "после чтения ESC действует 5 А");
    check(near(fc_effective_current_min(), -3.0f), "и -3 А, а не -5");
    check(fc_effective_current_max() < before,
          "реальный предел строже собственного: модель не может РАЗРЕШИТЬ больше");
}

void test_limits_plumbing_all(void) {
    test_parse();
    test_intersection();
    test_hierarchy();
}
