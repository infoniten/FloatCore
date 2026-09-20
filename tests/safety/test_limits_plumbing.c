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

// Первые 24 байта ТЕЛА конфигурации, снятые с живой половины 118
// (backups/vesc/v0.8b-post-detection/mcconf_id118.bin, без первого байта с
// номером пакета COMM). Больше не нужно: дальше идут поля, которых мы не
// разбираем.
static const uint8_t BODY_118[] = {
    0x2e, 0xfd, 0x01, 0x42,  // сигнатура раскладки
    0x01, 0x00, 0x02, 0x00,  // поля, которые мы не толкуем
    0x40, 0xa0, 0x00, 0x00,  // l_current_max    = +5.0
    0xc0, 0x40, 0x00, 0x00,  // l_current_min    = -3.0
    0x40, 0xa0, 0x00, 0x00,  // l_in_current_max = +5.0
    0xbf, 0x80, 0x00, 0x00,  // l_in_current_min = -1.0
};

static bool near(float a, float b) {
    float d = a - b;
    return d > -0.001f && d < 0.001f;
}

static void test_parse(void) {
    note("РАЗБОР ПРЕДЕЛОВ С ESC");

    FcMcconfLimits l = fc_vesc_mcconf_limits(BODY_118, sizeof BODY_118);
    check(l.valid, "тело реальной половины разобрано");
    check(near(l.current_max, 5.0f), "l_current_max = +5.0 А");
    check(near(l.current_min, -3.0f), "l_current_min = -3.0 А");
    check(near(l.in_current_max, 5.0f), "l_in_current_max = +5.0 А");
    check(near(l.in_current_min, -1.0f), "l_in_current_min = -1.0 А");
    check(!near(l.current_max, -l.current_min), "пределы НЕсимметричны: +5 против -3");

    // Сдвиг раскладки на один байт — ровно та ошибка, которую я допустил,
    // взяв смещения из кадра вместо тела. Сигнатура при этом совпадает, и
    // поймать это может только проверка правдоподобия.
    uint8_t shifted[sizeof BODY_118 + 1];
    shifted[0] = 0x0e;
    memcpy(shifted + 1, BODY_118, sizeof BODY_118);
    FcMcconfLimits bad = fc_vesc_mcconf_limits(shifted, sizeof shifted);
    check(!bad.valid, "сдвиг раскладки на байт отвергнут, а не принят как 'другие токи'");

    // Чужая сигнатура — отказ целиком.
    uint8_t alien[sizeof BODY_118];
    memcpy(alien, BODY_118, sizeof alien);
    alien[0] = 0xff;
    check(!fc_vesc_mcconf_limits(alien, sizeof alien).valid, "чужая сигнатура отвергнута");

    // Короткий ответ.
    check(!fc_vesc_mcconf_limits(BODY_118, 10).valid, "усечённый ответ отвергнут");
    check(!fc_vesc_mcconf_limits(NULL, 100).valid, "пустой указатель отвергнут");
}

static void test_intersection(void) {
    note("БЕЗОПАСНОЕ ПЕРЕСЕЧЕНИЕ ДВУХ ПОЛОВИН");

    FcMcconfLimits a = {.current_max = 5.0f,
                        .current_min = -3.0f,
                        .in_current_max = 5.0f,
                        .in_current_min = -1.0f,
                        .valid = true};
    FcMcconfLimits b = a;

    FcMcconfLimits same = fc_vesc_mcconf_intersect(&a, &b);
    check(same.valid && near(same.current_max, 5.0f) && near(same.current_min, -3.0f),
          "одинаковые половины дают те же пределы");

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
