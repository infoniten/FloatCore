// Учёт процессора без двойного счёта (ТЗ v0.9I §15, §25).

#include "../../compat/diag/fc_cpu_account.h"

#include <stdbool.h>
#include <stdio.h>

void fc_test_check(bool ok, const char *what);
void fc_test_note(const char *fmt, ...);
#define check fc_test_check
#define note fc_test_note

static void test_nested_preemption(void) {
    note("вложенное вытеснение не считается дважды");
    // A (Refloat Main) открыто 0..1000; внутри его вытесняет B (датчик)
    // 200..500, а B, в свою очередь, — C (супервизор) 300..350.
    FcCpuAccount acc = {0};
    FcCpuWindow a, b, c;
    FcCpuSample sa, sb, sc;
    fc_cpu_window_begin(&acc, &a, 0);
    fc_cpu_window_begin(&acc, &b, 200);
    fc_cpu_window_begin(&acc, &c, 300);
    fc_cpu_window_end(&acc, &c, 350, &sc);
    fc_cpu_window_end(&acc, &b, 500, &sb);
    fc_cpu_window_end(&acc, &a, 1000, &sa);

    check(sc.net_us == 50, "C: собственное 50 мкс");
    check(sb.wall_us == 300 && sb.preempt_us == 50 && sb.net_us == 250,
          "B: настенное 300, из них чужое 50, собственное 250");
    check(sa.wall_us == 1000 && sa.preempt_us == 300 && sa.net_us == 700,
          "A: настенное 1000, чужое 300 (B вместе с C), собственное 700");
    check(sa.net_us + sb.net_us + sc.net_us == 1000,
          "сумма собственных = настенному времени ядра: ничего не посчитано дважды");
    check(acc.busy_us == 1000, "накопитель ядра = 1000 мкс, а не 1350");
    check(sa.preempt_us <= sa.wall_us, "вытеснение не превышает окна");
}

static void test_serial_windows(void) {
    note("последовательные окна не влияют друг на друга");
    FcCpuAccount acc = {0};
    FcCpuWindow a, b;
    FcCpuSample sa, sb;
    fc_cpu_window_begin(&acc, &a, 0);
    fc_cpu_window_end(&acc, &a, 400, &sa);
    fc_cpu_window_begin(&acc, &b, 1000);
    fc_cpu_window_end(&acc, &b, 1300, &sb);
    check(sa.preempt_us == 0 && sa.net_us == 400, "первое окно без вытеснения");
    check(sb.preempt_us == 0 && sb.net_us == 300, "второе окно без вытеснения");
}

static void test_closed_twice(void) {
    note("окно нельзя закрыть дважды");
    FcCpuAccount acc = {0};
    FcCpuWindow w;
    fc_cpu_window_begin(&acc, &w, 0);
    check(fc_cpu_window_end(&acc, &w, 100, NULL), "первое закрытие выдаёт отсчёт");
    check(!fc_cpu_window_end(&acc, &w, 200, NULL), "второе отвергнуто");
    check(acc.busy_us == 100, "и в накопитель не попало");
}

void test_cpu_account_all(void) {
    test_nested_preemption();
    test_serial_windows();
    test_closed_twice();
}
