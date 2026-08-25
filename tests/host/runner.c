// Запуск всех host-сценариев.

#include "mock_vesc_if.h"
#include "scenarios.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static bool log_enabled = false;

static void log_sink(const char *fmt, va_list ap) {
    if (log_enabled) {
        printf("        \033[90m[refloat] ");
        vprintf(fmt, ap);
        printf("\033[0m");
    }
}

int main(int argc, char **argv) {
    const char *filter = NULL;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-v") == 0) {
            log_enabled = true;
        } else {
            filter = argv[i];
        }
    }

    setvbuf(stdout, NULL, _IONBF, 0);
    printf("\nRefloat host test harness — mock VESC platform, без вывода на CAN\n");
    printf("================================================================\n");

    size_t total_failures = 0, run = 0;
    for (size_t i = 0; i < SCENARIO_COUNT; ++i) {
        if (filter && !strstr(SCENARIOS[i].name, filter)) {
            continue;
        }
        printf("\n  \033[1m%s\033[0m\n", SCENARIOS[i].name);
        t_reset();
        mock_set_log_sink(log_sink);
        bool ok = SCENARIOS[i].run();
        if (!ok) {
            printf("      \033[31mСЦЕНАРИЙ НЕ ЗАПУСТИЛСЯ\033[0m\n");
            ++total_failures;
        }
        total_failures += t_failures();
        ++run;
    }

    printf("\n================================================================\n");
    if (total_failures == 0) {
        printf("\033[32mВсе проверки пройдены\033[0m (%zu сценариев)\n\n", run);
        return 0;
    }
    printf("\033[31mПровалено проверок: %zu\033[0m (%zu сценариев)\n\n", total_failures, run);
    return 1;
}
