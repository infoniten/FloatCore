#pragma once

#include <stdbool.h>
#include <stddef.h>

typedef struct {
    const char *name;
    bool (*run)(void);
} Scenario;

extern const Scenario SCENARIOS[];
extern const size_t SCENARIO_COUNT;

// Мини-фреймворк проверок (реализация в scenarios.c)
void t_reset(void);
size_t t_failures(void);
void t_check(bool ok, const char *fmt, ...);
void t_info(const char *fmt, ...);
