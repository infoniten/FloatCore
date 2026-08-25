// Shim-версия vesc_c_if.h для host-сборки Refloat.
//
// Ключевая идея Strategy B: исходники Refloat не меняются, меняется только этот
// заголовок. Все объявления типов и сигнатур берутся из upstream-заголовка SDK VESC,
// поэтому расхождение ABI невозможно. Переопределяются лишь платформенные макросы:
//
//   VESC_IF     — фиксированный адрес 0x1000F800  →  наш глобальный указатель
//   HEADER      — секция .program_ptr             →  обычная переменная
//   INIT_FUN    — секция .init_fun, символ `init` →  обычная функция refloat_init
//   PROG_ADDR   — база загруженного модуля        →  0
//
// ARG не переопределяется: он выражен через VESC_IF->get_arg(), который мы реализуем.
#pragma once

#define IS_VESC_LIB
#include "../../../refloat-upstream/vesc_pkg_lib/vesc_c_if.h"

#undef VESC_IF
extern vesc_c_if *mock_vesc_if;
#define VESC_IF mock_vesc_if

#undef HEADER
#define HEADER volatile int prog_ptr;

#undef INIT_START
#define INIT_START (void) prog_ptr;

#undef INIT_FUN
#define INIT_FUN bool refloat_init

#undef PROG_ADDR
#define PROG_ADDR ((uint32_t) 0)
