# FloatCore — прошивка ESP32 (ТЗ v0.5).
#
#   cd platform/esp32
#   idf.py set-target esp32
#   idf.py build flash monitor
#
# Второго FloatCore здесь нет: собираются те же самые исходники Refloat
# (refloat-upstream/src) и те же общие слои (compat/*), что и в host-сборке.
# Отличается только реализация платформы — main/fc_*.c.
cmake_minimum_required(VERSION 3.16)

set(FLOATCORE_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
# Каталог генерации общий с host-сборкой (Makefile: GEN := build/gen).
# Это не удобство, а требование: host и плата обязаны собирать один и тот же
# confparser/confxml, иначе сигнатура конфигурации разъедется.
set(FLOATCORE_GEN ${FLOATCORE_ROOT}/build/gen)

# Генерация нужна уже на этапе configure: idf_component_register проверяет,
# что каталоги include существуют.
if(NOT EXISTS ${FLOATCORE_GEN}/conf/confparser.c)
    message(STATUS "FloatCore: генерация conf/ из settings.xml")
    execute_process(
        COMMAND python3 ${FLOATCORE_ROOT}/tools/gen_conf.py
                --settings ${FLOATCORE_ROOT}/refloat-upstream/src/conf/settings.xml
                --out ${FLOATCORE_GEN}
                --version 1.3.0 --package-name Refloat --git-hash esp32
        RESULT_VARIABLE FC_GEN_RC
    )
    if(NOT FC_GEN_RC EQUAL 0)
        message(FATAL_ERROR "gen_conf.py завершился с кодом ${FC_GEN_RC}")
    endif()
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(floatcore_esp32)
