# Компонент «Refloat»: неизменённые исходники upstream + общие слои FloatCore.
#
# Выделен в отдельный компонент не ради красоты, а по необходимости: ему нужны
# include-пути refloat-upstream/src, где лежит собственный time.h с
# `typedef uint32_t time_t`. Попади этот каталог в пути платформенных файлов —
# заголовки ESP-IDF, включающие <time.h>, подхватили бы его. Ровно та же
# изоляция, что в host-сборке (см. Makefile, PLAIN_CFLAGS vs REFLOAT_INC).
set(FC_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../../..)
set(FC_RSRC ${FC_ROOT}/refloat-upstream/src)
set(FC_GEN ${FC_ROOT}/build/gen)

file(GLOB FC_REFLOAT_SRC ${FC_RSRC}/*.c ${FC_RSRC}/filters/*.c ${FC_RSRC}/lib/*.c)
# led_driver.c — единственный непортируемый файл (регистры STM32),
# заменяется на fc_led_driver.c.
list(REMOVE_ITEM FC_REFLOAT_SRC ${FC_RSRC}/led_driver.c)

idf_component_register(
    SRCS
        ${FC_REFLOAT_SRC}
        ${FC_RSRC}/conf/buffer.c
        ${FC_GEN}/conf/confparser.c
        ${FC_GEN}/conf/confxml.c
        ${FC_ROOT}/compat/refloat_glue/refloat_facade.c
        ${FC_ROOT}/compat/config/floatcore_limits.c
        ${CMAKE_CURRENT_LIST_DIR}/../../main/fc_led_driver.c
    INCLUDE_DIRS
        ${FC_ROOT}/compat/refloat_glue
    PRIV_INCLUDE_DIRS
        ${FC_ROOT}/compat/vesc_api
        ${FC_RSRC}
        ${FC_RSRC}/conf
        ${FC_GEN}
)

# Те же послабления, что и на host: Refloat собирается как есть, без правок.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-parameter -include stddef.h)
