# Платформенный слой FloatCore для ESP32.
# Здесь НЕТ ни одного исходника Refloat: он собирается компонентом refloat,
# у которого другой набор include-путей (см. комментарий в fc_platform.h).
idf_component_register(
    SRCS
        "app_main.c"
        "fc_vesc_if.c"
        "fc_imu_mock.c"
        "fc_imu_rt.c"
        "fc_imu_source.c"
        "fc_imu_cal_store.c"
        "fc_imu_stress.c"
        "fc_adc_safe.c"
        "fc_storage_nvs.c"
        "fc_timing.c"
        "fc_console.c"
        "fc_can_absent.c"
        "fc_can_bus.c"
        "fc_log_port.c"
        "fc_sched.c"
        "fc_supervisor_task.c"
        "fc_safety_report.c"
        "fc_gap_port.c"
        "fc_motor_experiment.c"
        "../drivers/icm20948.c"
        "../../../compat/safety/fc_supervisor.c"
        "../../../compat/safety/fc_motor_gate.c"
        "../../../compat/safety/fc_imu_health.c"
        "../../../compat/safety/fc_imu_policy.c"
        "../../../compat/safety/fc_command_watchdog.c"
        "../../../compat/safety/fc_battery_model.c"
        "../../../compat/safety/fc_motor_permit.c"
        "../../../compat/motor/fc_motor_model.c"
        "../../../compat/config/floatcore_limits.c"
        "../../../compat/imu/fc_ahrs.c"
        "../../../compat/imu/fc_imu_pipeline.c"
        "../../../compat/imu/fc_imu_calibration.c"
        "../../../compat/imu/fc_imu_detect.c"
        "../../../compat/log/fc_log.c"
        "../../../compat/can/fc_vesc_can.c"
        "../../../compat/can/fc_can_diag.c"
        "../../../compat/can/fc_can_health.c"
        "../../../compat/can/fc_vesc_can_motor.c"
        "../../../compat/motor/fc_dual_motor.c"
        "../../../compat/diag/fc_gap_trace.c"
        "../../../compat/diag/fc_shadow.c"
        "../../../compat/vesc_protocol/packet.c"
    INCLUDE_DIRS "."
    PRIV_INCLUDE_DIRS
        "${CMAKE_CURRENT_LIST_DIR}/../../../compat/vesc_api"
        "${CMAKE_CURRENT_LIST_DIR}/../../../compat/safety"
    REQUIRES refloat nvs_flash esp_timer esp_app_format spi_flash esp_hw_support hal soc driver
             esp_system esp_driver_i2c driver
)

# Профиль сборки — свойство двоичного файла, а не настройка во время работы.
#
#   lab_safe      моторного вывода нет как кода. Умолчание.
#   experimental  v0.9A: транспорт команд мотору существует. Собирать
#                 ТОЛЬКО для стенда с вывешенным колесом.
#
# Задаётся при сборке: `idf.py -DFLOATCORE_PROFILE=experimental build`.
#
# Умолчание намеренно лабораторное: сборка, способная крутить мотор, не должна
# получаться случайно — ни забытым флагом, ни чужим кэшем CMake.
if(NOT DEFINED FLOATCORE_PROFILE)
    set(FLOATCORE_PROFILE "lab_safe")
endif()

if(FLOATCORE_PROFILE STREQUAL "lab_safe")
    target_compile_definitions(${COMPONENT_LIB} PUBLIC FLOATCORE_LAB_SAFE=1)
elseif(FLOATCORE_PROFILE STREQUAL "experimental")
    # Второе определение — не удвоение, а подтверждение. Профиль не соберётся
    # без него (fc_build_profile.h), поэтому «случайно собрать моторную
    # прошивку» требует написать вслух, что колесо вывешено.
    target_compile_definitions(${COMPONENT_LIB} PUBLIC
        FLOATCORE_MOTOR_EXPERIMENTAL=1
        FLOATCORE_MOTOR_EXPERIMENTAL_WHEEL_OFF_GROUND=1)
else()
    message(FATAL_ERROR "FLOATCORE_PROFILE: ожидается lab_safe или experimental, получено "
                        "'${FLOATCORE_PROFILE}'")
endif()
message(STATUS "FloatCore: профиль сборки = ${FLOATCORE_PROFILE}")

# Профиль шины CAN — тоже свойство двоичного файла.
#
#   passive      TWAI в listen-only, ни одной функции передачи в сборке нет.
#   active_diag  TWAI в normal mode, разрешены только read-only запросы из
#                белого списка compat/can/fc_can_diag.h (ТЗ v0.7B).
#
# Значение задаётся при сборке: `idf.py -DFLOATCORE_CAN_PROFILE=passive build`.
# Умолчание — active_diag, текущий этап. Ни один из них не совместим с
# транспортом команд мотору: это проверяет fc_build_profile.h.
if(NOT DEFINED FLOATCORE_CAN_PROFILE)
    if(FLOATCORE_PROFILE STREQUAL "experimental")
        set(FLOATCORE_CAN_PROFILE "active_motor")
    else()
        set(FLOATCORE_CAN_PROFILE "active_diag")
    endif()
endif()

if(FLOATCORE_CAN_PROFILE STREQUAL "passive")
    target_compile_definitions(${COMPONENT_LIB} PUBLIC FLOATCORE_CAN_PASSIVE=1)
elseif(FLOATCORE_CAN_PROFILE STREQUAL "active_diag")
    target_compile_definitions(${COMPONENT_LIB} PUBLIC FLOATCORE_CAN_ACTIVE_DIAG=1)
elseif(FLOATCORE_CAN_PROFILE STREQUAL "active_motor")
    target_compile_definitions(${COMPONENT_LIB} PUBLIC FLOATCORE_CAN_ACTIVE_MOTOR=1)
else()
    message(FATAL_ERROR "FLOATCORE_CAN_PROFILE: ожидается passive, active_diag или active_motor, "
                        "получено '${FLOATCORE_CAN_PROFILE}'")
endif()
message(STATUS "FloatCore: профиль CAN = ${FLOATCORE_CAN_PROFILE}")
