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
        "fc_imu_stress.c"
        "fc_adc_safe.c"
        "fc_storage_nvs.c"
        "fc_timing.c"
        "fc_console.c"
        "fc_can_absent.c"
        "fc_supervisor_task.c"
        "fc_safety_report.c"
        "../drivers/icm20948.c"
        "../../../compat/safety/fc_supervisor.c"
        "../../../compat/safety/fc_motor_gate.c"
        "../../../compat/safety/fc_imu_health.c"
        "../../../compat/imu/fc_ahrs.c"
        "../../../compat/imu/fc_imu_pipeline.c"
    INCLUDE_DIRS "."
    PRIV_INCLUDE_DIRS
        "${CMAKE_CURRENT_LIST_DIR}/../../../compat/vesc_api"
        "${CMAKE_CURRENT_LIST_DIR}/../../../compat/safety"
    REQUIRES refloat nvs_flash esp_timer esp_app_format spi_flash esp_hw_support hal soc driver
             esp_system esp_driver_i2c
)

# Профиль сборки — свойство двоичного файла, а не настройка во время работы.
# На этапах v0.5–v0.6x собирается только лабораторный профиль.
target_compile_definitions(${COMPONENT_LIB} PUBLIC FLOATCORE_LAB_SAFE=1)
