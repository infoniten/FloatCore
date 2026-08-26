# Платформенный слой FloatCore для ESP32.
# Здесь НЕТ ни одного исходника Refloat: он собирается компонентом refloat,
# у которого другой набор include-путей (см. комментарий в fc_platform.h).
idf_component_register(
    SRCS
        "app_main.c"
        "fc_vesc_if.c"
        "fc_motor_blocked.c"
        "fc_imu_mock.c"
        "fc_adc_safe.c"
        "fc_storage_nvs.c"
        "fc_timing.c"
        "fc_console.c"
    INCLUDE_DIRS "."
    PRIV_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../compat/vesc_api"
    REQUIRES refloat nvs_flash esp_timer esp_app_format spi_flash esp_hw_support hal soc driver esp_system
)
