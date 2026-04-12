idf_component_register(
    SRC_DIRS
        "."
        "board"
        "barometer"
        "attitude_bench"
        "params"
        "motor"
        "imu"
        "estimator"
        "controller"
        "mixer"
        "led_status"
        "network"
        "console"
        "safety"
        "udp_manual"
        "udp_protocol"
    INCLUDE_DIRS
        "."
        "board"
        "barometer"
        "attitude_bench"
        "params"
        "motor"
        "imu"
        "estimator"
        "controller"
        "mixer"
        "led_status"
        "network"
        "console"
        "safety"
        "udp_manual"
        "udp_protocol"
    REQUIRES
        nvs_flash
        esp_timer
        esp_adc
        esp_driver_gpio
        esp_driver_uart
        esp_driver_ledc
        esp_vfs_console
        esp_wifi
        esp_netif
        esp_event
        lwip
)

execute_process(
    COMMAND git rev-parse --short=12 HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/..
    OUTPUT_VARIABLE ESP_DRONE_BUILD_GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT ESP_DRONE_BUILD_GIT_HASH)
    set(ESP_DRONE_BUILD_GIT_HASH "unknown")
endif()

string(TIMESTAMP ESP_DRONE_BUILD_TIME_UTC "%Y-%m-%dT%H:%M:%SZ" UTC)

target_compile_definitions(${COMPONENT_LIB} PRIVATE
    ESP_DRONE_BUILD_GIT_HASH="${ESP_DRONE_BUILD_GIT_HASH}"
    ESP_DRONE_BUILD_TIME_UTC="${ESP_DRONE_BUILD_TIME_UTC}"
)
