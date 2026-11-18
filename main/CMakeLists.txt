idf_component_register(
    SRCS
        "mqtt_cmd.c"
        "mqtt_apply.c"
        "mqtt_fault.c"
        "mqtt_json.c"
        "traffic_fault.c"
        "blink_example_main.c"
        "board_hw.c"
        "mux_task.c"
        "fault_handler.c"
        "node_proto.c"
        "root_wd.c"
        "buttons.c"
        "connectivity.c"
        "i2c_bus.c"
        "pi4ioe5v6408.c"
        "lora.cpp"
        "transport.c"
        "mux.c"
        "shift_register.c"
        "config_manager.c"
        "wifi_ap.c"
        "wifi_mqtt.c"
        "mux_data.c"
        "traffic_config.c"
        "traffic_light.c"
        "traffic_nvs.c"
        "modem_ppp.c"
        "rs485.c"
        "can_bus.c"
        "time_sync.c"
        "lora_nvs.c"
        "sync_handshake.c"
        "sync_config.c"
        # "eth_w5500.c"
    INCLUDE_DIRS "." "include"
    REQUIRES
        driver esp_driver_gpio esp_driver_uart esp_driver_spi freertos
        esp_driver_i2c esp_driver_twai esp_eth esp_netif esp_timer
        esp_adc esp_wifi esp_event esp_http_server lwip nvs_flash
        espressif__esp_modem
        espp__sx126x
        cjson esp-tls
)

if(CONFIG_BT_ENABLED)
    target_sources(${COMPONENT_LIB} PRIVATE
        "ble_transport.c"
        "ble_config.c"
    )
    target_link_libraries(${COMPONENT_LIB} PRIVATE bt)
    message(STATUS "BLE: ble_transport.c + ble_config.c включены")
else()
    message(WARNING "BLE disabled (CONFIG_BT_ENABLED=n): ble_transport.c/ble_config.c НЕ собираются")
endif()

target_add_binary_data(${COMPONENT_LIB} "config.json" TEXT)