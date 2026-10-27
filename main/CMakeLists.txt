idf_component_register(
    SRCS "mqtt_cmd.c" "mqtt_apply.c" "mqtt_fault.c" "mqtt_json.c" "traffic_fault.c" "traffic_config.c" "blink_example_main.c"
         "board_hw.c"
         "mux_task.c"
         "fault_handler.c"
         "node_proto.c"
         "root_wd.c"
         "buttons.c"
         "connectivity.c"
         "i2c_bus.c"
         "pi4ioe5v6408.c"
         "lora.c"
         "mux.c"
         "shift_register.c"
         "wifi_ap.c"
         "config_manager.c"
         "wifi_mqtt.c"
         "mqtt_json.c"
         "mqtt_cmd.c"
         "mqtt_fault.c"
         "mqtt_apply.c"
         "eth_w5500.c"
         "mux_data.c"
         "traffic_light.c"
         "traffic_config.c"
         "traffic_fault.c"         
         "traffic_nvs.c"
         "node_link.c"
         "modem_ppp.c"
         "rs485.c"
         "can_bus.c"
         "time_sync.c"
    INCLUDE_DIRS "." "include"
    REQUIRES
        driver esp_driver_gpio esp_driver_uart esp_driver_spi
        esp_driver_i2c esp_driver_twai esp_eth esp_netif esp_timer
        esp_adc esp_wifi esp_event esp_http_server lwip nvs_flash
        espressif__cjson w5500 espressif__esp_modem
)
target_add_binary_data(${COMPONENT_LIB} "config.json" TEXT)