set(SOURCES
    "control.c"
    "main.c"
    "soft_ap.c"
    "led_control.c"
    "tcp_server.c"
    "pid_control.c"
    "led_strip_rmt_ws2812.c")

idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS "."
		    REQUIRES soc driver nvs_flash esp_wifi esp_netif lwip)
