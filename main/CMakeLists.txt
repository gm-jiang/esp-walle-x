set(SOURCES
    "control.c"
    "main.c"
    "soft_ap.c"
    "tcp_server.c")


idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS "."
                    REQUIRES soc nvs_flash esp_wifi esp_netif lwip)
