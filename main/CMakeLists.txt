set(SOURCES
    "control.c"
    "main.c"
    "soft_ap.c"
    "led_control.c"
    "tcp_server.c"
    "pid_control.c"
    "./mpu6050/mpu6050.c"
    "./mpu6050/inv_mpu.c"
    "./mpu6050/inv_mpu_dmp_motion_driver.c"
    "led_strip_rmt_ws2812.c")

idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS "."
                    INCLUDE_DIRS "./mpu6050"
		    REQUIRES soc driver nvs_flash esp_wifi esp_netif lwip)
