/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "control.h"
#include "driver/gpio.h"
#include "soft_ap.h"
#include "tcp_server.h"

#include "sdkconfig.h"
#include "led_strip.h"
#include "pid_control.h"
#include "mpu6050.h"

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

static const char *TAG = "ESP-WALLE";

static led_strip_t *pStrip_a;

static void blink_led(uint8_t r_value, uint8_t g_value)
{
    /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
    pStrip_a->set_pixel(pStrip_a, 0, r_value, g_value, 0);
    /* Refresh the strip to send data */
    pStrip_a->refresh(pStrip_a, 100);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    pStrip_a = led_strip_init(CONFIG_BLINK_LED_RMT_CHANNEL, BLINK_GPIO, 1);
    /* Set all LED off to clear all pixels */
    pStrip_a->clear(pStrip_a, 50);
}

extern QueueHandle_t MPU6050_Q;
static void ledc_blink_task(void *pvParameters)
{
    /* Configure the peripheral according to the LED type */
    configure_led();

    while (1) {
        mpu6050_data_t pxRxedMessage;
        int8_t red_data, green_data;
        if (MPU6050_Q != NULL) {
            if (xQueueReceive(MPU6050_Q, &(pxRxedMessage), (portTickType)portMAX_DELAY)) {
                red_data = (int8_t)pxRxedMessage.pitch;
                red_data = red_data < 0 ? (-1*red_data) : red_data;

                green_data = (int8_t)pxRxedMessage.roll;
                green_data = green_data < 0 ? (-1*green_data) : green_data;

                ESP_LOGI(TAG, "%d, %d", red_data, green_data);
                blink_led(red_data, green_data);
            }
        }
        vTaskDelay(1/portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    WALLE_Init(&WALLEClient);
    WALLEClient.walle_init_drv_cb();
    //WALLEClient.walle_init_pwm_cb();
    ESP_LOGI(TAG, "name: %s devid: %d", WALLEClient.name, WALLEClient.devid);

    ESP_ERROR_CHECK(nvs_flash_init());

    //setup a soft ap
    wifi_init_softap();

    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
    xTaskCreate(ledc_blink_task, "led_blink", 4096, NULL, 4, NULL);
    xTaskCreate(mpu6050_sensor_task, "mpu6050", 4096, NULL, 4, NULL);
}
