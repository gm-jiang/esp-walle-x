/* GPIO Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"

#include "control.h"


#define GPIO_OUTPUT_IO_M0    4  //left
#define GPIO_OUTPUT_IO_M1    5

#define GPIO_OUTPUT_IO_M2    6  //right
#define GPIO_OUTPUT_IO_M3    7

#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_M0) | (1ULL<<GPIO_OUTPUT_IO_M1) | \
                              (1ULL<<GPIO_OUTPUT_IO_M2) | (1ULL<<GPIO_OUTPUT_IO_M3) )


WALLEClient_t WALLEClient;

static uint8_t controler_gpio_init(void);
static uint8_t controler_act(uint8_t act, uint32_t run_time);
static uint8_t controler_spd(speed_e spd, uint32_t param);
void test_function(void);

void WALLE_Init(WALLEClient_t *thiz)
{
    thiz->devid = 123;
    thiz->name = "WALL-E";
    thiz->spd_status = FULL_SPEED;
    thiz->walle_init_drv_cb = controler_gpio_init;
    thiz->walle_init_pwm_cb = NULL;
    thiz->walle_control_act_cb = controler_act;
    thiz->walle_control_spd_cb = controler_spd;
}

static uint8_t controler_gpio_init(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 1;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_set_level(GPIO_OUTPUT_IO_M0, 0);
    gpio_set_level(GPIO_OUTPUT_IO_M1, 0);

    gpio_set_level(GPIO_OUTPUT_IO_M2, 0);
    gpio_set_level(GPIO_OUTPUT_IO_M3, 0);

    return 0;
}

static void go_forward(uint32_t ms)
{
    gpio_set_level(GPIO_OUTPUT_IO_M0, 1);
    gpio_set_level(GPIO_OUTPUT_IO_M1, 0);

    gpio_set_level(GPIO_OUTPUT_IO_M2, 1);
    gpio_set_level(GPIO_OUTPUT_IO_M3, 0);
}

static void go_back(uint32_t ms)
{
    gpio_set_level(GPIO_OUTPUT_IO_M0, 0);
    gpio_set_level(GPIO_OUTPUT_IO_M1, 1);

    gpio_set_level(GPIO_OUTPUT_IO_M2, 0);
    gpio_set_level(GPIO_OUTPUT_IO_M3, 1);
}

static void stop(void)
{
    gpio_set_level(GPIO_OUTPUT_IO_M0, 0);
    gpio_set_level(GPIO_OUTPUT_IO_M1, 0);

    gpio_set_level(GPIO_OUTPUT_IO_M2, 0);
    gpio_set_level(GPIO_OUTPUT_IO_M3, 0);
}

static void turn_right(uint32_t ms)
{
    //stop first
    gpio_set_level(GPIO_OUTPUT_IO_M0, 0);
    gpio_set_level(GPIO_OUTPUT_IO_M1, 0);

    gpio_set_level(GPIO_OUTPUT_IO_M2, 0);
    gpio_set_level(GPIO_OUTPUT_IO_M3, 0);

    vTaskDelay(100 / portTICK_PERIOD_MS);

    //turn right
    gpio_set_level(GPIO_OUTPUT_IO_M0, 1);
    gpio_set_level(GPIO_OUTPUT_IO_M1, 0);
    gpio_set_level(GPIO_OUTPUT_IO_M2, 0);
    gpio_set_level(GPIO_OUTPUT_IO_M3, 1);

    vTaskDelay(ms / portTICK_PERIOD_MS);

    //stop end
    gpio_set_level(GPIO_OUTPUT_IO_M0, 0);
    gpio_set_level(GPIO_OUTPUT_IO_M1, 0);
    gpio_set_level(GPIO_OUTPUT_IO_M2, 0);
    gpio_set_level(GPIO_OUTPUT_IO_M3, 0);
}

static void turn_left(uint32_t ms)
{
    //stop first
    gpio_set_level(GPIO_OUTPUT_IO_M0, 0);
    gpio_set_level(GPIO_OUTPUT_IO_M1, 0);

    gpio_set_level(GPIO_OUTPUT_IO_M2, 0);
    gpio_set_level(GPIO_OUTPUT_IO_M3, 0);

    vTaskDelay(100 / portTICK_PERIOD_MS);

    //turn right
    gpio_set_level(GPIO_OUTPUT_IO_M0, 0);
    gpio_set_level(GPIO_OUTPUT_IO_M1, 1);
    gpio_set_level(GPIO_OUTPUT_IO_M2, 1);
    gpio_set_level(GPIO_OUTPUT_IO_M3, 0);

    vTaskDelay(ms / portTICK_PERIOD_MS);

    //stop end
    gpio_set_level(GPIO_OUTPUT_IO_M0, 0);
    gpio_set_level(GPIO_OUTPUT_IO_M1, 0);
    gpio_set_level(GPIO_OUTPUT_IO_M2, 0);
    gpio_set_level(GPIO_OUTPUT_IO_M3, 0);
}

static void around_r(void)
{
    //stop first
    gpio_set_level(GPIO_OUTPUT_IO_M0, 0);
    gpio_set_level(GPIO_OUTPUT_IO_M1, 0);

    gpio_set_level(GPIO_OUTPUT_IO_M2, 0);
    gpio_set_level(GPIO_OUTPUT_IO_M3, 0);

    vTaskDelay(100 / portTICK_PERIOD_MS);

    //turn right
    gpio_set_level(GPIO_OUTPUT_IO_M0, 0);
    gpio_set_level(GPIO_OUTPUT_IO_M1, 1);
    gpio_set_level(GPIO_OUTPUT_IO_M2, 1);
    gpio_set_level(GPIO_OUTPUT_IO_M3, 0);
}

static void around_l(void)
{
    //stop first
    gpio_set_level(GPIO_OUTPUT_IO_M0, 0);
    gpio_set_level(GPIO_OUTPUT_IO_M1, 0);

    gpio_set_level(GPIO_OUTPUT_IO_M2, 0);
    gpio_set_level(GPIO_OUTPUT_IO_M3, 0);

    vTaskDelay(100 / portTICK_PERIOD_MS);

    //turn right
    gpio_set_level(GPIO_OUTPUT_IO_M0, 1);
    gpio_set_level(GPIO_OUTPUT_IO_M1, 0);
    gpio_set_level(GPIO_OUTPUT_IO_M2, 0);
    gpio_set_level(GPIO_OUTPUT_IO_M3, 1);
}


void test_function(void)
{
    gpio_set_level(GPIO_OUTPUT_IO_M0, 1);
    gpio_set_level(GPIO_OUTPUT_IO_M1, 0);
}

#if 0
static uint8_t controler(action_e act, speed_e spd, uint32_t run_time)
{
    switch (act)
    {
        case FORWARD:
            go_forward(run_time);
            break;
        case BACK:
            go_back(run_time);
            break;
        case STOP:
            stop();
            break;
        case RIGHT:
            turn_right(run_time);
            break;
        case LEFT:
            turn_left(run_time);
            break;
        case AROUND_R:
            around_r()
            break;
        case AROUND_L:
            around_l()
            break;
        default:
            stop();
            break;
    }
    return 0;
}
#endif

static uint8_t controler_spd(speed_e spd, uint32_t run_time)
{
    switch (spd)
    {
    case FULL_SPEED:
        //brushed_motor_high_speed();
        break;
    case LOW_SPEED:
        //brushed_motor_low_speed();
        break;
    default:
        //brushed_motor_high_speed();
        break;
    }
    return 0;
}

static uint8_t controler_act(uint8_t act, uint32_t run_time)
{

    switch (act)
    {
        case 'w':
            go_forward(run_time);
            break;
        case 'b':
            go_back(run_time);
            break;
        case 's':
            stop();
            break;
        case 'd':
            turn_right(run_time);
            break;
        case 'a':
            turn_left(run_time);
            break;
        case 'q':
            around_r();
            break;
        case 'e':
            around_l();
            break;
        default:
            stop();
            break;
    }
    return 0;
}

