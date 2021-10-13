#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "mpu6050.h"
#include "inv_mpu.h"

static const char *TAG = "i2c-mpu6050";

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU6050_SENSOR_ADDR                 0x68        /*!< Slave address of the MPU6050 sensor */
#define MPU6050_WHO_AM_I_REG_ADDR           0x75        /*!< Register addresses of the "who am I" register */

#define MPU6050_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
#define MPU6050_RESET_BIT                   7


/**
 * @brief Read a sequence of bytes from a MPU6050 sensor registers
 */
static esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

/**
 * @brief Write a byte to a MPU6050 sensor register
 */
static esp_err_t mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

uint8_t mpu_write_len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    int ret;
    uint8_t write_buf[64];

    write_buf[0] = reg;
    memcpy(&write_buf[1], buf, len);

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, addr, write_buf, len + 1, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

uint8_t mpu_read_len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, addr, &reg, 1, buf, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

uint8_t mpu6050_delay_ms(uint32_t ms)
{
    esp_rom_delay_us(1000*ms);
    return 0;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


void mpu6050_simple_test(void)
{
    uint8_t data[2];
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    /* Read the MPU6050 WHO_AM_I register, on power up the register should have the value 0x71 */
    ESP_ERROR_CHECK(mpu6050_register_read(MPU6050_WHO_AM_I_REG_ADDR, data, 1));
    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

    /* Demonstrate writing by reseting the MPU6050 */
    ESP_ERROR_CHECK(mpu6050_register_write_byte(MPU6050_PWR_MGMT_1_REG_ADDR, 1 << MPU6050_RESET_BIT));

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C unitialized successfully");
}

uint8_t mpu6050_reset(void)
{
    mpu6050_register_write_byte(MPU_PWR_MGMT1_REG, 0x80); // Reset MPU6050
    esp_rom_delay_us(100*1000);
    mpu6050_register_write_byte(MPU_PWR_MGMT1_REG, 0x00); // Wakeup MPU6050
    return 0;
}

uint8_t mpu6050_set_lpf(uint16_t lpf)
{
    uint8_t data = 0;
    if (lpf >= 188) data = 1;
    else if (lpf >= 98) data = 2;
    else if (lpf >= 42) data = 3;
    else if (lpf >= 20) data = 4;
    else if (lpf >= 10) data = 5;
    else data = 6; 
    return mpu6050_register_write_byte(MPU_CFG_REG, data);
}

uint8_t mpu6050_set_rate(uint16_t rate)
{
    uint8_t data;
    if (rate > 1000) rate = 1000;
    if (rate < 4) rate = 4;
    data = 1000/rate - 1;
    data = mpu6050_register_write_byte(MPU_SAMPLE_RATE_REG, data);
    return mpu6050_set_lpf(rate/2);
}

uint8_t mpu6050_set_gyro_fsr(uint8_t fsr)
{
    return mpu6050_register_write_byte(MPU_GYRO_CFG_REG, fsr<<3);
}

uint8_t mpu6050_set_accel_fsr(uint8_t fsr)
{
    return mpu6050_register_write_byte(MPU_ACCEL_CFG_REG, fsr<<3);
}

uint8_t mpu6050_com_config(void)
{
    mpu6050_register_write_byte(MPU_INT_EN_REG, 0X00); // Disable all interrupt
    mpu6050_register_write_byte(MPU_USER_CTRL_REG, 0X00); //DisableI2C master
    mpu6050_register_write_byte(MPU_FIFO_EN_REG, 0X00); //Disable FIFO
    mpu6050_register_write_byte(MPU_INTBP_CFG_REG, 0X80); //INT low active
    return 0;
}

uint8_t mpu6050_devid_read(uint8_t *devid)
{
    mpu6050_register_read(MPU_DEVICE_ID_REG, devid, 1); //Read device id
    return 0;
}

uint8_t mpu6050_clk_pll_config(void)
{
    mpu6050_register_write_byte(MPU_PWR_MGMT1_REG, 0X01); //set CLKSEL,PLL X as reference
    mpu6050_register_write_byte(MPU_PWR_MGMT2_REG, 0X00); //ACC and GYRO workd
    return 0;
}

short MPU_Get_Temperature(void)
{
    uint8_t buf[2]; 
    short raw;
    float temp;
    mpu_read_len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
    raw = ((uint16_t)buf[0]<<8)|buf[1];
    temp = 36.53+((double)raw)/340;
    return temp*100;;
}

uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    uint8_t buf[6],res;
    res = mpu_read_len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
    if(res == 0)
    {
        *gx = ((uint16_t)buf[0]<<8)|buf[1];
        *gy = ((uint16_t)buf[2]<<8)|buf[3];
        *gz = ((uint16_t)buf[4]<<8)|buf[5];
    }
    return res;
}

uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    uint8_t buf[6], res;
    res=mpu_read_len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
    if(res == 0)
    {
        *ax = ((uint16_t)buf[0]<<8)|buf[1];
        *ay = ((uint16_t)buf[2]<<8)|buf[3];
        *az = ((uint16_t)buf[4]<<8)|buf[5];
    }
    return res;;
}

uint8_t mpu6050_init(void)
{
    uint8_t devid;

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    mpu6050_reset();
    mpu6050_set_gyro_fsr(3);
    mpu6050_set_accel_fsr(0);
    mpu6050_set_rate(50);
    mpu6050_com_config();
    mpu6050_devid_read(&devid);
    if (devid == MPU_ADDR) {
        mpu6050_clk_pll_config();
        mpu6050_set_rate(50);
        ESP_LOGI(TAG, "MPU6050 Init Successed!, DeviceID: %x", devid);
    }

    while(mpu_dmp_init())
    {
        ESP_LOGI(TAG, "MPU6050 Error");
        mpu6050_delay_ms(2000);
    }
    ESP_LOGI(TAG, "MPU6050 OK");
    return 0;
}

void mpu6050_sensor_task(void *pvParameters)
{
    float pitch, roll, yaw;
    short aacx, aacy, aacz;
    short gyrox, gyroy, gyroz;
    short temp;

    mpu6050_init();

    while(1) {
        if (mpu_dmp_get_data(&pitch, &roll, &yaw) == 0) {
            temp = MPU_Get_Temperature();
            MPU_Get_Accelerometer(&aacx, &aacy, &aacz);
            MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
            ESP_LOGI(TAG, "Temp: %d.%d", temp/100, temp%10);
            ESP_LOGI(TAG, "===============================");
            ESP_LOGI(TAG, " %3.2f(P), %3.2f(R), %3.2f(Y)", pitch, roll, yaw);
            ESP_LOGI(TAG, "===============================");
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
    }
}

