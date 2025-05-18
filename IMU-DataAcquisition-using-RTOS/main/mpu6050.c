#include "mpu6050.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SCL_IO 7
#define I2C_MASTER_SDA_IO 6
#define I2C_MASTER_FREQ_HZ 400000

#define MPU6050_ADDR 0x68
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

static const char *TAG = "MPU6050";

esp_err_t mpu6050_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));

    // OPTIONAL: Reset device (uncomment if needed)
    /*
    uint8_t reset_cmd[2] = {MPU6050_REG_PWR_MGMT_1, 0x80};
    i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, reset_cmd, 2, 1000 / portTICK_PERIOD_MS);
    vTaskDelay(pdMS_TO_TICKS(100));
    */

    // Wake up sensor
    uint8_t wake_cmd[2] = {MPU6050_REG_PWR_MGMT_1, 0x00};
    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, wake_cmd, 2, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Wake-up failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set accelerometer to ±2g
    uint8_t accel_config[2] = {MPU6050_REG_ACCEL_CONFIG, 0x00};
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, accel_config, 2, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Accel config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return ESP_OK;
}

esp_err_t mpu6050_read_raw(mpu6050_raw_data_t *data) {
    uint8_t buffer[14];
    uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;

    esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, &reg, 1, buffer, 14, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensor data: %s", esp_err_to_name(ret));
        return ret;
    }

    //ESP_LOG_BUFFER_HEX_LEVEL("MPU6050_RAW", buffer, 14, ESP_LOG_INFO);

    int16_t ax = (buffer[0] << 8) | buffer[1];
    int16_t ay = (buffer[2] << 8) | buffer[3];
    int16_t az = (buffer[4] << 8) | buffer[5];
    int16_t gx = (buffer[8] << 8) | buffer[9];
    int16_t gy = (buffer[10] << 8) | buffer[11];
    int16_t gz = (buffer[12] << 8) | buffer[13];

    //ESP_LOGI(TAG, "RAW AX: %d AY: %d AZ: %d | GX: %d GY: %d GZ: %d", ax, ay, az, gx, gy, gz);

    data->accel_x = ax / 16384.0f;
    data->accel_y = ay / 16384.0f;
    data->accel_z = az / 16384.0f;
    data->gyro_x = gx / 131.0f;
    data->gyro_y = gy / 131.0f;
    data->gyro_z = gz / 131.0f;

    return ESP_OK;
}
