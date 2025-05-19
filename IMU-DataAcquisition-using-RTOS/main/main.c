#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "imu_sampler.h"
#include "imu_bias.h"
#include "mpu6050.h"

#define LED_GPIO 10
#define NUM_AXIS 3

const char* accel_keys[NUM_AXIS] = {"accel_x", "accel_y", "accel_z"};
const char* gyro_keys[NUM_AXIS] = {"gyro_x", "gyro_y", "gyro_z"};
float accel_bias_defaults[NUM_AXIS] = {0.029, 0.054, 1.087};
float gyro_bias_defaults[NUM_AXIS] = {2.785, 3.506, 1.164};

void blink_led(int times, int delay_ms) {
    for (int i = 0; i < times; i++) {
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

esp_err_t load_or_store_bias() {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open("imu_storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) return err;

    bool any_missing = false;

    for (int i = 0; i < NUM_AXIS; i++) {
        size_t length = sizeof(float);
        err = nvs_get_blob(nvs_handle, accel_keys[i], &accel_bias[i], &length);
        if (err != ESP_OK) {
            //ESP_OK("Loaded default Accel bias values");
            accel_bias[i] = accel_bias_defaults[i];
            nvs_set_blob(nvs_handle, accel_keys[i], &accel_bias[i], length);
            any_missing = true;
        }
    }

    for (int i = 0; i < NUM_AXIS; i++) {
        size_t length = sizeof(float);
        err = nvs_get_blob(nvs_handle, gyro_keys[i], &gyro_bias[i], &length);
        if (err != ESP_OK) {
            //ESP_OK("Loaded default Gyro bias values");
            gyro_bias[i] = gyro_bias_defaults[i];
            nvs_set_blob(nvs_handle, gyro_keys[i], &gyro_bias[i], length);
            any_missing = true;
        }
    }

    if (any_missing) {
        err = nvs_commit(nvs_handle);
        if (err != ESP_OK) {
            nvs_close(nvs_handle);
            return err;
        }
    }

    nvs_close(nvs_handle);
    return ESP_OK;
}

void app_main(void) {
    // Configure LED
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    blink_led(3, 200);

    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        blink_led(5, 100); // for failure
        return;
    }

    // Load bias values
    err = load_or_store_bias();
    if (err != ESP_OK) {
        blink_led(5, 100);
        return;
    } else {
        blink_led(3, 200);
    }

    // Initialize MPU6050
    if (mpu6050_init() != ESP_OK) {
        blink_led(5, 100);
        return;
    }

    // Wait 10 seconds before starting
    vTaskDelay(pdMS_TO_TICKS(10000));
    blink_led(1, 500); // Signal sampling start
    imu_sampler_start();
}