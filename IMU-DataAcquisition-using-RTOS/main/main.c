#include <stdio.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>

// constants
#define LED_GPIO 10
#define NUM_AXIS 3

// Bias keys for each axis 
const char* accel_keys[NUM_AXIS] = {"accel_x", "accel_y", "accel_z"};
const char* gyro_keys[NUM_AXIS] = {"gyro_x", "gyro_y", "gyro_z"};

// Default Bias Values for each axis {pre calculated bias values}
float accel_bias_deafults[NUM_AXIS] = {0.020, 0.026, 1.031};
float gyro_bias_defaults[NUM_AXIS] = {2.015, 2.952, 0.256};

// these will hold loaded values
float accel_bias[NUM_AXIS];
float gyro_bias[NUM_AXIS];

void blink_led(int times, int delay_ms) {
    for (int i = 0; i < times; i++) {
        gpio_set_level(LED_GPIO, 1); // Turn on the LED
        vTaskDelay(pdMS_TO_TICKS(delay_ms)); // Delay for the specified time
        gpio_set_level(LED_GPIO, 0); // Turn off the LED
        vTaskDelay(pdMS_TO_TICKS(delay_ms)); // Delay for the specified time
    }
}

// Function to load or store bias values
esp_err_t load_or_store_bias()
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Open NVS
    err = nvs_open("imu_storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) return err;

    bool any_missing = false; // Flag to check if any bias value is missing

    // Load or store each accelerometer bias
    for (int i = 0; i < NUM_AXIS; i++) {
        err = nvs_get_blob(nvs_handle, accel_keys[i], &accel_bias[i], sizeof(float));
        if (err != ESP_OK) {
            accel_bias[i] = accel_bias_deafults[i]; // Use default value
            nvs_get_blob(nvs_handle, accel_keys[i], &accel_bias[i], sizeof(float));
            any_missing = true; // Mark that a value was missing
        }
    }

    // Load or store each gyroscope bias
    for (int i = 0; i < NUM_AXIS; i++) {
        err = nvs_get_blob(nvs_handle, gyro_keys[i], &gyro_bias[i], sizeof(float));
        if (err != ESP_OK) {
            gyro_bias[i] = gyro_bias_defaults[i]; // Use default value
            nvs_get_blob(nvs_handle, gyro_keys[i], &gyro_bias[i], sizeof(float));
            any_missing = true; // Mark that a value was missing
        }
    }
}

void app_main(void)
{

}