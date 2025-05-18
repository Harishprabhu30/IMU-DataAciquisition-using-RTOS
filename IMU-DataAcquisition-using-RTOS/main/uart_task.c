#include "uart_task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#define UART_PORT_NUM      UART_NUM_0
#define UART_BAUD_RATE    115200
#define UART_BUF_SIZE     1024

extern EventGroupHandle_t sampling_event_group; // Defined in main.c
#define START_SAMPLING_BIT (1 << 0)

static const char *TAG = "UART_TASK";

static void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    esp_err_t err = uart_param_config(UART_PORT_NUM, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART config failed: %s", esp_err_to_name(err));
        return;
    }
    err = uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
    }
}

static void uart_task(void *arg) {
    uart_init();
    ESP_LOGI(TAG, "UART task started");

    uint8_t rx_data[128];
    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, rx_data, sizeof(rx_data) - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            rx_data[len] = '\0';
            ESP_LOGI(TAG, "Received command: %s", (char*)rx_data);

            if (strncmp((char*)rx_data, "START", 5) == 0 || strncmp((char*)rx_data, "start", 5) == 0) {
                ESP_LOGI(TAG, "Triggering sampling");
                xEventGroupSetBits(sampling_event_group, START_SAMPLING_BIT);
                const char *msg = "Starting sampling session\n";
                uart_write_bytes(UART_PORT_NUM, msg, strlen(msg));
            } else {
                const char *msg = "Unknown command. Use START to begin sampling.\n";
                uart_write_bytes(UART_PORT_NUM, msg, strlen(msg));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void uart_task_start(void) {
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 5, NULL);
}