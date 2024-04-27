#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <esp_log.h>

#define UART_NUM UART_NUM_0
#define BUF_SIZE 1024
#define RX_PIN 44
#define TX_PIN 43

void app_main(void)
{
    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122, // 122 bytes
    };

    // Set UART parameters
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TX_PIN, RX_PIN, 18, 19));
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;

    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));


    // // Allocate buffer for received data
    // uint8_t* data = (uint8_t*) malloc(BUF_SIZE);

    while(1) {
        // // Read data from UART with a timeout of 20 milliseconds
        // int len = uart_read_bytes(UART_NUM, data, BUF_SIZE, 1000);
        
        // // Write data back to UART
        // uart_write_bytes(UART_NUM, (const char*) data, len);

        // // esp_log_write
        // esp_log_write(ESP_LOG_ERROR, "UART", "Received %d bytes: %s\n", len, data);

        // vTaskDelay(1000 / portTICK_PERIOD_MS);

        // Read data from UART
        uint8_t data[128];

        int length = 0;

        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM, (size_t*)&length));
        length = uart_read_bytes(   UART_NUM, // UART number
                                    data, // Buffer to store data 
                                    length, // Read all available data 
                                    100 // 100ms timeout
        );

        if (length > 0) {
            char* test = (char*) data;
            ESP_LOGI("UART", "Received %d bytes: %s", length, test);
        }

    }
}