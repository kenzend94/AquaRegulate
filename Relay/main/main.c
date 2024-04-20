#include <stdio.h>
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
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);

    // Set UART pins
    uart_set_pin(UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Install UART driver
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Allocate buffer for received data
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE);

    while(1) {
        // Read data from UART with a timeout of 20 milliseconds
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE, 20);
        
        // Write data back to UART
        uart_write_bytes(UART_NUM, (const char*) data, len);

        // esp_log_write
        esp_log_write(ESP_LOG_ERROR, "UART", "Received %d bytes: %s\n", len, data);

        // export esp_log_write to .txt file
        
    }
}