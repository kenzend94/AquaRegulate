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

#define RELAY_PIN1 GPIO_NUM_8
#define RELAY_PIN2 GPIO_NUM_10
#define RELAY_PIN3 GPIO_NUM_17

void app_main(void)
{
    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122, // 122 bytes
    };

    // Set UART parameters
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TX_PIN, RX_PIN, 17, 18));
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;

    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));

    // // Allocate buffer for received data
    // uint8_t* data = (uint8_t*) malloc(BUF_SIZE);

    while (1)
    {
        // Read data from UART
        uint8_t data[128];

        int length = 0;

        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM, (size_t *)&length));
        length = uart_read_bytes(UART_NUM, // UART number
                                 data,     // Buffer to store data
                                 length,   // Read all available data
                                 10        // 50ms timeout
        );



        // drive the logic here
        // sensors range from 1300 to 3050, super wet to dry
        // if the sensor is below 2000, turn off the relay
        // if the sensor is above 2500, turn on the relay
        // if the sensor is between 2000 and 2500, do nothing
        if (length > 0)
        {
            char *value = (char *)data;
            ESP_LOGI("UART", "Received %d bytes: %s", length, value);

            int sensor_value = atoi(value);
            if (sensor_value < 2000)
            {
                printf("Turning off relay\n");
                gpio_set_level(RELAY_PIN1, 0);
                gpio_set_level(RELAY_PIN2, 0);
                gpio_set_level(RELAY_PIN3, 0);
            }
            else if (sensor_value > 2500)
            {
                printf("Turning on relay\n");
                gpio_set_level(RELAY_PIN1, 1);
                gpio_set_level(RELAY_PIN2, 1);
                gpio_set_level(RELAY_PIN3, 1);
            }
        }
    }
}