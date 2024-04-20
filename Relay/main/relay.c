#include <stdio.h>
#include "freertos/FreeRTOS.h" 
#include "freertos/task.h"
#include "driver/gpio.h"
#include <portmacro.h>

#define RELAY_PIN1 GPIO_NUM_8
#define RELAY_PIN2 GPIO_NUM_10
#define RELAY_PIN3 GPIO_NUM_11

void app_main(void)
{
    gpio_reset_pin(RELAY_PIN1);
    gpio_reset_pin(RELAY_PIN2);
    gpio_reset_pin(RELAY_PIN3);
    gpio_set_direction(RELAY_PIN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(RELAY_PIN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(RELAY_PIN3, GPIO_MODE_OUTPUT);

    while (1) {
        printf("Turning on relay\n");
        gpio_set_level(RELAY_PIN1, 1);
        gpio_set_level(RELAY_PIN2, 1);
        gpio_set_level(RELAY_PIN3, 1);
        vTaskDelay(5000 / portTICK_PERIOD_MS);

        printf("Turning off relay\n");
        gpio_set_level(RELAY_PIN1, 0);  
        gpio_set_level(RELAY_PIN2, 0);
        gpio_set_level(RELAY_PIN3, 0);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}