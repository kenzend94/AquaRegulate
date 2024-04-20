#include <stdio.h>
#include "freertos/FreeRTOS.h" 
#include "freertos/task.h"
#include "driver/gpio.h"
#include <portmacro.h>

#define RELAY_PIN GPIO_NUM_9



void app_main(void)
{
    gpio_reset_pin(RELAY_PIN);
    gpio_set_direction(RELAY_PIN, GPIO_MODE_OUTPUT);

    while (1) {
        printf("Turning on relay\n");
        gpio_set_level(RELAY_PIN, 1);
        vTaskDelay(5000 / portTICK_PERIOD_MS);

        printf("Turning off relay\n");
        gpio_set_level(RELAY_PIN, 0);  
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}