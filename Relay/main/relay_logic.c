#include "driver/gpio.h"
#include <stdio.h>

#define RELAY_PIN1 GPIO_NUM_5
#define RELAY_PIN2 GPIO_NUM_10
// #define RELAY_PIN3 GPIO_NUM_17

// drive the logic here
// sensors range from 1300 to 3050, super wet to dry
// if the sensor is below 2000, turn off the relay
// if the sensor is above 2500, turn on the relay
// if the sensor is between 2000 and 2500, do nothing
void control_relay(int sensor_value) {
  if (sensor_value < 2400) {

    printf("Turning off relay\n");
    gpio_set_level(RELAY_PIN1, 0);
    gpio_set_level(RELAY_PIN2, 0);
    // gpio_set_level(RELAY_PIN3, 0);

  } else if (sensor_value > 2900) {
    printf("Turning on relay\n");
    gpio_set_level(RELAY_PIN1, 1);
    gpio_set_level(RELAY_PIN2, 1);
    // gpio_set_level(RELAY_PIN3, 1);
  }
}

void init_relay() {
  // RELAY INITIALIZATION
  gpio_reset_pin(RELAY_PIN1);
  gpio_reset_pin(RELAY_PIN2);
  // gpio_reset_pin(RELAY_PIN3);
  gpio_set_direction(RELAY_PIN1, GPIO_MODE_OUTPUT);
  gpio_set_direction(RELAY_PIN2, GPIO_MODE_OUTPUT);
  // gpio_set_direction(RELAY_PIN3, GPIO_MODE_OUTPUT);
}
