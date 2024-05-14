#include "driver/gpio.h"
#include "sensorsData.h"
#include <stdio.h>

#define RELAY_PIN1 GPIO_NUM_5
#define RELAY_PIN2 GPIO_NUM_10
#define RELAY_PIN3 GPIO_NUM_17
#define RELAY_PIN4 GPIO_NUM_18

// drive the logic here
// sensors range from 1300 to 3050, super wet to dry
// if the sensor is below 2000, turn off the relay
// if the sensor is above 2500, turn on the relay
// if the sensor is between 2000 and 2500, do nothing
void control_relay(SensorData *sensor) {
  if (sensor->sensor_id == 1) {
    if (sensor->sensor_value < 800) {
      // turn off the relay
      sensor->sensor_relay = 0;
      printf("Turning off relay\n");
      gpio_set_level(RELAY_PIN1, 0);

    } else if (sensor->sensor_value > 1000) {
      // turn on the relay
      sensor->sensor_relay = 1;
      gpio_set_level(RELAY_PIN1, 1);
    }
  } else if (sensor->sensor_id == 2) {
    if (sensor->sensor_value < 2400) {
      // turn off the relay
      sensor->sensor_relay = 0;
      gpio_set_level(RELAY_PIN2, 0);
    } else if (sensor->sensor_value > 2900) {
      // turn on the relay
      sensor->sensor_relay = 1;
      gpio_set_level(RELAY_PIN2, 1);
    }
  } else if (sensor->sensor_id == 3) {
    if (sensor->sensor_value < 2400) {
      // turn off the relay
      sensor->sensor_relay = 0;
      gpio_set_level(RELAY_PIN3, 0);
    } else if (sensor->sensor_value > 2900) {
      // turn on the relay
      sensor->sensor_relay = 1;
      gpio_set_level(RELAY_PIN3, 1);
    }
  } else if (sensor->sensor_id == 4) {
    if (sensor->sensor_value < 2400) {
      // turn off the relay
      sensor->sensor_relay = 0;
      gpio_set_level(RELAY_PIN4, 0);
    } else if (sensor->sensor_value > 2900) {
      // turn on the relay
      sensor->sensor_relay = 1;
      gpio_set_level(RELAY_PIN4, 1);
    }
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