#include "driver/uart.h"
#include "freertos/queue.h"
#include "relay_logic.h"
#include "sensorsData.h"
#include <string.h>

#define UART_NUM UART_NUM_0
#define BUF_SIZE 1024
#define RX_PIN 44
#define TX_PIN 43

void init_uart() {
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
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE,
                               UART_PIN_NO_CHANGE));
  const int uart_buffer_size = (1024 * 2);
  QueueHandle_t uart_queue;

  // Install UART driver using an event queue here
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM, uart_buffer_size,
                                      uart_buffer_size, 10, &uart_queue, 0));
}

// Parse the sensor data and determine the relay status
int parse_sensor_data(const char *data, SensorData *sensors, int max_sensors) {
  const char *start = data;
  int count = 0;

  // Parse the sensor data
  while (*start && (count < max_sensors)) {
    if (*start == '{' || *start == ',' || *start == ' ') {
      start++;
      continue;
    }

    int sensor_id, sensor_value;

    // Parse the sensor ID and value
    if (sscanf(start, "%d: %d", &sensor_id, &sensor_value) == 2) {
      sensors[count].sensor_id = sensor_id;
      sensors[count].sensor_value = sensor_value;
      // Determine relay status based on sensor ID and value
      // if (sensor_id == 1) {
      //   sensors[count].sensor_relay = (sensor_value < 800) ? 0
      //                                 : (sensor_value > 1000)
      //                                     ? 1
      //                                     : sensors[count].sensor_relay;
      // } else if (sensor_id == 2 || sensor_id == 3 || sensor_id == 4) {
      //   sensors[count].sensor_relay = (sensor_value < 2400) ? 0
      //                                 : (sensor_value > 2900)
      //                                     ? 1
      //                                     : sensors[count].sensor_relay;
      // }

      // call the control_relay function to control the relay
      control_relay(&sensors[count]);
      count++;
    }

    while (*start && *start != ',') {
      start++;
    }
  }

  return count;
}
const char *handle_uart_data(const char *uart_data) {
  SensorData sensors[MAX_SENSORS];
  int num_sensors = parse_sensor_data(uart_data, sensors, MAX_SENSORS);

  int bufferSize = num_sensors * 60; // Adjusted buffer size for relay status
  char *data = malloc(bufferSize);
  if (data == NULL) {
    return NULL;
  }

  data[0] = '\0';
  char tempBuffer[60]; // Adjusted buffer size for relay status

  for (int i = 0; i < num_sensors; i++) {
    sprintf(tempBuffer, "sensors_data,sensor_id=%d value=%d,relay_status=%d\n",
            sensors[i].sensor_id, sensors[i].sensor_value,
            sensors[i].sensor_relay);
    strcat(data, tempBuffer);
  }

  return data;
}
