#include "driver/uart.h"
#include "freertos/queue.h"
#include <string.h>


#define UART_NUM UART_NUM_0
#define BUF_SIZE 1024
#define RX_PIN 44
#define TX_PIN 43

#define MAX_SENSORS 4

typedef struct {
  int sensor_id;
  int sensor_value;
} SensorData;

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

int parse_sensor_data(const char *data, SensorData *sensors, int max_sensors) {
  const char *start = data;
  int count = 0;

  // Skip non-numeric characters to find the first sensor ID
  while (*start && (count < max_sensors)) {
    if (*start == '{' || *start == ',' || *start == ' ') {
      start++;
      continue;
    }

    // Read sensor ID and value
    int sensor_id, sensor_value;
    if (sscanf(start, "%d: %d", &sensor_id, &sensor_value) == 2) {
      sensors[count].sensor_id = sensor_id;
      sensors[count].sensor_value = sensor_value;
      count++;
    }

    // Move to the next part of the string
    while (*start && *start != ',') {
      start++;
    }
  }

  return count; // Return the number of sensors parsed
}

const char *handle_uart_data(const char *uart_data) {
  SensorData sensors[MAX_SENSORS];
  int num_sensors = parse_sensor_data(uart_data, sensors, MAX_SENSORS);

  // Estimate the required buffer size
  // Assuming each sensor takes up to 50 characters, including the newline and
  // null terminator
  int bufferSize = num_sensors * 50;
  char *data = malloc(bufferSize);
  if (data == NULL) {
    return NULL; // Failed to allocate memory
  }

  data[0] = '\0';      // Initialize the string with a null terminator
  char tempBuffer[50]; // Temporary buffer for each sensor's data

  // Format and append each sensor's data to the string
  for (int i = 0; i < num_sensors; i++) {
    sprintf(tempBuffer, "sensors,sensor_id=%d value=%d\n",
            sensors[i].sensor_id, sensors[i].sensor_value);
    strcat(data, tempBuffer);
  }

  return data;
}