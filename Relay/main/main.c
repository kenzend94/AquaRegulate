#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"

#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"
// #include <stdlib.h>
// #include <string.h>
#include "influxDB.c"
#include "relay_logic.c"
#include "time.c"
#include "uart.c"
#include "wifi.c"


void app_main(void) {

  // WIFI INITIALIZATION
  init_NVS();

  ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
  wifi_init_sta();

  // Initialize SNTP for time synchronization
  initialize_sntp();
  wait_for_time_sync(); // Wait until time is synchronized

  // RELAY INITIALIZATION
  init_relay();

  // initialize UART
  init_uart();

  while (1) {
    // Read data from UART
    uint8_t data[128];

    int length = 0;

    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM, (size_t *)&length));
    length = uart_read_bytes(UART_NUM, // UART number
                             data,     // Buffer to store data
                             length,   // Read all available data
                             10        // 50ms timeout
    );

    if (length > 0) {
      char *value = (char *)data;
      ESP_LOGI("UART", "Received %d bytes: %s", length, value);

      // Decode the values {1: 1149, 2: 3077, 3: 3016, 4: 3851} into an array
      const char *formattedData = handle_uart_data(value);

      // const char *data = "sensors, sensor_id=1 value=8000\nsensors,
      // sensor_id=2 value=9000\nsensors, sensor_id=3 value=1000\nsensors,
      // sensor_id=4 value=1050\n";

      if (formattedData != NULL) {
        send_to_influxdb(formattedData);

        printf("%s", formattedData);
        free((void *)formattedData);
      }

      // wait for 5 seconds
      vTaskDelay(5000 / portTICK_PERIOD_MS);

      // // if length greater than 3, then do the logic
      // int sensor_value = atoi(value);
      // control_relay(sensor_value);
    }
  }
}