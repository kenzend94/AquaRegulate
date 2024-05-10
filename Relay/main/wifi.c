#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "wifi_cred.h"
#include <stdint.h>

// event group to handle Wi-Fi events
static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

static const char *TAG = "wifi_station";
static const char *TAG_NVS = "NVS_INIT";

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    esp_wifi_connect();
    xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

// Initialize Wi-Fi as sta and set hostname
void wifi_init_sta(void) {

  // Initialize the event loop
  wifi_event_group = xEventGroupCreate();

  // Initialize the underlying TCP/IP stack
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default()); // Create default event loop
  esp_netif_t *my_sta =
      esp_netif_create_default_wifi_sta(); // Create a default Wi-Fi station

  // Set the hostname to "AquaRegulate"
  esp_netif_set_hostname(my_sta, MY_WIFI_HOSTNAME);

  // Initialize the Wi-Fi stack in station mode with config
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg)); // Initialize Wi-Fi with default config

  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                             &event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                             &event_handler, NULL));

  wifi_config_t wifi_config = {
      .sta =
          {
              .ssid = MY_WIFI_SSID,
              .password = MY_WIFI_PASSWORD,
          },
  };

  ESP_ERROR_CHECK(
      esp_wifi_set_mode(WIFI_MODE_STA)); // Set Wi-Fi mode to station
  ESP_ERROR_CHECK(esp_wifi_set_config(
      ESP_IF_WIFI_STA, &wifi_config)); // Set Wi-Fi configuration for station
  ESP_ERROR_CHECK(esp_wifi_start());   // Start Wi-Fi

  ESP_LOGI(
      TAG,
      "wifi_init_sta finished."); // Log that Wi-Fi initialization is finished
  ESP_LOGI(TAG, "connect to ap SSID:%s password:%s", MY_WIFI_SSID,
           MY_WIFI_PASSWORD);

  // Wait for Wi-Fi connection
  EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                                         pdFALSE, pdTRUE, portMAX_DELAY);
  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", MY_WIFI_SSID,
             MY_WIFI_PASSWORD);
  } else {
    ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s", MY_WIFI_SSID,
             MY_WIFI_PASSWORD);
  }
}

void init_NVS(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGI(TAG_NVS, "Erasing NVS flash due to initialization error.");
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
}