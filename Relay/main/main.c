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

#define RELAY_PIN1 GPIO_NUM_5
#define RELAY_PIN2 GPIO_NUM_10
// #define RELAY_PIN3 GPIO_NUM_17

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/inet.h"


// header file with Wi-Fi credentials
#include "wifi_cred.h"

// event group to handle Wi-Fi events
static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

static const char *TAG = "wifi_station";

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *my_sta = esp_netif_create_default_wifi_sta();

    // Set the hostname to "AquaRegulate"
    esp_netif_set_hostname(my_sta, MY_WIFI_HOSTNAME);

    // Set static IP
    esp_netif_ip_info_t ipInfo;
    ipInfo.ip.addr = ipaddr_addr("192.168.115.191");
    ipInfo.netmask.addr = ipaddr_addr("255.255.255.0");
    esp_netif_dhcpc_stop(my_sta); // Stop DHCP client
    esp_netif_set_ip_info(my_sta, &ipInfo);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = MY_WIFI_SSID,
            .password = MY_WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

void app_main(void)
{

    // WIFI INITIALIZATION
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();


    // RELAY INITIALIZATION
    gpio_reset_pin(RELAY_PIN1);
    gpio_reset_pin(RELAY_PIN2);
    // gpio_reset_pin(RELAY_PIN3);
    gpio_set_direction(RELAY_PIN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(RELAY_PIN2, GPIO_MODE_OUTPUT);
    // gpio_set_direction(RELAY_PIN3, GPIO_MODE_OUTPUT);

    // UART INITIALIZATION
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
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
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

            // if length greater than 3, then do the logic
            if (length > 3)

            {

                int sensor_value = atoi(value);
                if (sensor_value < 2400)
                {
                    printf("Turning off relay\n");
                    gpio_set_level(RELAY_PIN1, 0);
                    gpio_set_level(RELAY_PIN2, 0);
                    // gpio_set_level(RELAY_PIN3, 0);
                }
                else if (sensor_value > 2900)
                {
                    printf("Turning on relay\n");
                    gpio_set_level(RELAY_PIN1, 1);
                    gpio_set_level(RELAY_PIN2, 1);
                    // gpio_set_level(RELAY_PIN3, 1);
                }
            }
        }
    }
}