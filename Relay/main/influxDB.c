#include "esp_err.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_tls.h"

#define INFLUXDB_HOST "https://db.kenzend.net"
#define INFLUXDB_ORG "c4f03d288ae6856c"
#define INFLUXDB_BUCKET "bucket"
#define INFLUXDB_TOKEN                                                         \
  "FWTRnKH8iSHlzaeWiIpv7fOK_BtqZ_T2610J0TheqbNKq769c2e50va-"                   \
  "Pgu56I02s4NEPArI64EQdADNJZnJCQ=="

#define INFLUXDB_URL                                                           \
  INFLUXDB_HOST "/api/v2/write?org=" INFLUXDB_ORG "&bucket=" INFLUXDB_BUCKET

static const char *TAG_INFLUX = "INFLUXDB_CLIENT";

// HTTP event handler
esp_err_t http_event_handler(esp_http_client_event_t *evt) {
  switch (evt->event_id) {
  case HTTP_EVENT_ERROR:
    ESP_LOGI(TAG_INFLUX, "HTTP_EVENT_ERROR");
    break;
  case HTTP_EVENT_ON_DATA:
    if (!esp_http_client_is_chunked_response(evt->client)) {
      ESP_LOGI(TAG_INFLUX, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
      printf("%.*s\n", evt->data_len, (char *)evt->data);
    }
    break;
  default:
    break;
  }
  return ESP_OK;
}

// Send data to InfluxDB
void send_to_influxdb(const char *line_protocol_data) {
  esp_http_client_config_t config = {
      .url = INFLUXDB_URL,
      .event_handler = http_event_handler,

  };
  esp_http_client_handle_t client = esp_http_client_init(&config);

  char auth_header[128];
  snprintf(auth_header, sizeof(auth_header), "Token %s", INFLUXDB_TOKEN);
  esp_http_client_set_header(client, "Authorization", auth_header);
  esp_http_client_set_header(client, "Content-Type",
                             "text/plain; charset=utf-8");

  esp_http_client_set_method(client, HTTP_METHOD_POST);
  esp_http_client_set_post_field(client, line_protocol_data,
                                 strlen(line_protocol_data));

  esp_err_t err = esp_http_client_perform(client);
  if (err == ESP_OK) {
    ESP_LOGI(TAG_INFLUX, "HTTP POST Status = %d, content_length = %lld",
             esp_http_client_get_status_code(client),
             esp_http_client_get_content_length(client));
  } else {
    ESP_LOGE(TAG_INFLUX, "HTTP POST request failed: %s", esp_err_to_name(err));
  }

  esp_http_client_cleanup(client);
}