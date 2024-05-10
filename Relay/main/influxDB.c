#include "esp_http_client.h"
#include "esp_log.h"
#include "fullchain.h"

// #define INFLUXDB_HOST "db.kenzend.net"
#define INFLUXDB_HOST "192.168.0.127"
#define INFLUXDB_PORT "8086"
#define INFLUXDB_ORG "c4f03d288ae6856c"
#define INFLUXDB_BUCKET "bucket"
#define INFLUXDB_TOKEN                                                         \
  "NkDekShZFA7eO8p0llgEnUNMhGSwt-Tgu8cesUdUlJZhuJxKgeDY7vqejeTBXeKkthndUSan_"  \
  "RdDGisvIlBz_Q=="

/* #define INFLUXDB_URL                                                           \
  "https://" INFLUXDB_HOST "/api/v2/write?org=" INFLUXDB_ORG                   \
  "&bucket=" INFLUXDB_BUCKET */

#define INFLUXDB_URL                                                           \
  "http://" INFLUXDB_HOST ":" INFLUXDB_PORT "/api/v2/write?org=" INFLUXDB_ORG  \
  "&bucket=" INFLUXDB_BUCKET

static const char *TAG_INFLUX = "INFLUXDB_CLIENT";

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

void send_to_influxdb(const char *line_protocol_data) {
  esp_http_client_config_t config = {
      .url = INFLUXDB_URL,
    //   .cert_pem = (const char *)fullchain_pem,
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