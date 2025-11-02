/*
  Weather App for ESP32-S2 (Modified & Fixed)
  - DHT11 reading (bit-bang)
  - NVS save/load last reading
  - UART command "GET TEMP"
  - HTTP server with web UI and /api JSON
  - Try STA first, fallback to AP if STA fails
  - Fixed ip address printing using esp_ip4addr_ntoa
  - Fixed printf format specifiers and removed self-assignment
  - Extensive English comments for each part
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"          // FreeRTOS core
#include "freertos/task.h"              // FreeRTOS tasks
#include "freertos/event_groups.h"      // FreeRTOS event groups
#include "freertos/queue.h"             // FreeRTOS queues

#include "driver/gpio.h"                // GPIO driver
#include "driver/uart.h"                // UART driver

#include "esp_timer.h"                  // microsecond timer
#include "esp_log.h"                    // logging
#include "nvs.h"                        // NVS API
#include "nvs_flash.h"                  // NVS flash init
#include "esp_wifi.h"                   // Wi-Fi driver
#include "esp_event.h"                  // event loop
#include "esp_netif.h"                  // network interface
#include "esp_netif_ip_addr.h"          // esp_ip4addr_ntoa
#include "esp_http_server.h"            // HTTP server
#include "esp_err.h"                    // error codes

// ======================
// CONFIGURATION
// ======================
#define DHT_GPIO            GPIO_NUM_4       // DHT11 data pin
#define UART_PORT           UART_NUM_0       // UART port
#define UART_TX_PIN         GPIO_NUM_1       // UART TX
#define UART_RX_PIN         GPIO_NUM_3       // UART RX
#define UART_BUF_SIZE       128              // UART buffer size

// Station (STA) credentials — change to your router credentials
#define STA_SSID            "YourHomeSSID"
#define STA_PASS            "YourHomePass"

// Access Point (AP) fallback credentials
#define AP_SSID             "DHT11_Station"
#define AP_PASS             "IoT2025!"
#define AP_MAX_CONN         4

// Timings
#define STA_CONNECT_TIMEOUT_MS   10000       // 10 seconds to try STA
#define SENSOR_READ_INTERVAL_MS  5000        // sensor read every 5 seconds

// Logging tag
static const char *TAG = "weather_app";

// ======================
// GLOBAL STATE
// ======================
static float last_temp = 0.0f;               // last temperature read
static float last_hum  = 0.0f;               // last humidity read
static bool dht_valid  = false;              // have we a valid reading?
static httpd_handle_t http_server = NULL;    // HTTP server handle
static QueueHandle_t uart_queue = NULL;      // UART event queue handle
static EventGroupHandle_t wifi_event_group = NULL; // Wi-Fi event group

// Event bits
#define WIFI_CONNECTED_BIT   BIT0
#define WIFI_FAIL_BIT        BIT1

// ======================
// UTILITY: microsecond delay using esp_timer
// ======================
static void dht_delay_us(uint32_t us) {
    uint64_t start = esp_timer_get_time();    // record start in µs
    while ((esp_timer_get_time() - start) < us) {
        ; // busy wait for microseconds (DHT timing requires tight loops)
    }
}

// ======================
// DHT11 bit-bang read
// - returns ESP_OK on success and fills temperature & humidity
// ======================
esp_err_t dht11_read(float *temperature, float *humidity)
{
    uint8_t data[5] = {0}; // DHT11 returns 5 bytes

    // 1) Send start signal: pull data line low for >=18ms
    gpio_set_direction(DHT_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_GPIO, 0);
    dht_delay_us(18000); // 18 ms

    // 2) Release line and switch to input
    gpio_set_level(DHT_GPIO, 1);
    dht_delay_us(30); // 20-40 µs
    gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT);

    // 3) Wait for DHT response: LOW, HIGH, LOW sequence (with simple timeouts)
    uint32_t timeout = 0;
    while (gpio_get_level(DHT_GPIO) == 1) { if (++timeout > 10000) return ESP_ERR_TIMEOUT; }
    timeout = 0;
    while (gpio_get_level(DHT_GPIO) == 0) { if (++timeout > 10000) return ESP_ERR_TIMEOUT; }
    timeout = 0;
    while (gpio_get_level(DHT_GPIO) == 1) { if (++timeout > 10000) return ESP_ERR_TIMEOUT; }

    // 4) Read 40 bits (5 bytes)
    for (int i = 0; i < 40; ++i) {
        // wait for start of bit (line goes low) - small timeout
        timeout = 0;
        while (gpio_get_level(DHT_GPIO) == 0) { if (++timeout > 10000) return ESP_ERR_TIMEOUT; }

        // measure high duration
        uint64_t tstart = esp_timer_get_time();
        timeout = 0;
        while (gpio_get_level(DHT_GPIO) == 1) { if (++timeout > 10000) break; }
        uint32_t duration = (uint32_t)(esp_timer_get_time() - tstart);

        data[i/8] <<= 1;
        if (duration > 40) { // > ~40µs is '1'
            data[i/8] |= 1;
        }
    }

    // 5) checksum check
    uint8_t checksum = data[0] + data[1] + data[2] + data[3];
    if (checksum != data[4]) {
        return ESP_ERR_INVALID_CRC;
    }

    // Convert to floats (DHT11 returns integer parts only usually)
    *humidity = (float)data[0] + ((float)data[1] / 10.0f);
    *temperature = (float)data[2] + ((float)data[3] / 10.0f);
    return ESP_OK;
}

// ======================
// NVS: save & load last readings (scaled by 10 to keep one decimal)
// ======================
void nvs_save_reading(float temp, float hum)
{
    nvs_handle_t h;
    if (nvs_open("sensor", NVS_READWRITE, &h) != ESP_OK) return;
    int16_t t = (int16_t)(temp * 10.0f);
    int16_t hval = (int16_t)(hum * 10.0f);
    nvs_set_i16(h, "temp", t);
    nvs_set_i16(h, "hum", hval);
    nvs_commit(h);
    nvs_close(h);
}

void nvs_load_reading(float *temp, float *hum)
{
    nvs_handle_t h;
    if (nvs_open("sensor", NVS_READWRITE, &h) != ESP_OK) return;
    int16_t t, hval;
    if (nvs_get_i16(h, "temp", &t) == ESP_OK && nvs_get_i16(h, "hum", &hval) == ESP_OK) {
        *temp = ((float)t) / 10.0f;
        *hum  = ((float)hval) / 10.0f;
        dht_valid = true;
        ESP_LOGI(TAG, "Loaded from NVS: %.1f°C, %.1f%%", *temp, *hum);
    }
    nvs_close(h);
}

// ======================
// UART task: respond to "GET TEMP"
// ======================
void uart_task(void *arg)
{
    uart_event_t event;
    uint8_t *buf = (uint8_t *)malloc(UART_BUF_SIZE);
    if (!buf) vTaskDelete(NULL);

    while (1) {
        // wait for uart event from queue
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                // read bytes from UART
                int len = uart_read_bytes(UART_PORT, buf, event.size, portMAX_DELAY);
                if (len <= 0) continue;
                if (len >= UART_BUF_SIZE) len = UART_BUF_SIZE - 1;
                buf[len] = '\0';

                // trim leading whitespace/newline
                char *cmd = (char *)buf;
                while (*cmd == ' ' || *cmd == '\r' || *cmd == '\n') cmd++;

                // check command
                if (strncmp(cmd, "GET TEMP", 8) == 0) {
                    if (dht_valid) {
                        char resp[64];
                        // Use correct format specifiers
                        snprintf(resp, sizeof(resp), "TEMP:%.1f,HUM:%.1f\n", last_temp, last_hum);
                        uart_write_bytes(UART_PORT, resp, strlen(resp));
                    } else {
                        uart_write_bytes(UART_PORT, "ERROR: No valid reading\n", strlen("ERROR: No valid reading\n"));
                    }
                }
            }
        }
    }

    free(buf);
    vTaskDelete(NULL);
}

// ======================
// HTTP handlers (root UI & /api JSON)
// ======================
static esp_err_t root_handler(httpd_req_t *req)
{
    char html[512];
    const char *status = dht_valid ? "OK" : "ERROR";
    const char *color = dht_valid ? "green" : "red";

    snprintf(html, sizeof(html),
        "<!DOCTYPE html>"
        "<html><head><meta charset='utf-8'><title>DHT11 Weather Station</title></head>"
        "<body style='font-family:Arial;text-align:center;padding:40px;'>"
        "<h2>🌡️ DHT11 Weather Station</h2>"
        "<p>Status: <strong style='color:%s'>%s</strong></p>"
        "<p>Temperature: <strong>%.1f &deg;C</strong></p>"
        "<p>Humidity: <strong>%.1f %%</strong></p>"
        "<p><small>Send 'GET TEMP' over UART for readings</small></p>"
        "</body></html>",
        color, status, last_temp, last_hum);

    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t api_handler(httpd_req_t *req)
{
    if (!dht_valid) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No valid sensor data");
        return ESP_FAIL;
    }
    char json[128];
    snprintf(json, sizeof(json), "{\"temperature\":%.1f,\"humidity\":%.1f,\"status\":\"ok\"}", last_temp, last_hum);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static void start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    if (httpd_start(&http_server, &config) == ESP_OK) {
        httpd_uri_t root_uri = {
            .uri = "/", .method = HTTP_GET, .handler = root_handler, .user_ctx = NULL
        };
        httpd_register_uri_handler(http_server, &root_uri);

        httpd_uri_t api_uri = {
            .uri = "/api", .method = HTTP_GET, .handler = api_handler, .user_ctx = NULL
        };
        httpd_register_uri_handler(http_server, &api_uri);

        ESP_LOGI(TAG, "HTTP server started");
    } else {
        ESP_LOGE(TAG, "Failed to start HTTP server");
    }
}

// ======================
// Sensor task — periodically read DHT11
// ======================
void sensor_task(void *arg)
{
    while (1) {
        float t = 0.0f, h = 0.0f;
        esp_err_t r = dht11_read(&t, &h);
        if (r == ESP_OK) {
            last_temp = t;
            last_hum = h;
            dht_valid = true;
            nvs_save_reading(t, h); // persist last valid reading
            ESP_LOGI(TAG, "DHT11: %.1f°C, %.1f%%", t, h);
        } else {
            // do not overwrite last valid reading; just log
            ESP_LOGW(TAG, "DHT11 read failed: 0x%x", r);
        }
        vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
    }
    vTaskDelete(NULL);
}

// ======================
// Wi-Fi event handler
// - handle STA start, STA disconnected, and got IP events
// - use esp_ip4addr_ntoa to convert ip to string (fixes implicit declaration error)
// ======================
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            // when STA starts, try connect
            esp_wifi_connect();
            ESP_LOGI(TAG, "WIFI_EVENT_STA_START -> esp_wifi_connect()");
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            // STA disconnected
            ESP_LOGW(TAG, "WIFI_EVENT_STA_DISCONNECTED");
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        } else if (event_id == WIFI_EVENT_AP_START) {
            ESP_LOGI(TAG, "WIFI_EVENT_AP_START");
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            // got IP from AP: print it using esp_ip4addr_ntoa (correct function)
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            char ip_str[16];
            // esp_ip4addr_ntoa writes the dotted string into ip_str
            esp_ip4addr_ntoa(&event->ip_info.ip, ip_str, sizeof(ip_str));
            ESP_LOGI(TAG, "IP acquired: %s", ip_str);
            // signal connected
            xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        }
    }
}

// ======================
// Start Wi-Fi in STA mode and wait for connection or timeout
// - returns true if connected within timeout
// - ordering chosen to avoid ESP_ERR_WIFI_MODE on ESP32-S2
// ======================
static bool wifi_start_sta_and_wait(void)
{
    // Initialize TCP/IP stack and event loop (if not done already)
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize Wi-Fi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Create default WIFI STA netif AFTER esp_wifi_init (safe ordering for S2)
    esp_netif_create_default_wifi_sta();

    // Use RAM storage for Wi-Fi config (avoid flash writes)
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    // Create event group for wifi events
    wifi_event_group = xEventGroupCreate();

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    // Configure STA credentials
    wifi_config_t wifi_config = { 0 };
    strncpy((char*)wifi_config.sta.ssid, STA_SSID, sizeof(wifi_config.sta.ssid)-1);
    strncpy((char*)wifi_config.sta.password, STA_PASS, sizeof(wifi_config.sta.password)-1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK;

    // Set mode to STA and apply config
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    // Start Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Wi-Fi STA started, connecting to SSID: %s", STA_SSID);

    // Wait for either connected or fail bit
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdTRUE,    // clear on exit
                                           pdFALSE,   // wait for any
                                           pdMS_TO_TICKS(STA_CONNECT_TIMEOUT_MS));

    // If connected bit set, return true; else false
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to Wi-Fi STA");
        return true;
    } else {
        ESP_LOGW(TAG, "Failed to connect to STA within timeout");
        return false;
    }
}

// ======================
// Start Wi-Fi AP (fallback)
// - uses proper calls (esp_netif_create_default_wifi_ap AFTER esp_wifi_init)
// ======================
static void wifi_start_ap(void)
{
    // If wifi already initialized, just create default AP netif
    // (esp_wifi_init was called in wifi_start_sta_and_wait; safe to call create_default_wifi_ap now)
    esp_netif_create_default_wifi_ap();

    // Configure AP parameters
    wifi_config_t wifi_config = { 0 };
    strncpy((char*)wifi_config.ap.ssid, AP_SSID, sizeof(wifi_config.ap.ssid)-1);
    wifi_config.ap.ssid_len = strlen(AP_SSID);
    wifi_config.ap.channel = 1;
    wifi_config.ap.max_connection = AP_MAX_CONN;
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    if (strlen(AP_PASS) == 0) wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    else strncpy((char*)wifi_config.ap.password, AP_PASS, sizeof(wifi_config.ap.password)-1);

    // Set AP mode and apply config, then start
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Started Wi-Fi AP: SSID=%s", AP_SSID);

    // Get AP IP for logging (use default netif)
    esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (ap_netif) {
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(ap_netif, &ip_info) == ESP_OK) {
            char ip_str[16];
            esp_ip4addr_ntoa(&ip_info.ip, ip_str, sizeof(ip_str));
            ESP_LOGI(TAG, "AP IP: %s", ip_str);
        }
    }
}

// ======================
// UART initialization helper
// ======================
static void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB
    };
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // install uart driver and get event queue handle
    uart_driver_install(UART_PORT, UART_BUF_SIZE * 2, 0, 20, &uart_queue, 0);
}

// ======================
// MAIN (app_main)
// ======================
void app_main(void)
{
    // 1) Initialize NVS (required by Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2) Load last saved readings into memory (if any)
    nvs_load_reading(&last_temp, &last_hum);

    // 3) Configure DHT pin as input initially
    gpio_reset_pin(DHT_GPIO);
    gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT);

    // 4) Init UART and create UART task
    uart_init();
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 10, NULL);

    // 5) Attempt STA connection first — if fails, start AP as fallback
    bool sta_ok = wifi_start_sta_and_wait();
    if (!sta_ok) {
        ESP_LOGW(TAG, "STA failed — starting AP fallback");
        wifi_start_ap();
    } else {
        ESP_LOGI(TAG, "STA succeeded — running in station mode");
    }

    // 6) Start HTTP server
    start_http_server();

    // 7) Start sensor task for periodic DHT11 readings
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);

    // 8) Save initial log
    if (sta_ok) {
        ESP_LOGI(TAG, "System ready (STA). Use network to access UI/API.");
    } else {
        ESP_LOGI(TAG, "System ready (AP). Connect to SSID: %s", AP_SSID);
    }

    // main task idle loop (everything handled by tasks)
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
