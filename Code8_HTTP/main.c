#include <stdio.h>                   // For basic input/output
#include <string.h>                  // For string functions like strlen()
#include "freertos/FreeRTOS.h"       // FreeRTOS core definitions
#include "freertos/task.h"           // For creating/delaying tasks
#include "esp_system.h"              // For general ESP system functions
#include "esp_wifi.h"                // For Wi-Fi control
#include "esp_event.h"               // For handling Wi-Fi events
#include "esp_log.h"                 // For logging info/errors
#include "nvs_flash.h"               // For NVS (non-volatile storage)
#include "driver/gpio.h"             // For controlling GPIO pins
#include "esp_netif.h"               // For network interface initialization
#include "esp_http_server.h"         // Official HTTP Server library

// ────────────────────────────────
// Configuration
// ────────────────────────────────
#define AP_SSID      "EmAIoT_WEB_CTRL"     // Wi-Fi SSID
#define AP_PASSWORD  "omarsamehsyam"           // Wi-Fi password (must be >= 8 chars)
#define LED_GPIO     GPIO_NUM_2           // Built-in LED pin on most ESP32 boards

static const char *TAG = "http_led";      // Tag for logging messages
static bool led_state = false;            // Global variable to store LED state

// ────────────────────────────────
// URI Handlers
// ────────────────────────────────

// Handler for root path "/"
static esp_err_t root_handler(httpd_req_t *req)
{
    // Choose LED color and status text based on led_state
    const char* status = led_state ? "ON" : "OFF";
    const char* color = led_state ? "green" : "red";

    // Create dynamic HTML page showing LED state and control buttons
    char html[1024];
    int len = snprintf(html, sizeof(html),
        "<!DOCTYPE html>"
        "<html>"
        "<head>"
        "  <title>ESP32 LED Control</title>"
        "  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
        "</head>"
        "<body style=\"font-family:Arial; text-align:center; padding:30px; background:#f0f0f0;\">"
        "  <h2>💡 ESP32 Web-Controlled LED</h2>"
        "  <p style=\"font-size:24px; margin:20px;\">"
        "    LED Status: <strong style=\"color:%s;\">%s</strong>"
        "  </p>"
        "  <div style=\"margin:20px;\">"
        "    <a href=\"/led/on\"><button style=\"font-size:18px; padding:12px 24px; background:green; color:white; border:none; border-radius:6px; margin:5px;\">Turn ON</button></a>"
        "    <a href=\"/led/off\"><button style=\"font-size:18px; padding:12px 24px; background:red; color:white; border:none; border-radius:6px; margin:5px;\">Turn OFF</button></a>"
        "  </div>"
        "  <p><small>Connect to Wi-Fi: <strong>%s</strong></small></p>"
        "</body>"
        "</html>",
        color, status, AP_SSID);

    httpd_resp_send(req, html, len);    // Send the HTML response
    return ESP_OK;
}

// Handler for "/led/on"
static esp_err_t led_on_handler(httpd_req_t *req)
{
    led_state = true;                   // Update LED state
    gpio_set_level(LED_GPIO, 1);        // Turn LED ON
    
    const char* resp = 
        "<!DOCTYPE html>"
        "<html><body style=\"text-align:center;padding:50px;\">"
        "<h3>✅ LED is NOW ON</h3>"
        "<a href=\"/\" style=\"font-size:18px;\">← Back to Control Panel</a>"
        "</body></html>";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN); // Send confirmation page
    return ESP_OK;
}

// Handler for "/led/off"
static esp_err_t led_off_handler(httpd_req_t *req)
{
    led_state = false;                  // Update LED state
    gpio_set_level(LED_GPIO, 0);        // Turn LED OFF
    
    const char* resp = 
        "<!DOCTYPE html>"
        "<html><body style=\"text-align:center;padding:50px;\">"
        "<h3>🛑 LED is NOW OFF</h3>"
        "<a href=\"/\" style=\"font-size:18px;\">← Back to Control Panel</a>"
        "</body></html>";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN); // Send confirmation page
    return ESP_OK;
}

// Handler for "/api/status" (JSON API)
static esp_err_t api_status_handler(httpd_req_t *req)
{
    const char* json = led_state ? 
        "{\"led\":\"on\",\"gpio\":2}" : 
        "{\"led\":\"off\",\"gpio\":2}";
    
    httpd_resp_set_type(req, "application/json"); // Set response type to JSON
    httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// ────────────────────────────────
// HTTP Server Initialization
// ────────────────────────────────

static httpd_handle_t start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();   // Default config
    config.server_port = 80;                          // Port 80 for HTTP
    config.max_open_sockets = 5;                      // Max concurrent clients

    httpd_handle_t server = NULL;
    esp_err_t ret = httpd_start(&server, &config);    // Start server
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return NULL;
    }

    // Define and register routes
    static httpd_uri_t root_uri = { .uri = "/", .method = HTTP_GET, .handler = root_handler };
    static httpd_uri_t led_on_uri = { .uri = "/led/on", .method = HTTP_GET, .handler = led_on_handler };
    static httpd_uri_t led_off_uri = { .uri = "/led/off", .method = HTTP_GET, .handler = led_off_handler };
    static httpd_uri_t api_status_uri = { .uri = "/api/status", .method = HTTP_GET, .handler = api_status_handler };

    httpd_register_uri_handler(server, &root_uri);
    httpd_register_uri_handler(server, &led_on_uri);
    httpd_register_uri_handler(server, &led_off_uri);
    httpd_register_uri_handler(server, &api_status_uri);

    ESP_LOGI(TAG, "HTTP server started on port %d", config.server_port);
    return server;
}

// ────────────────────────────────
// Wi-Fi Access Point Setup
// ────────────────────────────────

void wifi_init_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());                   // Initialize network interface
    ESP_ERROR_CHECK(esp_event_loop_create_default());    // Create default event loop
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    assert(ap_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); // Default Wi-Fi config
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = strlen(AP_SSID),
            .channel = 1,
            .password = AP_PASSWORD,
            .max_connection = 3,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));       // Set AP mode
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config)); // Apply config
    ESP_ERROR_CHECK(esp_wifi_start());                      // Start AP

    ESP_LOGI(TAG, "Wi-Fi AP started. SSID: %s | IP: 192.168.4.1", AP_SSID);
}

// ────────────────────────────────
// Main Application Entry Point
// ────────────────────────────────

void app_main(void)
{
    // Initialize NVS (used by Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize LED pin
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);      // Turn OFF initially
    ESP_LOGI(TAG, "LED initialized on GPIO %d", LED_GPIO);

    // Initialize Wi-Fi access point
    wifi_init_ap();

    // Start HTTP server
    httpd_handle_t server = start_http_server();
    if (!server) {
        ESP_LOGE(TAG, "HTTP server failed to start!");
        return;
    }

    // Keep main task alive
    while (1) {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
