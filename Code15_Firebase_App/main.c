#include <stdio.h>                  // Standard input/output library (printf, etc.)
#include <string.h>                 // String manipulation functions (strlen, strcmp, etc.)
#include <time.h>                   // Time functions (time(), etc.)
#include "freertos/FreeRTOS.h"      // FreeRTOS main header for tasks, delays, etc.
#include "freertos/task.h"          // FreeRTOS task management
#include "esp_system.h"             // ESP32 system-level functions
#include "esp_wifi.h"               // ESP32 WiFi API
#include "esp_event.h"              // Event loop library for handling events
#include "esp_log.h"                // Logging library
#include "nvs_flash.h"              // Non-volatile storage (flash memory) API
#include "esp_netif.h"              // Network interface library
#include "esp_http_client.h"        // HTTP client library
#include "cJSON.h"                  // JSON parsing library
#include "driver/gpio.h"            // GPIO driver library
#include "esp_timer.h"              // High-resolution timer functions
#include "certs/server_certs.h"     // SSL/TLS certificate for HTTPS

// ==========================
// WiFi Credentials
// ==========================
#define WIFI_SSID       "omarsamehsyam"        // WiFi SSID
#define WIFI_PASSWORD   "omar1996"             // WiFi password

// ==========================
// Firebase Settings
// ==========================
#define FIREBASE_URL    "https://fir-eb5cc-default-rtdb.firebaseio.com/" // Firebase Realtime Database URL
#define FIREBASE_SECRET "gq3HH0JvA6Fh5oZZmPxDzoSvG1IXwPMuGQMZx1YJ"       // Firebase database secret

// ==========================
// GPIO Pins
// ==========================
#define DHT_GPIO        GPIO_NUM_4      // DHT11 sensor connected to GPIO4
#define LED_GPIO        GPIO_NUM_2      // LED connected to GPIO2

// ==========================
// Logging Tag
// ==========================
static const char *TAG = "firebase_dht11";  // Tag for logging messages

// ======================================================
// Microsecond Delay Utility
// ======================================================
static void dht_delay_us(uint32_t us) {
    uint64_t start = esp_timer_get_time();   // Get current time in microseconds
    while ((esp_timer_get_time() - start) < us);  // Busy wait until desired time elapsed
}

// ======================================================
// DHT11 Sensor Reading Function
// ======================================================
esp_err_t dht11_read(float *temperature, float *humidity)
{
    uint8_t data[5] = {0};                    // Array to store 5 bytes from DHT11

    gpio_set_direction(DHT_GPIO, GPIO_MODE_OUTPUT); // Set DHT pin as output
    gpio_set_level(DHT_GPIO, 0);              // Pull pin low for at least 18ms to start signal
    dht_delay_us(18000);                      // 18ms delay

    gpio_set_level(DHT_GPIO, 1);              // Pull pin high for 20-40us
    dht_delay_us(30);                         // Delay 30 microseconds
    gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT);  // Switch pin to input to read sensor response

    uint32_t timeout = 0;                     // Timeout counter
    while (gpio_get_level(DHT_GPIO) == 1)     // Wait for DHT to pull low
        if (++timeout > 10000) return ESP_ERR_TIMEOUT; // Return error if timeout

    timeout = 0;
    while (gpio_get_level(DHT_GPIO) == 0)     // Wait for DHT to pull high
        if (++timeout > 10000) return ESP_ERR_TIMEOUT; // Return error if timeout

    timeout = 0;
    while (gpio_get_level(DHT_GPIO) == 1)     // Wait for DHT to start sending data
        if (++timeout > 10000) return ESP_ERR_TIMEOUT; // Return error if timeout

    // Read 40 bits (5 bytes) from DHT11
    for (int i = 0; i < 40; ++i) {
        timeout = 0;
        while (gpio_get_level(DHT_GPIO) == 0) // Wait for high signal start
            if (++timeout > 10000) return ESP_ERR_TIMEOUT;

        uint64_t tstart = esp_timer_get_time(); // Record time when signal goes high
        timeout = 0;
        while (gpio_get_level(DHT_GPIO) == 1) // Measure high signal duration
            if (++timeout > 10000) break;

        uint32_t duration = (uint32_t)(esp_timer_get_time() - tstart); // High pulse duration

        data[i / 8] <<= 1;                     // Shift current byte left by 1
        if (duration > 40) data[i / 8] |= 1;   // If pulse >40us, it's a 1, else 0
    }

    // Check checksum
    if (data[0] + data[1] + data[2] + data[3] != data[4])
        return ESP_ERR_INVALID_CRC;           // Return CRC error if checksum fails

    *humidity = data[0] + (data[1] / 10.0f);  // Combine integer and decimal humidity
    *temperature = data[2] + (data[3] / 10.0f); // Combine integer and decimal temperature

    return ESP_OK;                             // Return success
}

// ======================================================
// Send Data to Firebase
// ======================================================
void send_to_firebase(float temp, float hum)
{
    cJSON *root = cJSON_CreateObject();       // Create JSON object
    cJSON_AddNumberToObject(root, "temperature", temp); // Add temperature
    cJSON_AddNumberToObject(root, "humidity", hum);    // Add humidity
    cJSON_AddNumberToObject(root, "timestamp", (double)time(NULL)); // Add current timestamp
    char *post_data = cJSON_PrintUnformatted(root);   // Convert JSON object to string

    char url[300];
    snprintf(url, sizeof(url),                    // Construct Firebase URL with auth
             "%s/sensors/dht11.json?auth=%s",
             FIREBASE_URL,
             FIREBASE_SECRET);

    esp_http_client_config_t config = {          // Configure HTTP client
        .url = url,                              // URL to send data
        .method = HTTP_METHOD_PUT,               // HTTP PUT method
        .timeout_ms = 10000,                     // 10-second timeout
        .cert_pem = (char *)google_root_ca_pem_start, // SSL certificate
    };

    esp_http_client_handle_t client = esp_http_client_init(&config); // Initialize HTTP client
    esp_http_client_set_header(client, "Content-Type", "application/json"); // Set header
    esp_http_client_set_post_field(client, post_data, strlen(post_data)); // Set payload

    esp_err_t err = esp_http_client_perform(client); // Perform HTTP request

    if (err == ESP_OK) {                        // If request successful
        int status = esp_http_client_get_status_code(client); // Get HTTP response code
        ESP_LOGI(TAG, "Firebase Status: %d", status); // Log status
    } else {                                    // If request failed
        ESP_LOGE(TAG, "HTTP Error: %s", esp_err_to_name(err)); // Log error
    }

    esp_http_client_cleanup(client);            // Clean up HTTP client
    cJSON_Delete(root);                         // Delete JSON object
    free(post_data);                             // Free JSON string memory
}

// ======================================================
// WiFi Event Handler
// ======================================================
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) { // WiFi started
        esp_wifi_connect();                    // Connect to WiFi
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) { // WiFi disconnected
        ESP_LOGW(TAG, "WiFi Disconnected. Reconnecting...");
        esp_wifi_connect();                    // Reconnect
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) { // Got IP
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;      // Extract IP event data
        ESP_LOGI(TAG, "Connected! IP: " IPSTR, IP2STR(&event->ip_info.ip)); // Log IP
    }
}

// ======================================================
// Initialize WiFi in Station Mode
// ======================================================
void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());         // Initialize TCP/IP network stack
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // Create default event loop
    esp_netif_create_default_wifi_sta();       // Create default WiFi station

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); // Default WiFi init config
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));      // Initialize WiFi driver

    esp_event_handler_instance_t instance_any_id; // Event handler instances
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id)); // Register WiFi events

    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip)); // Register IP event

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,                 // WiFi SSID
            .password = WIFI_PASSWORD,         // WiFi password
            .threshold.authmode = WIFI_AUTH_WPA2_PSK, // Minimum auth mode
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA)); // Set WiFi to station mode
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config)); // Set WiFi config
    ESP_ERROR_CHECK(esp_wifi_start());             // Start WiFi
}

// ======================================================
// Task to Read DHT11 and Send to Firebase
// ======================================================
void dht_firebase_task(void *pvParameter)
{
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT); // Set LED pin as output

    float temp = 0.0f, hum = 0.0f;                 // Variables for temperature & humidity

    while (1) {                                    // Infinite loop
        if (dht11_read(&temp, &hum) == ESP_OK) {  // Read DHT11 sensor
            ESP_LOGI(TAG, "DHT11: %.1fC, %.1f%%", temp, hum); // Log values
            send_to_firebase(temp, hum);          // Send values to Firebase

            gpio_set_level(LED_GPIO, 1);          // Turn LED on briefly
            vTaskDelay(100 / portTICK_PERIOD_MS); // Delay 100ms
            gpio_set_level(LED_GPIO, 0);          // Turn LED off
        }
        else {                                     // If reading failed
            ESP_LOGW(TAG, "DHT11 read failed");   // Log warning
        }

        vTaskDelay(5000 / portTICK_PERIOD_MS);    // Wait 5 seconds before next reading
    }
}

// ======================================================
// Application Entry Point
// ======================================================
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());            // Initialize non-volatile storage

    gpio_reset_pin(DHT_GPIO);                     // Reset DHT GPIO
    gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT); // Set DHT pin as input initially

    wifi_init_sta();                              // Initialize WiFi

    xTaskCreate(dht_firebase_task, "dht_firebase_task", 4096, NULL, 5, NULL); // Start DHT task
}
