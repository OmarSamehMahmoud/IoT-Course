#include <stdio.h>                  // Standard input/output functions
#include <string.h>                 // For string handling (memcpy, strcmp, etc.)
#include <freertos/FreeRTOS.h>      // FreeRTOS base definitions
#include <freertos/task.h>          // For task creation and delays
#include <esp_system.h>             // ESP32 system functions
#include <esp_wifi.h>               // Wi-Fi driver
#include <esp_event.h>              // Event loop for Wi-Fi and system
#include <esp_log.h>                // Logging utilities
#include <nvs_flash.h>              // Non-volatile storage for Wi-Fi credentials
#include <driver/gpio.h>            // GPIO control
#include <esp_netif.h>              // Network interface setup
#include <mqtt_client.h>            // MQTT client
#include <esp_timer.h>              // High-resolution timer
#include "esp_rom_sys.h"            // For microsecond delay (esp_rom_delay_us)

// ====== Wi-Fi and MQTT configuration ======
#define WIFI_SSID       "omarsamehsyam"    // Your Wi-Fi SSID
#define WIFI_PASSWORD   "omar1996"         // Your Wi-Fi password
#define MQTT_BROKER_IP  "192.168.137.1"    // Local MQTT broker IP address

// ====== GPIO pin configuration ======
#define LED_GPIO        GPIO_NUM_2         // Onboard LED (GPIO 2)
#define DHT_GPIO        GPIO_NUM_4         // DHT11 data pin (GPIO 4)

// ====== MQTT topics ======
#define TEMP_TOPIC      "local/dht11/temperature"   // Temperature topic
#define HUMIDITY_TOPIC  "local/dht11/humidity"      // Humidity topic
#define LED_TOPIC       "local/actuator/led"        // LED control topic

// ====== Global variables ======
static const char *TAG = "dht11_mqtt";     // Logging tag name
static esp_mqtt_client_handle_t mqtt_client = NULL;  // MQTT client handle
static bool led_state = false;             // Current LED state

// ====== Microsecond delay function ======
static void dht_delay_us(uint32_t us)
{
    esp_rom_delay_us(us);  // Accurate microsecond delay using ROM function
}

// ====== Read data from DHT11 sensor with retries ======
esp_err_t dht11_read(uint8_t *temperature, uint8_t *humidity)
{
    uint8_t data[5] = {0};     // Buffer for DHT11 data
    int retry = 2;             // Try reading twice if it fails

    while (retry--) {
        // Start signal for DHT11
        gpio_set_direction(DHT_GPIO, GPIO_MODE_OUTPUT);  // Set as output
        gpio_set_level(DHT_GPIO, 0);                     // Pull low
        vTaskDelay(pdMS_TO_TICKS(20));                   // Hold for 20ms
        gpio_set_level(DHT_GPIO, 1);                     // Pull high
        dht_delay_us(50);                                // Wait 50µs
        gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT);   // Switch to input to read data

        // Wait for DHT11 response (low and high pulses)
        int timeout = 0;
        while (gpio_get_level(DHT_GPIO) == 1 && timeout++ < 100) dht_delay_us(1);
        timeout = 0;
        while (gpio_get_level(DHT_GPIO) == 0 && timeout++ < 100) dht_delay_us(1);
        timeout = 0;
        while (gpio_get_level(DHT_GPIO) == 1 && timeout++ < 100) dht_delay_us(1);

        // Read 40 bits of data (humidity, temperature, checksum)
        for (int i = 0; i < 40; i++) {
            timeout = 0;
            while (gpio_get_level(DHT_GPIO) == 0 && timeout++ < 200) dht_delay_us(1); // Wait for start of bit
            uint64_t start = esp_timer_get_time(); // Start measuring pulse length
            while (gpio_get_level(DHT_GPIO) == 1 && timeout++ < 200) dht_delay_us(1); // Wait for end of bit
            uint32_t duration = esp_timer_get_time() - start; // Measure pulse width

            // If pulse > 40µs, bit = 1, else bit = 0
            data[i / 8] <<= 1;
            if (duration > 40) data[i / 8] |= 1;
        }

        // Verify checksum
        if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
            *humidity = data[0];       // Integer part of humidity
            *temperature = data[2];    // Integer part of temperature
            return ESP_OK;             // Reading OK
        }

        // Retry if checksum fails
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGE(TAG, "DHT11 checksum error after retries");
    return ESP_FAIL;  // Failed after retries
}

// ====== Wi-Fi event handler ======
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect(); // Start connecting when Wi-Fi starts
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi disconnected. Reconnecting...");
        esp_wifi_connect(); // Auto reconnect on disconnection
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Wi-Fi connected, IP: " IPSTR, IP2STR(&event->ip_info.ip)); // Log IP address
    }
}

// ====== MQTT event handler ======
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "✅ Connected to MQTT broker");
            esp_mqtt_client_subscribe(client, LED_TOPIC, 1); // Subscribe to LED topic
            break;

        case MQTT_EVENT_DATA: {
            char topic[128] = {0};
            char payload[128] = {0};
            memcpy(topic, event->topic, event->topic_len);    // Copy topic string
            memcpy(payload, event->data, event->data_len);    // Copy payload string
            topic[event->topic_len] = '\0';
            payload[event->data_len] = '\0';
            ESP_LOGI(TAG, "📩 %s => %s", topic, payload);     // Log received message

            // If LED topic message received
            if (strcmp(topic, LED_TOPIC) == 0) {
                bool new_state = led_state; // Default: no change

                // Parse payload text
                if (strcasecmp(payload, "ON") == 0) {
                    new_state = true;
                } else if (strcasecmp(payload, "OFF") == 0) {
                    new_state = false;
                }

                // Only change if state differs
                if (new_state != led_state) {
                    gpio_set_level(LED_GPIO, new_state ? 1 : 0); // Set LED level
                    led_state = new_state;
                    // Publish confirmation message (ON/OFF)
                    esp_mqtt_client_publish(client, LED_TOPIC, led_state ? "ON" : "OFF", 0, 1, true);
                    ESP_LOGI(TAG, "LED state updated to: %s", led_state ? "ON" : "OFF");
                }
                // If same state, do nothing → prevents MQTT feedback loop
            }
            break;
        }

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT connection error"); // Connection error
            break;

        default:
            break;
    }
}

// ====== Initialize Wi-Fi in station mode ======
void wifi_init_sta(void)
{
    esp_netif_init();                       // Initialize network interface
    esp_event_loop_create_default();        // Create default event loop
    esp_netif_create_default_wifi_sta();    // Create default Wi-Fi station

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); // Default config
    esp_wifi_init(&cfg);                                // Initialize Wi-Fi driver

    // Register event handlers
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

    // Configure Wi-Fi credentials
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);          // Station mode (client)
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config); // Apply Wi-Fi settings
    esp_wifi_start();                          // Start Wi-Fi
}

// ====== Start MQTT client ======
void mqtt_app_start(void)
{
    static char broker_uri[64];
    snprintf(broker_uri, sizeof(broker_uri), "mqtt://%s:1883", MQTT_BROKER_IP); // Format broker URI

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = broker_uri,                 // Broker URI
        .credentials.client_id = "esp32_dht11_local",     // MQTT client ID
        .network.disable_auto_reconnect = false,          // Auto reconnect enabled
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg); // Initialize MQTT client
    if (!mqtt_client) {
        ESP_LOGE(TAG, "MQTT init failed");
        return;
    }

    // Register MQTT event callback
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client); // Start client
}

// ====== Task: read DHT11 and send data via MQTT ======
void dht11_mqtt_task(void *pvParameters)
{
    uint8_t temp = 0, hum = 0;
    char temp_str[10], hum_str[10];

    while (1) {
        if (!mqtt_client) {
            ESP_LOGW(TAG, "MQTT not ready yet...");
            vTaskDelay(pdMS_TO_TICKS(2000));  // Wait before retrying
            continue;
        }

        if (dht11_read(&temp, &hum) == ESP_OK) { // Successfully read DHT11
            snprintf(temp_str, sizeof(temp_str), "%d", temp);
            snprintf(hum_str, sizeof(hum_str), "%d", hum);
            ESP_LOGI(TAG, "Temperature:  %s  | Humidity:  %s%%", temp_str, hum_str);
            esp_mqtt_client_publish(mqtt_client, TEMP_TOPIC, temp_str, 0, 1, false); // Publish temperature
            esp_mqtt_client_publish(mqtt_client, HUMIDITY_TOPIC, hum_str, 0, 1, false); // Publish humidity
        } else {
            ESP_LOGW(TAG, "Failed to read DHT11 — skipping publish");
        }

        vTaskDelay(pdMS_TO_TICKS(3000)); // Wait 3 seconds before next reading
    }
}

// ====== Main application entry point ======
void app_main(void)
{
    // Initialize non-volatile storage
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Configure LED pin
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0); // LED off initially

    // Configure DHT pin
    gpio_reset_pin(DHT_GPIO);
    gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT);

    ESP_LOGI(TAG, "🔌 Connecting Wi-Fi...");
    wifi_init_sta();  // Initialize Wi-Fi

    vTaskDelay(pdMS_TO_TICKS(7000)); // Wait for connection before MQTT
    mqtt_app_start();                // Start MQTT client

    // Create task for DHT11 readings
    xTaskCreate(dht11_mqtt_task, "dht11_mqtt_task", 4096, NULL, 5, NULL);
}
