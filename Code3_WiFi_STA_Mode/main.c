#include <string.h>                     // Standard C header for string functions (memcpy, strcmp, etc.)
#include "freertos/FreeRTOS.h"          // FreeRTOS base types and macros
#include "freertos/task.h"              // FreeRTOS task API (vTaskDelay, xTaskCreate, etc.)
#include "freertos/event_groups.h"      // FreeRTOS event groups API (xEventGroupCreate, xEventGroupWaitBits)
#include "esp_system.h"                 // ESP system level functions and types
#include "esp_wifi.h"                   // ESP-IDF Wi-Fi driver API
#include "esp_event.h"                  // Event loop and event handler API
#include "esp_log.h"                    // Logging macros (ESP_LOGI, ESP_LOGE, etc.)
#include "nvs_flash.h"                  // Non-volatile storage (NVS) flash API for storing Wi-Fi credentials, etc.

#define WIFI_SSID      "omarsamehsyam"   // Compile-time define for the Wi-Fi SSID to connect to
#define WIFI_PASS      "omar1996" // Compile-time define for the Wi-Fi password
#define WIFI_MAXIMUM_RETRY 5           // Maximum number of retries to attempt when connection fails

static const char *TAG = "wifi_sta";    // Tag used by ESP_LOG* macros to identify log messages from this module

static EventGroupHandle_t s_wifi_event_group; // Handle to a FreeRTOS event group for signaling Wi-Fi status
const int WIFI_CONNECTED_BIT = BIT0;    // Bit mask used to indicate successful connection in the event group
const int WIFI_FAIL_BIT = BIT1;         // Bit mask used to indicate failure to connect in the event group

static int s_retry_num = 0;             // Counter for number of connection attempts made so far

// Wi-Fi Event Handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    // If the Wi-Fi driver has started in station mode, attempt to connect to the AP
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();             // Ask the Wi-Fi driver to attempt a connection using configured credentials
    }
    // If station got disconnected from AP, handle reconnect logic and retries
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAXIMUM_RETRY) { // If we haven't exhausted retries
            esp_wifi_connect();         // Try to reconnect
            s_retry_num++;              // Increment retry counter
            ESP_LOGI(TAG, "Retry to connect to the AP"); // Log a retry attempt
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT); // Set failure bit to notify waiting tasks
        }
        ESP_LOGI(TAG, "Connect to the AP failed"); // Informational log that a disconnect happened
    }
    // If IP address was obtained from DHCP, signal success and reset retry counter
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data; // Cast event data to IP event structure
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip)); // Log the acquired IP address
        s_retry_num = 0;               // Reset retry counter on successful connection
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT); // Set connected bit to notify waiting tasks
    }
}

// Initialize Wi-Fi as a station (client)
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate(); // Create a FreeRTOS event group for Wi-Fi events

    ESP_ERROR_CHECK(esp_netif_init());        // Initialize network interface abstraction layer
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // Create default event loop for handling events
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta(); // Create default Wi-Fi station network interface
    assert(sta_netif);                        // Ensure the returned pointer is valid (debug assert)

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); // Default Wi-Fi driver initialization configuration
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));     // Initialize Wi-Fi driver with the default config

    // Register the Wi-Fi and IP event handlers so our wifi_event_handler will be called
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id)); // Register handler for any Wi-Fi events
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip)); // Register handler specifically for GOT_IP

    // Fill the wifi_config_t structure with SSID, password and security parameters
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,              // SSID to connect to
            .password = WIFI_PASS,          // Password for the SSID
            .threshold.authmode = WIFI_AUTH_WPA2_PSK, // Minimum allowed auth mode (WPA2)
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH, // SAE configuration for WPA3 compatibility (if supported)
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA)); // Set Wi-Fi interface to station (client) mode
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config)); // Apply the station configuration
    ESP_ERROR_CHECK(esp_wifi_start());            // Start the Wi-Fi driver (this will emit WIFI_EVENT_STA_START)

    ESP_LOGI(TAG, "Wi-Fi initialized in STA mode."); // Log that initialization completed
}

void app_main(void)
{
    // Initialize NVS (non-volatile storage) which is required by Wi-Fi for storing credentials
    esp_err_t ret = nvs_flash_init();            // Initialize NVS flash
    if (ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {  // If NVS partition was formatted with a newer version
        ESP_ERROR_CHECK(nvs_flash_erase());      // Erase NVS to reformat it for the current version
        ret = nvs_flash_init();                  // Re-initialize NVS after erase
    }
    ESP_ERROR_CHECK(ret);                        // Abort on any other NVS initialization error

    wifi_init_sta();                             // Call the function to initialize Wi-Fi in station mode

    // Wait for either the connected bit or the fail bit to be set (blocks indefinitely)
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,  // Wait for either success or failure bit
            pdFALSE,                             // Do not clear bits on exit
            pdFALSE,                             // Wait for any one bit (not all)
            portMAX_DELAY);                      // Block indefinitely until one of the bits is set

    if (bits & WIFI_CONNECTED_BIT) {             // If the connected bit is set in the returned bits
        ESP_LOGI(TAG, "Connected to AP!");      // Log successful connection
    } else if (bits & WIFI_FAIL_BIT) {           // If the fail bit is set
        ESP_LOGI(TAG, "Failed to connect to AP"); // Log failure to connect
    }

    // Main application loop — replace with your application code (HTTP client, MQTT, etc.)
    while (1) {
        vTaskDelay(5000 / portTICK_PERIOD_MS);   // Sleep/delay for 5000ms (5 seconds) to avoid busy loop
    }
}
