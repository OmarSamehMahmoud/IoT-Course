#include <string.h>                      // Standard C library for string operations (e.g., strlen)
#include "freertos/FreeRTOS.h"           // Core FreeRTOS definitions
#include "freertos/event_groups.h"       // FreeRTOS event group APIs
#include "esp_system.h"                  // Basic ESP system functions
#include "esp_wifi.h"                    // Wi-Fi driver APIs
#include "esp_event.h"                   // Event loop and event handling APIs
#include "esp_log.h"                     // Logging utilities
#include "nvs_flash.h"                   // Non-volatile storage for Wi-Fi and system data

// Wi-Fi credentials for STA (Station) mode
#define STA_SSID     "omarsamehsyam"      // Name of the Wi-Fi network to connect to
#define STA_PASSWORD "omar1996"      // Password of the Wi-Fi network

// Wi-Fi credentials for AP (Access Point) mode
#define AP_SSID      "EmAIoT_Config"      // SSID for the Access Point created by ESP32
#define AP_PASSWORD  "Config123"         // Password for that Access Point

// Tag for logging output (helps identify messages from this module)
static const char *TAG = "wifi_sta_ap";

//---------------------------------------------------------//
// Event handler — handles Wi-Fi and IP-related system events
//---------------------------------------------------------//
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    // When the Station interface starts, connect to configured Wi-Fi
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();  // Begin connection attempt to the router (STA mode)
    }
    // When disconnected from Wi-Fi, automatically retry connecting
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();  // Try reconnecting after disconnection
    }
    // When the Station successfully gets an IP address from the router
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;  // Cast to IP event structure
        ESP_LOGI(TAG, "STA Got IP: " IPSTR, IP2STR(&event->ip_info.ip));  // Print the obtained IP address
    }
}

//---------------------------------------------------------//
// Function: Initialize Wi-Fi in both STA + AP mode
//---------------------------------------------------------//
void wifi_init_sta_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());                // Initialize the network interface layer
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // Create the default event loop

    // Create both network interfaces: one for STA and one for AP
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();  // Create Station interface
    assert(sta_netif);                                             // Ensure STA interface created successfully

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();    // Create Access Point interface
    assert(ap_netif);                                              // Ensure AP interface created successfully

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();           // Load default Wi-Fi initialization config
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));                          // Initialize the Wi-Fi driver

    // Register Wi-Fi and IP event handlers
    esp_event_handler_instance_t instance_any_id;                  // Handle for general Wi-Fi events
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
                        WIFI_EVENT,                                // Event base
                        ESP_EVENT_ANY_ID,                          // Listen for all Wi-Fi events
                        &wifi_event_handler,                       // Function to call when event happens
                        NULL,                                      // Argument to pass to the handler
                        &instance_any_id));                        // Instance pointer for deregistration later

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
                        IP_EVENT,                                  // Event base for IP-related events
                        IP_EVENT_STA_GOT_IP,                       // Specific event: Station got IP
                        &wifi_event_handler,                       // Function to call
                        NULL,                                      // Argument
                        NULL));                                    // No instance handle needed

    // Configuration for the Station (client) part
    wifi_config_t sta_config = {
        .sta = {
            .ssid = STA_SSID,                                      // Set STA SSID
            .password = STA_PASSWORD,                              // Set STA password
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,              // Require WPA2 minimum authentication
        },
    };

    // Configuration for the Access Point (hotspot) part
    wifi_config_t ap_config = {
        .ap = {
            .ssid = AP_SSID,                                       // SSID for the AP network
            .ssid_len = strlen(AP_SSID),                           // Length of SSID string
            .channel = 1,                                          // Wi-Fi channel to use
            .password = AP_PASSWORD,                               // Password for the AP
            .max_connection = 2,                                   // Max number of clients that can connect
            .authmode = WIFI_AUTH_WPA2_PSK,                        // Security mode (WPA2)
        },
    };

    // Set Wi-Fi mode to dual (AP + STA)
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

    // Apply both configurations
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config)); // Apply STA config
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));   // Apply AP config

    // Start Wi-Fi with both interfaces active
    ESP_ERROR_CHECK(esp_wifi_start());

    // Log that Wi-Fi dual mode is ready
    ESP_LOGI(TAG, "Wi-Fi initialized in STA+AP mode.");
    ESP_LOGI(TAG, "STA connecting to: %s", STA_SSID);
    ESP_LOGI(TAG, "AP SSID: %s (IP: 192.168.4.1)", AP_SSID);
}

//---------------------------------------------------------//
// Main entry point (like main() in normal C)
//---------------------------------------------------------//
void app_main(void)
{
    // Initialize NVS (non-volatile storage)
    // Required for Wi-Fi driver to store calibration and settings
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {   // If a newer version of NVS is detected
        ESP_ERROR_CHECK(nvs_flash_erase());       // Erase the existing storage
        ret = nvs_flash_init();                   // Re-initialize NVS
    }
    ESP_ERROR_CHECK(ret);                         // Check for any errors

    wifi_init_sta_ap(); // Initialize Wi-Fi in STA+AP (dual) mode

    // Main application loop — keeps the system running
    // You can add your own logic here (HTTP server, OTA updates, etc.)
    while (1) {
        vTaskDelay(5000 / portTICK_PERIOD_MS);    // Wait for 5 seconds between iterations
    }
}
