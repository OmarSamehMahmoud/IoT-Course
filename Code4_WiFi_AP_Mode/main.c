#include "freertos/FreeRTOS.h"         // Core FreeRTOS definitions and data types
#include "freertos/event_groups.h"     // Event group functions (not used here but good for Wi-Fi events)
#include "esp_system.h"                // Basic ESP system functions
#include "esp_wifi.h"                  // Wi-Fi driver APIs
#include "esp_event.h"                 // Event loop and event handler management
#include "esp_log.h"                   // Logging utilities
#include "nvs_flash.h"                 // Non-volatile storage API (for Wi-Fi initialization requirement)
#include <string.h>              	   // For strlen()

#define AP_SSID      "EmAIoT_AP"        // SSID (network name) for the Access Point
#define AP_PASSWORD  "12345678"        // Password for the AP (must be at least 8 characters for WPA2)
#define AP_MAX_CONN  4                 // Maximum number of devices that can connect at the same time
#define AP_AUTHMODE  WIFI_AUTH_WPA2_PSK // Security mode: WPA2-PSK (most common and secure for APs)

static const char *TAG = "wifi_ap";    // Tag for log messages (helps identify which module logs come from)

//-----------------------------//
// Function: Initialize Wi-Fi Access Point
//-----------------------------//
void wifi_init_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init()); // Initialize network interface layer (required before using Wi-Fi)
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // Create default event loop for system events

    // Create default network interface for the Wi-Fi Access Point
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    assert(ap_netif); // Ensure AP network interface was created successfully

    // Load default configuration for Wi-Fi driver
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg)); // Initialize the Wi-Fi driver with the config

    // Prepare configuration for the Access Point
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = AP_SSID,                   // Set SSID (name of Wi-Fi network)
            .ssid_len = strlen(AP_SSID),       // Set SSID length
            .channel = 1,                      // Wi-Fi channel (1–13, depending on region)
            .password = AP_PASSWORD,           // Password for the network
            .max_connection = AP_MAX_CONN,     // Max number of connected clients
            .authmode = AP_AUTHMODE,           // Authentication mode (WPA2-PSK by default)
        },
    };

    // If the password is empty, make the network open (no password)
    if (strlen(AP_PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN; // No password (open network)
    }

    // Set Wi-Fi mode to Access Point (AP)
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    // Apply the AP configuration to the Wi-Fi interface
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));

    // Start the Wi-Fi driver — this activates the AP
    ESP_ERROR_CHECK(esp_wifi_start());

    // Log information messages about AP setup
    ESP_LOGI(TAG, "Wi-Fi initialized in AP mode. SSID: %s", AP_SSID);
    ESP_LOGI(TAG, "IP: 192.168.4.1"); // Default IP address of ESP32 in AP mode
}

//-----------------------------//
// Main Application Entry Point
//-----------------------------//
void app_main(void)
{
    // Initialize NVS (non-volatile storage)
    // NVS is required by Wi-Fi to store calibration and configuration data
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NEW_VERSION_FOUND) { // Handle NVS version mismatch
        ESP_ERROR_CHECK(nvs_flash_erase());     // Erase the old NVS partition
        ret = nvs_flash_init();                 // Reinitialize it
    }
    ESP_ERROR_CHECK(ret);                       // Abort if initialization failed

    wifi_init_ap(); // Initialize and start the Access Point

    // Placeholder: this loop keeps the program running
    // Here you can add here an HTTP server on 192.168.4.1
    while (1) {
        vTaskDelay(5000 / portTICK_PERIOD_MS);  // Delay for 5 seconds in each loop iteration
    }
}
