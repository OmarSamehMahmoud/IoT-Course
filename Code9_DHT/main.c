#include <stdio.h>                      // Standard I/O library
#include "freertos/FreeRTOS.h"          // FreeRTOS core
#include "freertos/task.h"              // For task handling
#include "driver/gpio.h"                // For GPIO configuration
#include "esp_timer.h"                  // For microsecond delays
#include "esp_log.h"                    // For ESP logging functions

// ====================== CONFIGURATION ===========================
#define DHT_GPIO GPIO_NUM_4             // GPIO pin connected to DHT DATA
#define DHT_TYPE 11                    // 11 for DHT11, 22 for DHT22
// ================================================================

static const char *TAG = "DHT";         // Log tag for identification

// ============================================================================
// Microsecond delay function using esp_timer (busy wait)
// ============================================================================
static void delay_us(uint32_t us)
{
    uint64_t start = esp_timer_get_time();        // Record start time
    while (esp_timer_get_time() - start < us);    // Wait until time passes
}

// ============================================================================
// Normalization function: maps a value from old range to new range
// ============================================================================
float normalize(float value, float old_min, float old_max, float new_min, float new_max)
{
    if (value < old_min) value = old_min;         // Clamp lower limit
    if (value > old_max) value = old_max;         // Clamp upper limit
    return ((value - old_min) * (new_max - new_min) / (old_max - old_min)) + new_min;
}

// ============================================================================
// DHT Read Function
// ============================================================================
esp_err_t read_dht(float *temperature, float *humidity)
{
    uint8_t data[5] = {0};                        // Data buffer (5 bytes)
    uint64_t start, duration;                     // Timing variables

    // 1️⃣ Send start signal to DHT
    gpio_set_direction(DHT_GPIO, GPIO_MODE_OUTPUT);   // Set pin as output
    gpio_set_level(DHT_GPIO, 0);                      // Pull line LOW
    delay_us(20000);                                  // Hold for ≥18ms (DHT22)
    gpio_set_level(DHT_GPIO, 1);                      // Release line
    delay_us(30);                                     // Wait 20–40µs
    gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT);    // Switch to input

    // 2️⃣ Wait for sensor response (LOW → HIGH → LOW)
    uint64_t timeout = esp_timer_get_time() + 2000;   // 2ms timeout limit

    while (gpio_get_level(DHT_GPIO) == 1)
        if (esp_timer_get_time() > timeout) return ESP_FAIL;

    while (gpio_get_level(DHT_GPIO) == 0)
        if (esp_timer_get_time() > timeout) return ESP_FAIL;

    while (gpio_get_level(DHT_GPIO) == 1)
        if (esp_timer_get_time() > timeout) return ESP_FAIL;

    // 3️⃣ Read 40 bits (5 bytes)
    for (int i = 0; i < 40; i++) {
        while (gpio_get_level(DHT_GPIO) == 0);         // Wait for HIGH start
        start = esp_timer_get_time();                  // Record start of HIGH
        while (gpio_get_level(DHT_GPIO) == 1);         // Wait until LOW
        duration = esp_timer_get_time() - start;       // Duration of HIGH signal

        data[i / 8] <<= 1;                             // Shift bit left
        if (duration > 50) data[i / 8] |= 1;           // >50µs means logic '1'
    }

    // 4️⃣ Verify checksum (last byte = sum of first 4)
    if (((data[0] + data[1] + data[2] + data[3]) & 0xFF) != data[4]) {
        ESP_LOGE(TAG, "Checksum failed");
        return ESP_FAIL;
    }

    // 5️⃣ Convert raw bytes to real temperature & humidity
    if (DHT_TYPE == 11) {
        *humidity = data[0];
        *temperature = data[2];
    } else {
        *humidity = ((data[0] << 8) + data[1]) * 0.1;              // %RH
        *temperature = (((data[2] & 0x7F) << 8) + data[3]) * 0.1;  // °C
        if (data[2] & 0x80) *temperature *= -1;                    // Negative values
    }

    return ESP_OK; // Success
}

// ============================================================================
// Task: Read and display DHT values every 2 seconds
// ============================================================================
void dht_task(void *pvParameter)
{
    float temperature = 0, humidity = 0;    // Original sensor readings
    float fixed_temp = 0, fixed_hum = 0;    // Normalized values

    while (1) {
        // Try reading sensor
        if (read_dht(&temperature, &humidity) == ESP_OK) {
            // Apply normalization to bring values into 0–100 range 
            fixed_temp = normalize(temperature, 700.0, 800.0, 20.0, 35.0);
            fixed_hum  = normalize(humidity, 1200.0, 1600.0, 30.0, 70.0);

            // Log both raw and corrected values
            ESP_LOGI(TAG, "Temp: %.1f, Hum: %.1f%%", fixed_temp, fixed_hum);
        } else {
            ESP_LOGE(TAG, "Failed to read DHT sensor");
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // Delay 2 seconds
    }
}

// ============================================================================
// Main Application Entry Point
// ============================================================================
void app_main(void)
{
    // Reset the GPIO pin to its default state
    gpio_reset_pin(DHT_GPIO);

    // Enable internal pull-up resistor (helps DHT data line stay HIGH)
    gpio_set_pull_mode(DHT_GPIO, GPIO_PULLUP_ONLY);

    // Create FreeRTOS task to read DHT sensor periodically
    xTaskCreate(dht_task, "dht_task", 4096, NULL, 5, NULL);
}

