#include <stdio.h>                        // Standard C I/O functions (printf, etc.)
#include <string.h>                       // String handling functions (memcpy, strlen, etc.)
#include "freertos/FreeRTOS.h"            // FreeRTOS core definitions
#include "freertos/task.h"                // FreeRTOS task API (xTaskCreate, vTaskDelete, etc.)
#include "driver/uart.h"                  // UART driver definitions
#include "driver/gpio.h"                  // GPIO driver definitions
#include "esp_log.h"                      // Logging macros (ESP_LOGI, ESP_LOGE, etc.)

// ---------------- UART Pin Configuration ----------------
#define ECHO_TEST_TXD  (GPIO_NUM_1)       // UART TX pin (transmit data)
#define ECHO_TEST_RXD  (GPIO_NUM_3)       // UART RX pin (receive data)
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE) // RTS not used (no change)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE) // CTS not used (no change)

// Buffer size for UART data
#define BUF_SIZE (1024)

// ---------------- UART Echo Task ----------------
void uart_echo_task(void *arg)
{
    // Step 1: Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = 115200,               // Communication speed (bits per second)
        .data_bits = UART_DATA_8_BITS,     // 8 data bits per frame
        .parity = UART_PARITY_DISABLE,     // No parity check
        .stop_bits = UART_STOP_BITS_1,     // One stop bit
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // Disable hardware flow control
        .source_clk = UART_SCLK_DEFAULT,   // Use default UART clock source
    };

    // Step 2: Install the UART driver
    // Parameters: (uart_num, rx_buffer_size, tx_buffer_size, queue_size, queue_handle, intr_alloc_flags)
    int intr_alloc_flags = 0; // Interrupt allocation flags (0 = default)
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));

    // Step 3: Apply UART configuration
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));

    // Step 4: Set the physical TX, RX, RTS, and CTS pins
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, ECHO_TEST_TXD, ECHO_TEST_RXD,
                                 ECHO_TEST_RTS, ECHO_TEST_CTS));

    // Step 5: Allocate memory for data buffer
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE);
    if (data == NULL) {                     // Check if malloc succeeded
        ESP_LOGE("UART", "Cannot allocate memory");
        vTaskDelete(NULL);                  // Delete the task if memory failed
    }

    // Step 6: Infinite loop to read and echo UART data
    while (1) {
        // Read data from UART1
        // Parameters: (uart_num, buffer, length, timeout_ticks)
        int len = uart_read_bytes(UART_NUM_0, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);

        // If we received data
        if (len > 0) {
            // Send back (echo) the same data
            uart_write_bytes(UART_NUM_0, (const char*) data, len);
        }

        // Optional: Add a small delay to reduce CPU load
        // vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // Step 7: Free allocated memory (theoretically unreachable in infinite loop)
    free(data);
    vTaskDelete(NULL);
}

// ---------------- Main Application Entry ----------------
void app_main(void)
{
    // Create a FreeRTOS task to run the UART echo function
    // Parameters: (task_function, task_name, stack_size, parameters, priority, task_handle)
    xTaskCreate(uart_echo_task, "uart_echo_task", 2048, NULL, 10, NULL);
}
