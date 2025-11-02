#include <stdio.h>                        // Standard I/O functions
#include <string.h>                       // String functions
#include "freertos/FreeRTOS.h"            // FreeRTOS definitions
#include "freertos/task.h"                // Task management
#include "freertos/queue.h"               // Queue functions
#include "driver/uart.h"                  // UART driver
#include "driver/gpio.h"                  // GPIO driver
#include "esp_log.h"                      // Logging functions (ESP_LOGI, etc.)

#define RX_BUF_SIZE 1024                  // Size of the receive buffer

#define TXD_PIN (GPIO_NUM_1)             // UART1 TX pin
#define RXD_PIN (GPIO_NUM_3)             // UART1 RX pin

static QueueHandle_t uart_queue;          // Queue handle for UART events

// ---------------- UART Event Handling Task ----------------
static void uart_event_task(void* arg)
{
    uart_event_t event;                   // Structure to hold UART event info
    uint8_t* dtmp = (uint8_t*) malloc(RX_BUF_SIZE);  // Temporary buffer for received data

    if (dtmp == NULL) {
        ESP_LOGE("UART", "Failed to allocate memory for UART buffer");
        vTaskDelete(NULL);
    }

    for (;;) {
        // Wait indefinitely for an event from the UART queue
        if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {

            // Check which kind of event occurred
            switch (event.type) {

                case UART_DATA:  // Data received event
                    // Read the received data into buffer
                    uart_read_bytes(UART_NUM_0, dtmp, event.size, portMAX_DELAY);

                    // Echo back the same data
                    uart_write_bytes(UART_NUM_0, (const char*) dtmp, event.size);
                    break;

                case UART_FIFO_OVF: // Hardware FIFO overflow
                    ESP_LOGI("UART", "Hardware FIFO Overflow");

                    // Flush input buffer to clear received data
                    uart_flush_input(UART_NUM_0);

                    // Reset the UART event queue
                    xQueueReset(uart_queue);
                    break;

                default:
                    // Ignore other event types
                    break;
            }
        }
    }

    // Free allocated memory (though this point is rarely reached)
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

// ---------------- Main Application ----------------
void app_main(void)
{
    // Step 1: Configure UART0 parameters
    uart_config_t uart_config = {
        .baud_rate = 115200,              // Set baud rate to 115200
        .data_bits = UART_DATA_8_BITS,    // 8 data bits
        .parity = UART_PARITY_DISABLE,    // No parity bit
        .stop_bits = UART_STOP_BITS_1,    // 1 stop bit
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // Disable hardware flow control
        .source_clk = UART_SCLK_DEFAULT,  // Use default UART clock
    };

    // Step 2: Apply configuration to UART0
    uart_param_config(UART_NUM_0, &uart_config);

    // Step 3: Set TX and RX pins for UART0
    uart_set_pin(UART_NUM_0, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Step 4: Install UART driver with an event queue
    // Parameters: (uart_num, rx_buffer_size, tx_buffer_size, queue_size, queue_handle, intr_alloc_flags)
    uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 20, &uart_queue, 0);

    // Step 5: Create the UART event handling task
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}
