#include <stdio.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"


//Settings 
static const gpio_num_t led_pin = GPIO_NUM_2; // Pin number for the LED
static const uint32_t sleep_time_ms = 1000; // Delay in milliseconds

void app_main(void){
    uint8_t led_state = 0;
    gpio_reset_pin(led_pin);
    gpio_set_direction(led_pin, GPIO_MODE_OUTPUT);

    while(1){
        led_state = !led_state; // Toggle LED state
        gpio_set_level(led_pin, led_state); // Set the LED pin level

        printf("LED state %d\n", led_state); // Print the LED state to the console

        vTaskDelay(pdMS_TO_TICKS(sleep_time_ms / portTICK_RATE_MS)); // Delay for the specified time

    }
}