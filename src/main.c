#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

#define LED_GPIO        GPIO_NUM_27
#define BUZZER_GPIO     GPIO_NUM_25

static const char *TAG = "TEST_LED_BUZZER";

// Functie pentru beep buzzer (PWM ON/OFF)
void buzzer_beep()
{
    // pornesc PWM cu duty 50%
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 4000);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(pdMS_TO_TICKS(200)); // beep 200ms

    // opresc PWM
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(pdMS_TO_TICKS(200)); // pauza
}

void app_main(void)
{
    ESP_LOGI(TAG, "Pornire test LED + BUZZER (fara DHT)");

    // Init LED
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);

    // Init PWM pentru buzzer
    ledc_timer_config_t timer_conf = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 3000,  // 3 kHz (sunet mai tare)
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t ch_conf = {
        .gpio_num       = BUZZER_GPIO,
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0,        // OFF
        .hpoint         = 0
    };
    ledc_channel_config(&ch_conf);

    // Variabila "temp" pentru test
    int temp = 18;

    while (1)
    {
        ESP_LOGI(TAG, "Temperatura simulata = %d°C", temp);

        // 1) LED ON la 20°C
        if (temp >= 20) {
            gpio_set_level(LED_GPIO, 1);
        } else {
            gpio_set_level(LED_GPIO, 0);
        }

        // 2) BUZZER BEEP la 21°C
        if (temp >= 21) {
            buzzer_beep();
        }

        // AICI poti schimba temperatura manual
        temp++;

        if (temp > 23) temp = 18; // reset test loop

        vTaskDelay(pdMS_TO_TICKS(1000)); // asteapta 1 sec
    }
}
