/* LEDC (LED Controller) fade example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

#include "led.h"
#include "gamma.h"

/*
 * About this example
 *
 * 1. Start with initializing LEDC module:
 *    a. Set the timer of LEDC first, this determines the frequency
 *       and resolution of PWM.
 *    b. Then set the LEDC channel you want to use,
 *       and bind with one of the timers.
 *
 * 2. You need first to install a default fade function,
 *    then you can use fade APIs.
 *
 * 3. You can also set a target duty directly without fading.
 *
 * 4. This example uses GPIO18/19/4/5 as LEDC output,
 *    and it will change the duty repeatedly.
 *
 * 5. GPIO18/19 are from high speed channel group.
 *    GPIO4/5 are from low speed channel group.
 *
 */
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (18)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_HS_CH1_GPIO       (19)
#define LEDC_HS_CH1_CHANNEL    LEDC_CHANNEL_1

#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_LS_CH2_GPIO       (4)
#define LEDC_LS_CH2_CHANNEL    LEDC_CHANNEL_2
#define LEDC_LS_CH3_GPIO       (5)
#define LEDC_LS_CH3_CHANNEL    LEDC_CHANNEL_3

#define LEDC_TEST_CH_NUM       (4)
#define LEDC_TEST_DUTY         (8191)
#define LEDC_TEST_FADE_TIME    (6000)

static const char* TAG = "LED PWM";
QueueHandle_t led_queue;

void led_turn_on(void) {

}

void led_turn_off(void) {

}

void led_fade(uint8_t direction, uint8_t startValue, uint8_t endValue, uint8_t delay) {

}

void led_task(void *pvParameters)
{
    int ch = 3;

    led_queue = xQueueCreate( 10, sizeof( led_command ) );
	if (led_queue == NULL) {
		ESP_LOGE(TAG, "Error creating the LED queue.");
		vTaskDelete(NULL);
		return;
	}

    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 5000,                      // frequency of PWM signal
        .speed_mode = LEDC_HS_MODE,           // timer mode
        .timer_num = LEDC_HS_TIMER            // timer index
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);

    // Prepare and set configuration of timer1 for low speed channels
    ledc_timer.speed_mode = LEDC_LS_MODE;
    ledc_timer.timer_num = LEDC_LS_TIMER;
    ledc_timer_config(&ledc_timer);

    /*
     * Prepare individual configuration
     * for each channel of LED Controller
     * by selecting:
     * - controller's channel number
     * - output duty cycle, set initially to 0
     * - GPIO number where LED is connected to
     * - speed mode, either high or low
     * - timer servicing selected channel
     *   Note: if different channels use one timer,
     *         then frequency and bit_num of these channels
     *         will be the same
     */
    ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
        {
            .channel    = LEDC_HS_CH0_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH0_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .timer_sel  = LEDC_HS_TIMER
        },
        {
            .channel    = LEDC_HS_CH1_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH1_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .timer_sel  = LEDC_HS_TIMER
        },
        {
            .channel    = LEDC_LS_CH2_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH2_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .timer_sel  = LEDC_LS_TIMER
        },
        {
            .channel    = LEDC_LS_CH3_CHANNEL,
            .duty       = 8191,
            .gpio_num   = LEDC_LS_CH3_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .timer_sel  = LEDC_LS_TIMER
        },
    };

    // Set LED Controller with previously prepared configuration
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }

    // Initialize fade service.
    ledc_fade_func_install(0);

    led_command command;
	uint8_t loop=0;
	int8_t direction=0;
	uint32_t current = 0;

    while (1) {
    	ch = 3;

    	if (xQueueReceive(led_queue, &command, loop ? 0 : portMAX_DELAY) == pdTRUE) {
    		if (command.start == 0xFF && command.end == 0xFF) { // Stop fade loop
    			loop = 0;
    			ESP_LOGI(TAG, "Stopping fade loop.\n");
			} else if(command.start == command.end) { // Set value

				if (command.start > GAMA_TABLE_SIZE-1) {
					command.start = GAMA_TABLE_SIZE-1;
				}

				if (command.end > GAMA_TABLE_SIZE-1) {
					command.end = GAMA_TABLE_SIZE-1;
				}

    			current = command.start;
    			ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, gamma_table[current]);
				ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
				ESP_LOGI(TAG, "Setting led power to %d%%\n", current);
				loop = 0;
			} else if(command.start != command.end) { // Start fade
				if (command.start == 0xFF) {
					command.start = current;
				}
				if (command.end == 0xFF) {
					command.end = current;
				}
				direction = command.end > command.start ? 1 : -1;
				current = command.start;
				loop = 1;
				ESP_LOGI(TAG, "Starting fade loop. Parameters: [ start=%d, end=%d, step=%d, delay=%d ] - direction: %d\n",
				command.start, command.end, command.step, command.delay, direction);
			}
    	}
    	if (loop) {
    		ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, gamma_table[current]);
    		ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
    		ESP_LOGI(TAG, "Fading step. Parameters: [ start=%d, end=%d, step=%d, delay=%d ] - direction: %d - value=%d\n",
    								command.start, command.end, command.step, command.delay, direction, current);
    		if ((direction == -1 && current <= command.end) || (direction == 1 && current >= command.end)) {
    			loop = 0;
    			continue;
    		} else {
    			current += (command.step * (direction ? -1 : 1));
    			vTaskDelay(command.delay / portTICK_RATE_MS);
    		}
    	}
    }
}
