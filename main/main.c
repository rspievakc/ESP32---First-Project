#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <stdlib.h>

#include "led.h"
#include "touch.h"
#include "wifi.h"

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

void app_main(void)
{
    nvs_flash_init();

	printf("Hello world !!!\n");
	fflush(stdout);

	xTaskCreate(&touch_task, "touch_pad_read_task", 2048, NULL, 5, NULL);
	xTaskCreate(&led_task, "led_task", 2048, NULL, 1, NULL);
	initialise_wifi();
	xTaskCreate(&tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);

    /*gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);
    int level = 0;
    while (true) {
    	if (level == 1) {
			printf(".A.");
			fflush(stdout);
    	}
        gpio_set_level(GPIO_NUM_5, level);
        level = !level;
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }*/
}

