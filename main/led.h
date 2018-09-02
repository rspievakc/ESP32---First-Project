#include "freertos/queue.h"

void led_task(void *pvParameters);
extern QueueHandle_t led_queue;

typedef struct __attribute__((__packed__)) led_command {
	uint8_t start;
	uint8_t end;
	uint8_t step;
	uint8_t delay;
} led_command;
