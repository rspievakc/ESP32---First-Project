#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/touch_pad.h"
#include "esp_log.h"
#include "esp_err.h"

#include "touch.h"

#define TOUCH_PAD_NO_CHANGE   (-1)
#define TOUCH_THRESH_NO_USE   (0)
#define TOUCH_FILTER_MODE_EN  (1)
#define TOUCHPAD_FILTER_TOUCH_PERIOD (10)

static const char* TAG = "Touch pad";

int state = 0;
int pressed = 0;
int touchThreshold = 0;
QueueHandle_t queue;
TickType_t start;
TickType_t end;

/*
  Read values sensed at all available touch pads.
  Use 2 / 3 of read value as the threshold
  to trigger interrupt when the pad is touched.
  Note: this routine demonstrates a simple way
  to configure activation threshold for the touch pads.
  Do not touch any pads when this routine
  is running (on application start).
 */
static void touch_init_thresholds(void)
{
    uint16_t touch_value;

    touch_pad_read_filtered(0, &touch_value);
    ESP_LOGI(TAG, "touch pad [%d] value is %d", 0, touch_value);
    //set interrupt threshold.
    touchThreshold = touch_value * 34 / 45;
    ESP_LOGI(TAG, "touch pad [%d] threshold value is %d", 0, touchThreshold);
    ESP_ERROR_CHECK(touch_pad_set_thresh(0, touchThreshold));
}

/*
  Handle an interrupt triggered when a pad is touched.
  Recognize what pad has been touched and save it in a table.
 */
static void touch_intr(void * arg)
{
    uint32_t pad_intr = touch_pad_get_status();
    //clear interrupt
    touch_pad_clear_status();

    if (state != pad_intr) {
    	state |= pad_intr;
    	start = xTaskGetTickCountFromISR();
    	end = 0;
		xQueueSendFromISR(queue, &state, NULL);
    }

    /*for (int i = 0; i < TOUCH_PAD_MAX; i++) {
        if ((pad_intr >> i) & 0x01) {
        	pressed = true;
        }
    }*/
}

void touch_monitor_task(void *pvParameter) {
	 uint16_t touch_value;
	 uint16_t touch_filter_value;
	while(1) {
		//touch_pad_read(0, &touch_value);
		//touch_pad_read_raw_data(0, &touch_value);
        touch_pad_read_filtered(0, &touch_filter_value);
        if ((state & 0x01) && touch_filter_value > touchThreshold) {
        	end = xTaskGetTickCount();
        	pressed = 0;
        	state &= ~0x01;
        	xQueueSendFromISR(queue, &state, NULL);
        }
		vTaskDelay(200 / portTICK_PERIOD_MS);
	}
}

/*
  Read values sensed at all available touch pads.
 Print out values in a loop on a serial monitor.
 */
void touch_task(void *pvParameter)
{
    queue = xQueueCreate( 10, sizeof( int ) );
    if (queue == NULL) {
    	ESP_LOGE(TAG, "Error creating the queue.");
    	vTaskDelete(NULL);
    	return;
    }

    // Initialize touch pad peripheral.
    // The default fsm mode is software trigger mode.
    touch_pad_init();
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);

    // Set reference voltage for charging/discharging
	// In this case, the high reference valtage will be 2.7V - 1V = 1.7V
	// The low reference voltage will be 0.5
	// The larger the range, the larger the pulse count value.
	touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);


	gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);
	touch_pad_config(0, TOUCH_THRESH_NO_USE);
	touch_pad_config(1, TOUCH_THRESH_NO_USE);


#if TOUCH_FILTER_MODE_EN
	touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD);
#endif

	touch_init_thresholds();
	touch_pad_isr_register(touch_intr, NULL);
	touch_pad_intr_enable();
	touch_pad_set_trigger_mode(TOUCH_TRIGGER_BELOW);

	xTaskCreate(&touch_monitor_task, "touch_pad_monitor_task", 2048, NULL, 5, NULL);

#if TOUCH_FILTER_MODE_EN
    printf("Touch Sensor filter mode read, the output format is: \nTouchpad num:[raw data, filtered data]\n\n");
#else
    printf("Touch Sensor normal mode read, the output format is: \nTouchpad num:[raw data]\n\n");
#endif
    uint32_t value;
    uint16_t touch_filter_value;
    uint32_t counter=0;
    while (1) {
    	xQueueReceive(queue, &value, portMAX_DELAY);
    	touch_pad_read_filtered(0, &touch_filter_value);

    	ESP_LOGI(TAG, "(%d) Value: %d - %d -> duration: %u %u %u.\n", counter, value, touch_filter_value, end, start, (end - start));
    	counter++;
    }
}
