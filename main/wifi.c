#include <stdio.h>
#include <string.h>
#include <sys/fcntl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#define MESSAGE "Hello TCP Client!!"
#define LISTENQ 2

static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;
static const char *TAG="sta_mode_tcp_server";

void wifi_connect(){
    wifi_config_t cfg = {
        .sta = {
			.ssid = "UNITO_SALA",
			.password = "05081997",
			.bssid_set = false
        },
    };
    ESP_ERROR_CHECK( esp_wifi_disconnect() );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &cfg) );
    ESP_ERROR_CHECK( esp_wifi_connect() );
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

void initialise_wifi(void) {
	ESP_LOGI(TAG,"initialising wifi\n");
	tcpip_adapter_init();

	ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
	wifi_event_group = xEventGroupCreate();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
	ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
	ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
	ESP_ERROR_CHECK( esp_wifi_start() );

}

void tcp_socket_task(void *pvParam) {
	static struct sockaddr_in remote_addr;
	static unsigned int socklen;
	int cs, s, r;//client socket
	char recv_buf[64];

	cs = *(int *)pvParam;

	ESP_LOGI(TAG,"Creating new client socket task. Socket: %d.", cs);

	//set O_NONBLOCK so that recv will return, otherwise we need to impliment message end
	//detection logic. If know the client message format you should instead impliment logic
	//detect the end of message
	fcntl(cs,F_SETFL,O_NONBLOCK);

	do {
		bzero(recv_buf, sizeof(recv_buf));
		r = recv(cs, recv_buf, sizeof(recv_buf)-1,0);
		for(int i = 0; i < r; i++) {
			putchar(recv_buf[i]);
		}
	} while(r > 0);

	ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d\r\n", r, errno);

	for (int j=0; j < 50; j++) {
		if( write(cs , MESSAGE , strlen(MESSAGE)) < 0) {
			ESP_LOGE(TAG, "... Send failed \n");
			break;
		} else {
			ESP_LOGI(TAG, "... socket send success");
		}
		vTaskDelay(2000 / portTICK_PERIOD_MS);
	}

	close(cs);
	ESP_LOGI(TAG, "socket closed.");

	// Deletes current task
	vTaskDelete(NULL);
}

void tcp_server_task(void *pvParam){

    ESP_LOGI(TAG,"tcp_server task starting \n");
    struct sockaddr_in tcpServerAddr;
    tcpServerAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    tcpServerAddr.sin_family = AF_INET;
    tcpServerAddr.sin_port = htons( 3000 );
    ESP_LOGI(TAG,"tcp server started on port 3000\n");
    int s, r;
    char recv_buf[64];
    static struct sockaddr_in remote_addr;
    static unsigned int socklen;
    socklen = sizeof(remote_addr);
    int cs;//client socket
    xEventGroupWaitBits(wifi_event_group,CONNECTED_BIT,false,true,portMAX_DELAY);

	s = socket(AF_INET, SOCK_STREAM, 0);
	if (s < 0) {
		ESP_LOGE(TAG, "... Failed to allocate socket.\n");
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		vTaskDelete(NULL);
		return;
	}

	ESP_LOGI(TAG, "... allocated socket\n");
	if (bind(s, (struct sockaddr *) &tcpServerAddr, sizeof(tcpServerAddr))
			!= 0) {
		ESP_LOGE(TAG, "... socket bind failed errno=%d \n", errno);
		close(s);
		vTaskDelete(NULL);
		return;
	}

	ESP_LOGI(TAG, "... socket bind done \n");
	if (listen(s, LISTENQ) != 0) {
		ESP_LOGE(TAG, "... socket listen failed errno=%d \n", errno);
		close(s);
		vTaskDelete(NULL);
		return;
	}

	while(1){
		cs=accept(s,(struct sockaddr *)&remote_addr, &socklen);
		if (cs < 0) {
			ESP_LOGI(TAG, "accept: %d %s\r\n", cs, strerror(errno));
			vTaskDelay(100 / portTICK_PERIOD_MS);
			continue;
		}
		ESP_LOGI(TAG,"New connection request. Client socket: %d.", cs);
		xTaskCreate(&tcp_socket_task, "tcp_server", 4096, (void *)&cs, 5, NULL);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
	return;
}
