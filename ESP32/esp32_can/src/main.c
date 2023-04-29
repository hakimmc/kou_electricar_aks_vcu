/*__________________________________________INCLUDES________________________________________*/
#include "stdio.h"
#include "stdlib.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "string.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "driver/twai.h"
#include "driver/spi_master.h"
#include "MCP2515.h"
/*__________________________________________DEFINES_________________________________________*/
#define led_number 18

#define uart0_rx 44
#define uart0_tx 43
#define UART0 UART_NUM_0

#define WIFI_SSID      "**********"
#define WIFI_PASS      "**********"
#define MAXIMUM_RETRY  10

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define SCRIPT_ID "AKfycbwPHgADCyh5aIyiaku_zNwz4JcQDqKH62RFQf-NAreZSIO_-CLCMwYxN6BT0cpL1l_w-A"

int SPI_CS_PIN = 34;
int SPI_MOSI_PIN = 35;
int SPI_MISO_PIN = 37;
int SPI_CLK_PIN = 36;
/*__________________________________________STRUCTERS_______________________________________*/
struct VARIABLES{

	/*     veri sırası-----|         */
	/*                     |         */					 
	/*                     v         */						 	 	 	 	 	 	 	 
	/*         RF Values Start	     */
	int direction;
	int vehicle_speed;
	float battery_voltage[24];
	int battery_temparature;
	/*          RF Values End	     */
}ecar;
/*__________________________________________VARIABLES_______________________________________*/
int level=0;

TaskHandle_t myTaskHandle = NULL;
TaskHandle_t myTaskHandle2 = NULL;

static const int RX_BUF_SIZE = 1024;

static EventGroupHandle_t s_wifi_event_group;

static const char *TAG = "wifi station";

static int s_retry_num = 0;

int wifi_state=0;

int random_value=0;

char arr_of_sheet[500];

spi_device_handle_t spi_ch;
/*______________________________________FUNCTION DEFINES____________________________________*/
void Google_Sheet_Send_Task(void *arg);
void Demo_Task2(void *arg);
void uart_init(uart_port_t uart_num,int TXD_PIN,int RXD_PIN,int baud);
static void event_handler(void* arg, esp_event_base_t event_base,int32_t event_id, void* event_data);
void wifi_init_sta(void);
void write_to_google_sheet(char* _url);
void can_receive();
void can_transmit();
void rastgele_veri();
/*__________________________________________MAIN____________________________________________*/
void app_main() {
/*__________________________________________LED SETTING_____________________________________*/
    esp_rom_gpio_pad_select_gpio(led_number);
    gpio_set_direction(led_number,GPIO_MODE_OUTPUT);
/*__________________________________________UART SETTINGS___________________________________*/
    uart_init(UART0,uart0_tx,uart0_rx,9600);
/*__________________________________________FREERTOS TASK___________________________________*/
    //if you are using freertos change 0 to value of rtos_number;
    int rtos_number=0;
    xTaskCreate(Google_Sheet_Send_Task, "Google_Sheet_Send_Task", 4096, NULL, 10, &myTaskHandle);
    xTaskCreate(Demo_Task2, "Demo_Task2", 4096, NULL,10, &myTaskHandle2);
/*__________________________________________WIFI TASKS______________________________________*/
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    ecar.direction=2;
/*________________________________________MCP2515 INIT_______________________________________*/
    printf("can starting.");
    while(!MCP2515_Initialize()){printf(".");}
    printf("\ncan started!");
/*________________________________________TWAI CODES_________________________________________*/
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_20, GPIO_NUM_21, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        printf("Driver installed\n");
    } else {
        printf("Failed to install driver\n");
        return;
    }

    //Start TWAI driver
    if (twai_start() == ESP_OK) {
        printf("Driver started\n");
    } else {
        printf("Failed to start driver\n");
        return;
    }
/*__________________________________________________________________________________________*/
    while(rtos_number){
        gpio_set_level(led_number,level);
        level=!level;
        uart_write_bytes(1,"hello",strlen("hello"));
        vTaskDelay(500/portTICK_PERIOD_MS);
    }
}
/*__________________________________________________________________________________________*/

void Google_Sheet_Send_Task(void *arg)
{
    while(1){
        if(wifi_state){
            //printf("Connected. %s\n",WIFI_SSID);
            write_to_google_sheet(arr_of_sheet);
        }
        vTaskDelay(10/ portTICK_PERIOD_MS);
    }
}
void Demo_Task2(void *arg)
{
    while(1){
        rastgele_veri();
        sprintf(arr_of_sheet,"https://script.google.com/macros/s/%s/exec?yon=%d&hiz=%d&sicaklik=%d&b1v=%.2f&b2v=%.2f&b3v=%.2f&b4v=%.2f&b5v=%.2f&b6v=%.2f&b7v=%.2f&b8v=%.2f&b9v=%.2f&b10v=%.2f&b11v=%.2f&b12v=%.2f&b13v=%.2f&b14v=%.2f&b15v=%.2f&b16v=%.2f&b17v=%.2f&b18v=%.2f&b19v=%.2f&b20v=%.2f&b21v=%.2f&b22v=%.2f&b23v=%.2f&b24v=%.2f",SCRIPT_ID, (ecar.direction) , (ecar.vehicle_speed) , (ecar.battery_temparature) , (ecar.battery_voltage[0]),(ecar.battery_voltage[1]),(ecar.battery_voltage[2]),(ecar.battery_voltage[3]),(ecar.battery_voltage[4]),(ecar.battery_voltage[5]),(ecar.battery_voltage[6]),(ecar.battery_voltage[7]),(ecar.battery_voltage[8]),(ecar.battery_voltage[9]),(ecar.battery_voltage[10]),(ecar.battery_voltage[11]),(ecar.battery_voltage[12]),(ecar.battery_voltage[13]),(ecar.battery_voltage[14]),(ecar.battery_voltage[15]),(ecar.battery_voltage[16]),(ecar.battery_voltage[17]),(ecar.battery_voltage[18]),(ecar.battery_voltage[19]),(ecar.battery_voltage[20]),(ecar.battery_voltage[21]),(ecar.battery_voltage[22]),(ecar.battery_voltage[23]));
        printf("Demo_Task2 printing..\n");
        vTaskDelay(1000/ portTICK_PERIOD_MS);
    }
}


void uart_init(uart_port_t uart_num,int TXD_PIN,int RXD_PIN,int baud) 
{
    const uart_config_t uart_config = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(uart_num, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}
static void event_handler(void* arg, esp_event_base_t event_base,int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);
                 wifi_state=1;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
  

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}
void write_to_google_sheet(char* _url)
{
    esp_http_client_config_t config = {
        .url = _url,
        .auth_type = HTTP_AUTH_TYPE_BASIC,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,  //Specify transport type
        .crt_bundle_attach = esp_crt_bundle_attach, //Attach the certificate bundle
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status_code = esp_http_client_get_status_code(client);
        if (status_code == 200) {
            ESP_LOGI(TAG, "Data written to Google Sheets successfully!");
        }
    }

    esp_http_client_cleanup(client);
}
void can_receive(){
    twai_message_t message;
    if (twai_receive(&message, pdMS_TO_TICKS(10000)) == ESP_OK) {
        printf("Message received\n");
    } else {
        printf("Failed to receive message\n");
        return;
    }

    //Process received message
    if (message.extd) {
        printf("Message is in Extended Format\n");
    } else {
        printf("Message is in Standard Format\n");
    }
    printf("ID is %d\n", (int)message.identifier);
    if (!(message.rtr)) {
        for (int i = 0; i < message.data_length_code; i++) {
            printf("Data byte %d = %d\n", i, message.data[i]);
        }
    }
}
void can_transmit(){
    twai_message_t message;
    message.identifier = 0xAAAA;
    message.extd = 1;
    message.data_length_code = 4;
    for (int i = 0; i < 4; i++) {
        message.data[i] = 0;
    }

    //Queue message for transmission
    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        printf("Message queued for transmission\n");
    } else {
        printf("Failed to queue message for transmission\n");
    }
}


void rastgele_veri(){
    for(int i=0;i<24;i++){
        ecar.battery_voltage[i] = (rand()%5)+((rand()%100)/100);
    }
    ecar.battery_temparature = 33;
    ecar.vehicle_speed = 100;
}

int CAN_Send_With_Feedback(twai_message_t message_t,twai_message_t message_r,uint32_t timeout){
	int t_out=0;
	int return_value;
	uint32_t id = message_r.identifier;
	while(1){
		if (twai_transmit(&message_t, pdMS_TO_TICKS(1000)) != ESP_OK) {
            return_value = 0;
        }
		if (twai_receive(&message_r, pdMS_TO_TICKS(10000)) == ESP_OK) {
            return_value = 0;
        }
		if(message_r.data[0]==id){
			return_value = 1;
			break;
		}
		t_out++;
		vTaskDelay(1/portTICK_PERIOD_MS);
		if(t_out==timeout){
			return_value = 2;
			break;
		}
	}
	return return_value;
}