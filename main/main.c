#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_vfs_fat.h"
#include "esp_spiffs.h"
#include "esp_spi_flash.h"
#include "protocol_examples_common.h"
#include "addr_from_stdin.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "mpu6050/GY521.h"
#include "max30102/max30102.h"
#include "esp_efuse.h"
#include "esp_efuse_table.h"
#include <stdbool.h>
#include <string.h>
#include "ds18b20.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "lwip/sys.h"
#include "freertos/message_buffer.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "vs1053.h"
static const char *TAG = "wifi_init";
#define I2C_FRQ 100000
max30102_config_t max30102 = {};
EventGroupHandle_t s_wifi_event_group;
#define delay_ms(x) vTaskDelay(x / portTICK_PERIOD_MS);

// #define EXAMPLE_ESP_WIFI_SSID "AAA"
// #define EXAMPLE_ESP_WIFI_PASS "1805300624."
uint64_t address_list[2];
uint8_t found = 0;
float temperature_list[2];
ds18x20_t ds18b20;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define RESET_SOCK         BIT2
#define PLAYSTATUS         BIT3
#define GPS_Start          BIT4
#define EXAMPLE_ESP_MAXIMUM_RETRY 5
#define xMessageBufferSize 55200L

MessageBufferHandle_t xMessageBuffer;


typedef struct
{
    u8_t start;
    u8_t Mac_id[6];
    int tem;
    short step;
    int heart;
    u8_t len;
    char gps[120];
    u8_t end;
}meseege;



meseege m;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    static int s_retry_num = 0;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        xEventGroupSetBits(s_wifi_event_group, RESET_SOCK);
    }
    ESP_LOGW("s","%d",s_retry_num);
}


static void wifi_init_sta(void* param)
{

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
            .ssid = "AAA",
            .password = "1805300624.",
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );
    while(1)
    {
        if(!(xEventGroupGetBits(s_wifi_event_group)&PLAYSTATUS))
        {
        if(!(xEventGroupGetBits(s_wifi_event_group)&WIFI_CONNECTED_BIT))
        {
            
            esp_wifi_connect();

            ESP_LOGI(TAG, "wifi_init_sta finished.");

            /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
            * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
            EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                    WIFI_CONNECTED_BIT,
                    pdFALSE,
                    pdFALSE,
                    500);

            /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
            * happened. */
            if (bits & WIFI_CONNECTED_BIT) {
                ESP_LOGI(TAG, "connected to ap SSID: password:");
            }   
        }            
        }

        vTaskDelay(10000);
    }

}


// static esp_err_t maxi2c_master_init(i2c_port_t i2c_port){
//     i2c_config_t conf = {};
//     conf.mode = I2C_MODE_MASTER;
//     conf.sda_io_num = 40;
//     conf.scl_io_num = 41;
//     conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
//     conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
//     conf.master.clk_speed = I2C_FRQ;
//     i2c_param_config(1, &conf);
//     return i2c_driver_install(1, I2C_MODE_MASTER, 0, 0, 0);
// }

// static void get_bpm(void* param) {
//     printf("MAX30102 Test\n");
//     max30102_data_t result = {};
//     ESP_ERROR_CHECK(max30102_print_registers(&max30102));
//     while(true) {
        
//         //Update sensor, saving to "result"
//         ESP_ERROR_CHECK(max30102_update(&max30102, &result));
//         if(result.pulse_detected) {
//             printf("BEAT\n");
//             printf("BPM: %f | SpO2: %f%%\n", result.heart_bpm, result.spO2);
//             // m.heart = (int)result.heart_bpm*100;
//         }
//         //Update rate: 100Hz
//         vTaskDelay(10);
//     }
// }
void getsportStatus(void *pvParameters)
{
    float a;
    // int n=1;
    char flag = 0;
    while (1)
    {
        // if(!(xEventGroupGetBits(s_wifi_event_group)&PLAYSTATUS))
        // {
            a = angle_z;
            ESP_LOGI("angle","%f",a);
            if(a > CONFIG_EXAMPLE_MPUFUDU && flag == 1)
            {
                flag =0;
            }
            else if(a < (-CONFIG_EXAMPLE_MPUFUDU)  && flag == 0)
            {
                flag = 1;
                m.step++;
            }            
        // }

        vTaskDelay(100);
    }
}
 void setup_onewire_gpio(gpio_num_t gpio, uint8_t mode)
{
    gpio_set_direction(gpio, mode ? GPIO_MODE_INPUT_OUTPUT_OD : GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(gpio, GPIO_PULLUP_ONLY);
}

static void ds18b20_delay_ms(uint32_t msec)
{
	vTaskDelay(((msec) + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS);
	//vTaskDelay(msec / portTICK_PERIOD_MS);
}


void myds18b20_init(void* param)
{
    memset(&ds18b20, 0, sizeof(ds18x20_t));

	/* config the ds18b20 struct */
	ds18b20.one_wire.gpio_pin 		= 21;
	ds18b20.one_wire.gpio_set_level = gpio_set_level;
	ds18b20.one_wire.gpio_get_level = gpio_get_level;
	ds18b20.one_wire.gpio_setup 	= setup_onewire_gpio;
	ds18b20.one_wire.delay_us		= ets_delay_us;			// in freeRTOS, you should use another function, or not....test.
	ds18b20.delay_ms 				= ds18b20_delay_ms;
	gpio_set_pull_mode(21, GPIO_PULLUP_ONLY);
	int ret = ds18x20_scan_devices(&ds18b20, address_list, 1, &found);
	ESP_LOGI("11", "ds18b20 founds: %d, ret: %d, address: %08x%08x", found, ret, (uint32_t)(ds18b20.address >> 32), (uint32_t)(ds18b20.address) );
    while(1)
    {
        if(!(xEventGroupGetBits(s_wifi_event_group)&PLAYSTATUS))
        {
            ds18b20_measure_and_read(&ds18b20);
            ESP_LOGI("11", "ds18x20.temp: %f", ds18b20.temp);
            m.tem = ds18b20.temp*100;            
        }

        vTaskDelay(500);
    }
}


static void tcp_SAR(void* param)
{
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_efuse_read_field_blob(ESP_EFUSE_MAC_FACTORY, &mac, sizeof(mac) * 8));
    memcpy(m.Mac_id,mac,6);
    while(!(xEventGroupGetBits(s_wifi_event_group)&WIFI_CONNECTED_BIT))
    {
        vTaskDelay(50);        
    }
    xEventGroupClearBits(s_wifi_event_group,PLAYSTATUS);

    int err;
    int sock = 0;
    char ad[144];
    char d[512];
    while(1)
    {
        if((xEventGroupGetBits(s_wifi_event_group)&RESET_SOCK)&&(xEventGroupGetBits(s_wifi_event_group)&WIFI_CONNECTED_BIT))
        {
            char host_ip[] = "192.168.8.236";
            struct sockaddr_in dest_addr;
            dest_addr.sin_addr.s_addr = inet_addr(host_ip);
            dest_addr.sin_family = AF_INET; 
            dest_addr.sin_port = htons(CONFIG_EXAMPLE_PORT);
            int addr_family = 2;
    
            int ip_protocol = IPPROTO_IP;
            sock = socket(addr_family,SOCK_STREAM,ip_protocol);    
            // ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &on_wifi_disconnect, NULL));
            // ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_got_ip, NULL));
            if(sock < 0)
            {
                ESP_LOGE(TAG,"Unable to create socket: errno %d", sock);
            }
            ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, CONFIG_EXAMPLE_PORT);
            err = connect(sock,(struct sockaddr *)&dest_addr,sizeof(struct sockaddr_in6));
            if (err != 0) {
                ESP_LOGE(TAG, "Socket unable to connect: errno %d", sock);
            }
            ESP_LOGI(TAG, "Successfully connected");
            if(sock >= 0 && err == 0)
            {
                xEventGroupClearBits(s_wifi_event_group,RESET_SOCK);
            }
        }

        if((!(xEventGroupGetBits(s_wifi_event_group)&RESET_SOCK))&&(xEventGroupGetBits(s_wifi_event_group)&WIFI_CONNECTED_BIT))
        {
            ESP_LOGI("len","%d",sizeof(m));
            memcpy(ad,&m,sizeof(m));
            err = send(sock,ad,144,0);
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);  
                xEventGroupSetBits(s_wifi_event_group,RESET_SOCK);          
            }
        }
        ESP_LOGI("123x","111x");
        recv(sock,d,10,8);        
        ESP_LOGI("123x","%d,%d",d[0],d[1]);
        
        if(strstr(d,"2"))
        {
            xEventGroupSetBits(s_wifi_event_group,PLAYSTATUS);
            while(1)
            {
                send(sock,"20",2,0);
                int len = recv(sock,d,512,0);
                // for(int i = 0; i < len; i++)
                // {
                //     ESP_LOGI("data","%d",d[i]);
                // }
                if(strstr(d,"OKOKOK12354784888/"))
                {
                    xEventGroupClearBits(s_wifi_event_group,PLAYSTATUS);
                    ESP_LOGI("123456","wedas");
                    close(sock);
                    xEventGroupSetBits(s_wifi_event_group,RESET_SOCK);    
                    break;
                }
                xMessageBufferSend(xMessageBuffer,d,len,portMAX_DELAY);
                memset(d,0,512);
                vTaskDelay(1);
            }
        }
        memset(d,0,10);
        m.step = 0;
        // if(d[0] == '2')
        vTaskDelay(1500);
    }
}
static void mp3_play(void *param)
{
    char a[512];
    size_t len;
	
	VS1053_t dev;
	spi_master_init(&dev,10, 16, 4, CONFIG_GPIO_RESET);
	ESP_LOGI(pcTaskGetTaskName(0), "spi_master_init done");
	switchToMp3Mode(&dev);
	//setVolume(&dev, 100);
	ESP_LOGI(pcTaskGetTaskName(0), "CONFIG_VOLUME=%d", CONFIG_VOLUME);
	setVolume(&dev, 100);
    while(1)
    {
        if(xEventGroupGetBits(s_wifi_event_group)&PLAYSTATUS)
        {
            len = xMessageBufferReceive(xMessageBuffer,a,512,1000);
            playChunk(&dev, (uint8_t *)a, len);
            memset(a,0,512);
            printf("111\r\n");
        }  
        vTaskDelay(1);
    }
}

static void uart_re(void *param)
{
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, 120 * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, 19, 20, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    char* data = (char*) malloc(120);
    while(1)
    {
        if(!(xEventGroupGetBits(s_wifi_event_group)&PLAYSTATUS))
        {
            const int rxBytes = uart_read_bytes(UART_NUM_1, data, 120, 100 );
            if (rxBytes > 0) {

                data[rxBytes] = 0;
                char *str1 = strstr(data,"$GPRMC")?strstr(data,"$GPRMC"):NULL;
                char *str2 = str1?(strstr(str1,"\n")?strstr(str1,"\n"):NULL):NULL;
                if(str1!=NULL && str2!=NULL)
                {
                    memcpy(m.gps,str1,str2-str1);
                    m.gps[str2-str1] = 0;
                    m.len = str2-str1;
                    ESP_LOGI("gps","%s",m.gps);
                }
                memset(data,0,120);
            }                
        }
    
        vTaskDelay(1000);
    }
}

esp_err_t maxi2c_master_init(i2c_port_t i2c_port){
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 40;
    conf.scl_io_num = 41;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FRQ;
    i2c_param_config(i2c_port, &conf);
    return i2c_driver_install(i2c_port, I2C_MODE_MASTER, 0, 0, 0);
}

void get_bpm(void* param) {
    printf("MAX30102 Test\n");
    while(!xEventGroupGetBits(s_wifi_event_group)&WIFI_CONNECTED_BIT)
    vTaskDelay(50);
    max30102_data_t result = {};
    ESP_ERROR_CHECK(max30102_print_registers(&max30102));
    while(true) {
        //Update sensor, saving to "result"
        if((!(xEventGroupGetBits(s_wifi_event_group)&PLAYSTATUS)))
        {
            ESP_ERROR_CHECK(max30102_update(&max30102, &result));
            if(result.pulse_detected) {
                ESP_LOGI("c","ao");
                printf("BEAT\n");
                printf("BPM: %f | SpO2: %f%%\n", result.heart_bpm, result.spO2);
                m.heart = (int)result.heart_bpm*100;
            }            
        }

        //Update rate: 100Hz
        vTaskDelay(50/portTICK_PERIOD_MS);
    }
}

void app_main() {
    nvs_flash_init();
    esp_netif_init();
    // 
    memset(&m,0,144);
    ESP_ERROR_CHECK(maxi2c_master_init(1));
    // //Init sensor at I2C_NUM_0.
    // ESP_ERROR_CHECK(max30102_init( &max30102, 1,
    //                MAX30102_DEFAULT_OPERATING_MODE,
    //                MAX30102_DEFAULT_SAMPLING_RATE,
    //                MAX30102_DEFAULT_LED_PULSE_WIDTH,
    //                MAX30102_DEFAULT_IR_LED_CURRENT,
    //                MAX30102_DEFAULT_START_RED_LED_CURRENT,
    //                MAX30102_DEFAULT_MEAN_FILTER_SIZE,
    //                MAX30102_DEFAULT_PULSE_BPM_SAMPLE_SIZE,
    //                MAX30102_DEFAULT_ADC_RANGE, 
    //                MAX30102_DEFAULT_SAMPLE_AVERAGING,
    //                MAX30102_DEFAULT_ROLL_OVER,
    //                MAX30102_DEFAULT_ALMOST_FULL,
    //                false ));
    s_wifi_event_group = xEventGroupCreate();
    configASSERT( s_wifi_event_group );
    xMessageBuffer = xMessageBufferCreate(xMessageBufferSize);
	configASSERT( xMessageBuffer );

    // printf("%d",GY521_Init());

    m.start = 0x05;
    m.end = sizeof(m);
    // vTaskDelay(1000);
    xTaskCreate(&wifi_init_sta, "connecting",512*6,NULL,5,NULL);
    xTaskCreate(&tcp_SAR,"tcp send recieve",512*8,NULL,5,NULL);
    xTaskCreate(&mp3_play,"mp3 play",512*6,NULL,5,NULL);
    // xTaskCreate(&uart_re,"UART receive",512*5,NULL,5,NULL);
    // xTaskCreate(&ga6_task,"ga6",1024*5,NULL,5,NULL);
    // xTaskCreate(&getsportStatus, "sports",1024*5,NULL,8,NULL);
    // xTaskCreate(&get_bpm, "Get BPM", 1024*10, NULL, 5, NULL);
    // xTaskCreate(&myds18b20_init, "tem", 1024*8, NULL, 5, NULL);
}
 