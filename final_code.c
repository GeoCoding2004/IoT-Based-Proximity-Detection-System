#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include <esp_http_server.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/api.h>
#include <lwip/netdb.h>
#include "driver/ledc.h"
#include "rom/gpio.h"
#include "esp_timer.h"
#include <inttypes.h>

#define MAX_MEASUREMENTS 6000 // number of measurements to store

typedef struct {
    uint32_t distance;  // distance in cm
    uint32_t timestamp; // timestamp in seconds since start
} measurement_t;

static measurement_t measurements[MAX_MEASUREMENTS];
static size_t measurement_count = 0; 
static size_t measurement_start = 0; 

static uint64_t start_time_us = 0;

static uint32_t freq = 2000;
static uint32_t buzzer_duty = 128;

volatile uint32_t duration = 0;
uint32_t current_distance = 0;

static const char *TAG = "espressif";
int state = 0;
int armed = 1;

#define EXAMPLE_ESP_WIFI_SSID "gg"
#define EXAMPLE_ESP_WIFI_PASS "george123"
#define EXAMPLE_ESP_MAXIMUM_RETRY 5

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num = 0;

#define TRIG 23
#define ECHO 21
#define BUZZER_PIN 22

void reconfigure_buzzer(uint32_t frequency, uint32_t duty);

char main_page[] = "<!DOCTYPE html><html><head><title>Main Page</title><style>body {font-family: Arial, sans-serif; text-align:center; margin:0; padding:0; background:#f4f4f9;} h1 {margin-top:50px;} button {background-color:#4CAF50; color:white; padding:10px 20px; border:none; border-radius:4px; cursor:pointer; margin:10px;} button:hover {background-color:#45a049;} .slider-container {margin-top:40px;} .slider-container input[type='range'] {width:300px;} .slider-container label {display:block; margin:20px 0 10px; font-weight:bold;} </style></head><body><h1>Armed Page</h1><p>The system is armed.</p><p>Current Distance: <span id='distance'>--</span> cm</p><a href='/armed_page'><button>ARM</button></a><a href=\"/disarm_page\"><button>DISARM</button></a><a href=\"/sensor_page\"><button>Go to Sensor Page</button></a><div class='slider-container'><label for='volume'>Buzzer Volume (Duty):</label><input type='range' id='volume' min='0' max='255' value='128' step='1'><span id='volume_val'>128</span><br><label for='frequency'>Buzzer Frequency (Hz):</label><input type='range' id='frequency' min='100' max='5000' value='2000' step='100'><span id='freq_val'>2000</span><br></div><script>function updateDistance(){fetch('/sensor_data').then(r=>r.text()).then(d=>{document.getElementById('distance').innerText=d;});} setInterval(updateDistance,1000);function updateBuzzer(){const vol=document.getElementById('volume').value;const freq=document.getElementById('frequency').value;fetch('/set_buzzer?volume='+vol+'&frequency='+freq);document.getElementById('volume_val').innerText=vol;document.getElementById('freq_val').innerText=freq;} document.getElementById('volume').addEventListener('input', updateBuzzer);document.getElementById('frequency').addEventListener('input', updateBuzzer);</script></body></html>";

char sensor_page[] = "<!DOCTYPE html><html><head><title>Sensor Graph</title><style>body {font-family:Arial,sans-serif; text-align:center; margin:0; padding:0; background:#f4f4f9;} h1 {margin-top:50px;} button {background-color:#4CAF50;color:white;padding:10px 20px;border:none;border-radius:4px;cursor:pointer;margin-top:20px;} button:hover {background-color:#45a049;} canvas {border:1px solid #ccc; margin-top:20px; display:block; margin:20px auto;} table {margin:20px auto; border-collapse: collapse; font-size:14px;} th, td {border:1px solid #ddd; padding:8px;} th {background-color:#f2f2f2;} </style></head><body><h1>Distance Measurements Over Time</h1><p>This page shows all recorded distance measurements since the device started.</p><canvas id='chart' width='700' height='400'></canvas><h2>Measurements Table</h2><table id='dataTable'><tr><th>Time (s)</th><th>Distance (cm)</th></tr></table><script>let dataPoints=[];let maxDistance=1;function fetchHistory(){return fetch('/sensor_history').then(r=>r.json()).then(data => {dataPoints=data;updateMax();drawChart();buildTable();}).catch(err => {console.error('Error fetching history:', err);drawChart();});} setInterval(fetchHistory, 5000);function updateMax(){maxDistance=1;for(let i=0;i<dataPoints.length;i++){if(dataPoints[i].distance>maxDistance){maxDistance=dataPoints[i].distance;}}} function drawChart(){const canvas=document.getElementById('chart');const ctx=canvas.getContext('2d');ctx.clearRect(0,0,canvas.width,canvas.height);ctx.strokeStyle='#000';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(50,10);ctx.lineTo(50,canvas.height-50);ctx.lineTo(canvas.width-20,canvas.height-50);ctx.stroke();ctx.font='16px Arial';ctx.fillStyle='#000';ctx.fillText('Distance (cm)',60,20);ctx.fillText('Time (s)',canvas.width-60,canvas.height-30);ctx.font='20px Arial';ctx.fillText('Distance Over Time', canvas.width/2 - 80, 40);if(dataPoints.length===0){ctx.font='16px Arial';ctx.fillStyle='#555';ctx.fillText('No data available yet', canvas.width/2 - 60, canvas.height/2);return;} ctx.beginPath();ctx.strokeStyle='#007acc';ctx.lineWidth=2;let xStep=(canvas.width-70)/(dataPoints.length-1);let yScale=(canvas.height-60)/maxDistance;ctx.moveTo(50,canvas.height-50-(dataPoints[0].distance*yScale));for(let i=1;i<dataPoints.length;i++){let x=50+i*xStep;let y=canvas.height-50-(dataPoints[i].distance*yScale);ctx.lineTo(x,y);}ctx.stroke();} function buildTable(){const table=document.getElementById('dataTable');while(table.rows.length>1){table.deleteRow(1);}for(let i=0;i<dataPoints.length;i++){let row=table.insertRow();let cellTime=row.insertCell();let cellDist=row.insertCell();cellTime.textContent=dataPoints[i].time;cellDist.textContent=dataPoints[i].distance;}} fetchHistory();</script></body></html>";

char login_page[] = "<!DOCTYPE html><html><head><title>Secure Login</title><style>body {font-family: Arial, sans-serif; text-align:center; margin:0; padding:0; background:#f4f4f9;} h1 {margin-top:50px;}.card {background:#fff; box-shadow:2px 2px 12px 1px rgba(140,140,140,0.5); max-width:400px; margin:50px auto; padding:20px; border-radius:8px;} .card-title {font-size:1.2rem; font-weight:bold; color:#034078; margin-bottom:20px;} input[type='text'], input[type='password'] {width:90%; padding:10px; margin:10px 0; border:1px solid #ddd; border-radius:4px;} .button {display:inline-block; background:#4CAF50; border:none; border-radius:4px; color:white; padding:16px 40px; font-size:16px; margin-top:20px; cursor:pointer;} .button:hover {background:#45a049;} #error-message{color:red; margin-top:10px;} </style></head><body><h1>Secure Login</h1><div class='card'><p class='card-title'>This system provides an added layer of security for your valuable objects.</p><form action='/secure' method='GET'><label for='username'>Username:</label><br><input type='text' id='username' name='username' required><br><label for='password'>Password:</label><br><input type='password' id='password' name='password' required><br><button type='submit' class='button'>Login</button></form></div></body></html>";

char disarm_page[] = "<!DOCTYPE html><html><head><title>Main Page</title><style>body {font-family: Arial, sans-serif; text-align:center; margin:0; padding:0; background:#f4f4f9;} h1 {margin-top:50px;} button {background-color:#4CAF50; color:white; padding:10px 20px; border:none; border-radius:4px; cursor:pointer; margin:10px;} button:hover {background-color:#45a049;} .slider-container {margin-top:40px;} .slider-container input[type='range'] {width:300px;} .slider-container label {display:block; margin:20px 0 10px; font-weight:bold;} </style></head><body><h1>Disarmed Page</h1><p>The system is disarmed.</p><p>Current Distance: <span id='distance'>--</span> cm</p><a href='/armed_page'><button>ARM</button></a><a href=\"/disarm_page\"><button>DISARM</button></a><a href=\"/sensor_page\"><button>Go to Sensor Page</button></a><div class='slider-container'><label for='volume'>Buzzer Volume (Duty):</label><input type='range' id='volume' min='0' max='255' value='128' step='1'><span id='volume_val'>128</span><br><label for='frequency'>Buzzer Frequency (Hz):</label><input type='range' id='frequency' min='100' max='5000' value='2000' step='100'><span id='freq_val'>2000</span><br></div><script>function updateDistance(){fetch('/sensor_data').then(r=>r.text()).then(d=>{document.getElementById('distance').innerText=d;});} setInterval(updateDistance,1000);function updateBuzzer(){const vol=document.getElementById('volume').value;const freq=document.getElementById('frequency').value;fetch('/set_buzzer?volume='+vol+'&frequency='+freq);document.getElementById('volume_val').innerText=vol;document.getElementById('freq_val').innerText=freq;} document.getElementById('volume').addEventListener('input', updateBuzzer);document.getElementById('frequency').addEventListener('input', updateBuzzer);</script></body></html>";

char armed_page[] = "<!DOCTYPE html><html><head><title>Main Page</title><style>body {font-family: Arial, sans-serif; text-align:center; margin:0; padding:0; background:#f4f4f9;} h1 {margin-top:50px;} button {background-color:#4CAF50; color:white; padding:10px 20px; border:none; border-radius:4px; cursor:pointer; margin:10px;} button:hover {background-color:#45a049;} .slider-container {margin-top:40px;} .slider-container input[type='range'] {width:300px;} .slider-container label {display:block; margin:20px 0 10px; font-weight:bold;} </style></head><body><h1>Armed Page</h1><p>The system is armed.</p><p>Current Distance: <span id='distance'>--</span> cm</p><a href='/armed_page'><button>ARM</button></a><a href=\"/disarm_page\"><button>DISARM</button></a><a href=\"/sensor_page\"><button>Go to Sensor Page</button></a><div class='slider-container'><label for='volume'>Buzzer Volume (Duty):</label><input type='range' id='volume' min='0' max='255' value='128' step='1'><span id='volume_val'>128</span><br><label for='frequency'>Buzzer Frequency (Hz):</label><input type='range' id='frequency' min='100' max='5000' value='2000' step='100'><span id='freq_val'>2000</span><br></div><script>function updateDistance(){fetch('/sensor_data').then(r=>r.text()).then(d=>{document.getElementById('distance').innerText=d;});} setInterval(updateDistance,1000);function updateBuzzer(){const vol=document.getElementById('volume').value;const freq=document.getElementById('frequency').value;fetch('/set_buzzer?volume='+vol+'&frequency='+freq);document.getElementById('volume_val').innerText=vol;document.getElementById('freq_val').innerText=freq;} document.getElementById('volume').addEventListener('input', updateBuzzer);document.getElementById('frequency').addEventListener('input', updateBuzzer);</script></body></html>";


static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event=(ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG,"got ip:" IPSTR,IP2STR(&event->ip_info.ip));
        s_retry_num=0;
        xEventGroupSetBits(s_wifi_event_group,WIFI_CONNECTED_BIT);
    }
}

void connect_wifi(void)
{
    s_wifi_event_group=xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg=WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,ESP_EVENT_ANY_ID,&event_handler,NULL,&instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,IP_EVENT_STA_GOT_IP,&event_handler,NULL,&instance_got_ip));

    wifi_config_t wifi_config={
        .sta={
            .ssid=EXAMPLE_ESP_WIFI_SSID,
            .password=EXAMPLE_ESP_WIFI_PASS,
            .threshold.authmode=WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA,&wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG,"wifi_init_sta finished.");
    EventBits_t bits=xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT|WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);
    if(bits & WIFI_CONNECTED_BIT){
        ESP_LOGI(TAG,"connected to ap SSID:%s password:%s",EXAMPLE_ESP_WIFI_SSID,EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG,"Failed to connect to SSID:%s, password:%s",EXAMPLE_ESP_WIFI_SSID,EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG,"UNEXPECTED EVENT");
    }
    vEventGroupDelete(s_wifi_event_group);
}

void measure() {
    gpio_set_level(TRIG,1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(TRIG,0);
}

void IRAM_ATTR distance_isr(void* arg) {
    static uint32_t start_time=0;
    if(gpio_get_level(ECHO)==1) {
        start_time=xthal_get_ccount();
    } else {
        uint32_t end_time=xthal_get_ccount();
        duration=(end_time - start_time)/(CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ);
    }
}

void init_hw(){
    gpio_pad_select_gpio(TRIG);
    gpio_set_direction(TRIG,GPIO_MODE_OUTPUT);
    gpio_set_level(TRIG,0);

    gpio_pad_select_gpio(ECHO);
    gpio_set_direction(ECHO,GPIO_MODE_INPUT);
    gpio_set_pull_mode(ECHO,GPIO_PULLDOWN_ONLY);
    gpio_set_intr_type(ECHO,GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ECHO,distance_isr,NULL);

    reconfigure_buzzer(freq,buzzer_duty);

    start_time_us = esp_timer_get_time();
}

void sensor_task(void *pvParameters) {
    while(1){
        measure();
        vTaskDelay(pdMS_TO_TICKS(60));
        uint32_t distance_cm= (uint32_t)(duration*0.0343/2.0);
        current_distance=distance_cm;

        uint64_t now_us = esp_timer_get_time();
        uint32_t timestamp_s = (uint32_t)((now_us - start_time_us)/1000000ULL);

        measurements[(measurement_start+measurement_count)%MAX_MEASUREMENTS].distance = distance_cm;
        measurements[(measurement_start+measurement_count)%MAX_MEASUREMENTS].timestamp = timestamp_s;

        if(measurement_count<MAX_MEASUREMENTS) {
            measurement_count++;
        } else {
            measurement_start=(measurement_start+1)%MAX_MEASUREMENTS;
        }

        ESP_LOGI(TAG,"Armed=%d, Distance=%u", armed, (unsigned)distance_cm);

        if((armed ==1) && (distance_cm < 30 || distance_cm >=60)) {
            ESP_LOGI(TAG,"Buzzer ON");
            ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,buzzer_duty);
        } else {
            ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,0);
        }
        ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void reconfigure_buzzer(uint32_t frequency,uint32_t duty) {
    freq=frequency;
    buzzer_duty=duty;

    ledc_timer_config_t ledc_timer={
        .speed_mode=LEDC_LOW_SPEED_MODE,
        .timer_num=LEDC_TIMER_0,
        .duty_resolution=LEDC_TIMER_8_BIT,
        .freq_hz=freq,
        .clk_cfg=LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel={
        .speed_mode=LEDC_LOW_SPEED_MODE,
        .channel=LEDC_CHANNEL_0,
        .timer_sel=LEDC_TIMER_0,
        .intr_type=LEDC_INTR_DISABLE,
        .gpio_num=BUZZER_PIN,
        .duty=0,
        .hpoint=0,
    };
    ledc_channel_config(&ledc_channel);

    ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,buzzer_duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0);
}

// HTTP Handlers
esp_err_t send_web_page(httpd_req_t *req) {
    if(state==0) {
        return httpd_resp_send(req,login_page,HTTPD_RESP_USE_STRLEN);
    } else if(state==1) {
        return httpd_resp_send(req,main_page,HTTPD_RESP_USE_STRLEN);
    } else if(state==2) {
        return httpd_resp_send(req,sensor_page,HTTPD_RESP_USE_STRLEN);
    } else if(state==3) {
        return httpd_resp_send(req,disarm_page,HTTPD_RESP_USE_STRLEN);
    } else if(state==4) {
        return httpd_resp_send(req,armed_page,HTTPD_RESP_USE_STRLEN);
    }else{
        return httpd_resp_send(req,main_page,HTTPD_RESP_USE_STRLEN);
    }
}

esp_err_t get_req_handler(httpd_req_t *req) {
    return send_web_page(req);
}

esp_err_t secure_handler(httpd_req_t *req){
    char query[100];
    char username[20]={0};
    char password[20]={0};

    if(httpd_req_get_url_query_str(req,query,sizeof(query))==ESP_OK) {
        httpd_query_key_value(query,"username",username,sizeof(username));
        httpd_query_key_value(query,"password",password,sizeof(password));
    }

    if(strcmp(username,"admin")==0 && strcmp(password,"password123")==0) {
        state=1;
        return send_web_page(req);
    } else {
        state=0;
        const char *error_msg="Invalid credentials. <a href='/'>Return to Login</a>";
        return httpd_resp_send(req,error_msg,strlen(error_msg));
    }
}


esp_err_t sensor_page_handler(httpd_req_t *req)
{
    state=2;
    return send_web_page(req);
}

esp_err_t disarm_page_handler(httpd_req_t *req) {
    armed = 0;
    state = 3; 
    return send_web_page(req);
}

esp_err_t armed_page_handler(httpd_req_t *req) {
    armed = 1;
    state = 4; 
    return send_web_page(req);
}

esp_err_t sensor_data_handler(httpd_req_t *req)
{
    char data_str[32];
    snprintf(data_str,sizeof(data_str),"%lu",(unsigned long)current_distance);
    httpd_resp_set_type(req,"text/plain");
    return httpd_resp_send(req,data_str,HTTPD_RESP_USE_STRLEN);
}

esp_err_t sensor_history_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req,"application/json");
    httpd_resp_set_hdr(req,"Access-Control-Allow-Origin","*");

    size_t total=measurement_count;
    size_t alloc_size = total*50+2; 
    char *buf=malloc(alloc_size); 
    if(!buf) {
        return httpd_resp_send_500(req);
    }
    char *p=buf;
    *p++='[';
    for(size_t i=0;i<total;i++){
        measurement_t m = measurements[(measurement_start+i)%MAX_MEASUREMENTS];
        int len = sprintf(p, "{\"time\":%" PRIu32 ",\"distance\":%" PRIu32 "}", m.timestamp, m.distance);
        p+=len;
        if(i<total-1) {
            *p++=',';
        }
    }
    *p++=']';
    *p='\0';

    esp_err_t res=httpd_resp_send(req,buf,strlen(buf));
    free(buf);
    return res;
}

esp_err_t set_buzzer_handler(httpd_req_t *req)
{
    char query[100];
    char volume_str[10]={0};
    char freq_str[10]={0};
    uint32_t new_vol=buzzer_duty;
    uint32_t new_freq=freq;

    if(httpd_req_get_url_query_str(req,query,sizeof(query))==ESP_OK) {
        httpd_query_key_value(query,"volume",volume_str,sizeof(volume_str));
        httpd_query_key_value(query,"frequency",freq_str,sizeof(freq_str));
    }

    if(strlen(volume_str)>0) {
        new_vol=atoi(volume_str);
    }
    if(strlen(freq_str)>0) {
        new_freq=atoi(freq_str);
    }

    reconfigure_buzzer(new_freq,new_vol);

    httpd_resp_set_type(req,"text/plain");
    return httpd_resp_send(req,"OK",HTTPD_RESP_USE_STRLEN);
}


// URI handlers
httpd_uri_t uri_get = {
    .uri="/",
    .method=HTTP_GET,
    .handler=get_req_handler,
    .user_ctx=NULL
};

httpd_uri_t uri_on = {
    .uri="/secure",
    .method=HTTP_GET,
    .handler=secure_handler,
    .user_ctx=NULL
};

httpd_uri_t uri_disarm_page = {
    .uri = "/disarm_page",
    .method = HTTP_GET,
    .handler = disarm_page_handler,
    .user_ctx = NULL
};

httpd_uri_t uri_sensor_page = {
    .uri="/sensor_page",
    .method=HTTP_GET,
    .handler=sensor_page_handler,
    .user_ctx=NULL
};

httpd_uri_t uri_armed_page = {
    .uri = "/armed_page",
    .method = HTTP_GET,
    .handler = armed_page_handler,
    .user_ctx = NULL
};

httpd_uri_t uri_sensor_data = {
    .uri="/sensor_data",
    .method=HTTP_GET,
    .handler=sensor_data_handler,
    .user_ctx=NULL
};

httpd_uri_t uri_sensor_history = {
    .uri="/sensor_history",
    .method=HTTP_GET,
    .handler=sensor_history_handler,
    .user_ctx=NULL
};

httpd_uri_t uri_set_buzzer = {
    .uri="/set_buzzer",
    .method=HTTP_GET,
    .handler=set_buzzer_handler,
    .user_ctx=NULL
};


httpd_handle_t setup_server(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_get);
        httpd_register_uri_handler(server, &uri_on);
        httpd_register_uri_handler(server, &uri_sensor_page);
        httpd_register_uri_handler(server, &uri_sensor_data);
        httpd_register_uri_handler(server, &uri_sensor_history);
        httpd_register_uri_handler(server, &uri_set_buzzer);
        httpd_register_uri_handler(server, &uri_disarm_page); 
        httpd_register_uri_handler(server, &uri_armed_page);
    }

    return server;
}

void app_main()
{
    esp_err_t ret=nvs_flash_init();
    if(ret==ESP_ERR_NVS_NO_FREE_PAGES||ret==ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret=nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG,"ESP_WIFI_MODE_STA");
    connect_wifi();

    init_hw();
    xTaskCreate(sensor_task,"sensor_task",2048,NULL,5,NULL);

    state=0;
    ESP_LOGI(TAG,"Web Server is running ...");
    setup_server();
}
