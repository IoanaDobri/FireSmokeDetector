#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "esp_log.h"
#include "esp_err.h"
#include "rom/ets_sys.h"

// ==== WIFI ====
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"

// ==== HTTP SERVER + WEBSOCKET ====
#include "esp_http_server.h"

// ------------------------------------------------------
// CONFIG
// ------------------------------------------------------

// DHT11 DATA pin
#define DHT_GPIO                  GPIO_NUM_32   // DATA

// MQ2 (AO -> GPIO34)
#define MQ2_ADC_CHANNEL           ADC1_CHANNEL_6   // GPIO34
#define MQ2_ADC_WIDTH             ADC_WIDTH_BIT_12
#define MQ2_ADC_ATTEN             ADC_ATTEN_DB_11   // 0..~3.3V
#define MQ2_SMOKE_THRESHOLD       1800             // <-- ajustezi din teste (0..4095)

// LED + BUZZER
#define LED_GPIO                  GPIO_NUM_27
#define BUZZER_GPIO               GPIO_NUM_25

// Prag alarma temperatura
#define TEMP_ALARM_THRESHOLD_C    24

// I2C / OLED
#define I2C_MASTER_SDA_IO         21
#define I2C_MASTER_SCL_IO         22
#define I2C_MASTER_NUM            I2C_NUM_0
#define I2C_MASTER_FREQ_HZ        400000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

#define OLED_ADDR                 0x3C

// WIFI CONFIG (hotspot iPhone)
#define WIFI_SSID                 "ESP32TEST"
#define WIFI_PASS                 "ioana1234"

static const char *TAG_SENSOR = "SENSOR_TASK";
static const char *TAG_COMM   = "COMM_TASK";
static const char *TAG_DHT    = "DHT";
static const char *TAG_OLED   = "OLED";
static const char *TAG_WIFI   = "WIFI";
static const char *TAG_MAIN   = "MAIN";
static const char *TAG_WS     = "WS";
static const char *TAG_MQ2    = "MQ2";

// wifi ready flag
static bool s_wifi_ready = false;

// ------------------------------------------------------
// Structura de date
// ------------------------------------------------------
typedef struct {
    float    temperature;
    float    humidity;
    int      smoke_level;     // 0..4095 (raw/filtrat)
    int      alarm_state;     // 0=OK, 2=ALARM
    uint32_t sample_id;
} system_data_t;

static QueueHandle_t xSystemDataQueue = NULL;

// ------------------------------------------------------
// WebSocket – variabile globale
// ------------------------------------------------------
static httpd_handle_t ws_server    = NULL;
static int            ws_client_fd = -1;  // ultimul client conectat

// ------------------------------------------------------
// I2C + OLED helpers
// ------------------------------------------------------

static esp_err_t oled_send_command(uint8_t cmd)
{
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, 0x00, true);   // command
    i2c_master_write_byte(cmd_handle, cmd, true);
    i2c_master_stop(cmd_handle);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd_handle);
    return ret;
}

static esp_err_t oled_send_data(const uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, 0x40, true);   // data
    i2c_master_write(cmd_handle, (uint8_t *)data, len, true);
    i2c_master_stop(cmd_handle);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd_handle);
    return ret;
}

static void oled_init(void)
{
    oled_send_command(0xAE);
    oled_send_command(0x20);
    oled_send_command(0x00);
    oled_send_command(0xB0);
    oled_send_command(0xC8);
    oled_send_command(0x00);
    oled_send_command(0x10);
    oled_send_command(0x40);
    oled_send_command(0x81);
    oled_send_command(0x7F);
    oled_send_command(0xA1);
    oled_send_command(0xA6);
    oled_send_command(0xA8);
    oled_send_command(0x3F);
    oled_send_command(0xA4);
    oled_send_command(0xD3);
    oled_send_command(0x00);
    oled_send_command(0xD5);
    oled_send_command(0x80);
    oled_send_command(0xD9);
    oled_send_command(0xF1);
    oled_send_command(0xDA);
    oled_send_command(0x12);
    oled_send_command(0xDB);
    oled_send_command(0x40);
    oled_send_command(0x8D);
    oled_send_command(0x14);
    oled_send_command(0xAF);
}

static void oled_clear(void)
{
    uint8_t buffer[128];
    memset(buffer, 0x00, sizeof(buffer));

    for (int page = 0; page < 8; page++) {
        oled_send_command(0xB0 | page);
        oled_send_command(0x00);
        oled_send_command(0x10);
        oled_send_data(buffer, sizeof(buffer));
    }
}

// font 5x7 (exact ca la tine)
static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00},
    {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00},
    {0x14,0x08,0x3E,0x08,0x14}, {0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08},
    {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E},
    {0x00,0x36,0x36,0x00,0x00}, {0x00,0x56,0x36,0x00,0x00},
    {0x08,0x14,0x22,0x41,0x00}, {0x14,0x14,0x14,0x14,0x14},
    {0x00,0x41,0x22,0x14,0x08}, {0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E}, {0x7E,0x11,0x11,0x11,0x7E},
    {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41},
    {0x7F,0x09,0x09,0x09,0x01}, {0x3E,0x41,0x49,0x49,0x7A},
    {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40}, {0x7F,0x02,0x0C,0x02,0x7F},
    {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E},
    {0x7F,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F}, {0x7F,0x20,0x18,0x20,0x7F},
    {0x63,0x14,0x08,0x14,0x63}, {0x03,0x04,0x78,0x04,0x03},
    {0x61,0x51,0x49,0x45,0x43}
};

static void oled_draw_char(uint8_t col, uint8_t page, char c)
{
    if (c < 0x20 || c > 0x5A) c = ' ';
    uint8_t index = (uint8_t)c - 0x20;
    uint8_t data[6];

    for (int i = 0; i < 5; i++) data[i] = font5x7[index][i];
    data[5] = 0x00;

    oled_send_command(0xB0 | page);
    oled_send_command(0x00 | (col & 0x0F));
    oled_send_command(0x10 | (col >> 4));
    oled_send_data(data, sizeof(data));
}

static void oled_draw_text(uint8_t col, uint8_t page, const char *text)
{
    while (*text) {
        oled_draw_char(col, page, *text++);
        col += 6;
        if (col > 122) break;
    }
}

// ------------------------------------------------------
// DHT11 driver (exact ca la tine)
// ------------------------------------------------------
static esp_err_t dht11_read(int *humidity, int *temperature)
{
    uint8_t data[5] = {0};
    int byte_index = 0;
    int bit_index  = 7;

    gpio_set_direction(DHT_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_GPIO, 1);
    gpio_set_pull_mode(DHT_GPIO, GPIO_PULLUP_ONLY);

    gpio_set_level(DHT_GPIO, 0);
    ets_delay_us(18000);
    gpio_set_level(DHT_GPIO, 1);
    ets_delay_us(40);
    gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT);

    int timeout = 0;

    while (gpio_get_level(DHT_GPIO) == 1) {
        if (++timeout > 20000) return ESP_FAIL;
        ets_delay_us(1);
    }

    timeout = 0;
    while (gpio_get_level(DHT_GPIO) == 0) {
        if (++timeout > 20000) return ESP_FAIL;
        ets_delay_us(1);
    }

    timeout = 0;
    while (gpio_get_level(DHT_GPIO) == 1) {
        if (++timeout > 20000) return ESP_FAIL;
        ets_delay_us(1);
    }

    for (int i = 0; i < 40; i++) {
        timeout = 0;
        while (gpio_get_level(DHT_GPIO) == 0) {
            if (++timeout > 20000) return ESP_FAIL;
            ets_delay_us(1);
        }

        int high_time = 0;
        while (gpio_get_level(DHT_GPIO) == 1) {
            if (++high_time > 20000) return ESP_FAIL;
            ets_delay_us(1);
        }

        if (high_time > 40) data[byte_index] |= (1 << bit_index);

        if (bit_index == 0) { bit_index = 7; byte_index++; }
        else bit_index--;
    }

    uint8_t checksum = data[0] + data[1] + data[2] + data[3];
    if (checksum != data[4]) return ESP_FAIL;

    *humidity    = data[0];
    *temperature = data[2];
    return ESP_OK;
}

// ------------------------------------------------------
// MQ2 – init + read (GPIO34)
// ------------------------------------------------------
static void mq2_init(void)
{
    adc1_config_width(MQ2_ADC_WIDTH);
    adc1_config_channel_atten(MQ2_ADC_CHANNEL, MQ2_ADC_ATTEN);
    ESP_LOGI(TAG_MQ2, "MQ2 init: AO pe GPIO34 (ADC1_CH6), atten=11dB");
}

// mic filtru: media pe 8 citiri
static int mq2_read_smoke_level(void)
{
    int sum = 0;
    const int N = 8;
    for (int i = 0; i < N; i++) {
        sum += adc1_get_raw(MQ2_ADC_CHANNEL);
        ets_delay_us(2000);
    }
    return sum / N; // 0..4095
}

// ------------------------------------------------------
// WIFI
// ------------------------------------------------------
static void wifi_event_handler(void* arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG_WIFI, "Connecting to WiFi...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* disc = (wifi_event_sta_disconnected_t*) event_data;
        ESP_LOGW(TAG_WIFI, "Disconnected, reason=%d", disc->reason);
        s_wifi_ready = false;
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG_WIFI, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_ready = true;
    }
}

static void wifi_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
                                               ESP_EVENT_ANY_ID,
                                               &wifi_event_handler,
                                               NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,
                                               IP_EVENT_STA_GOT_IP,
                                               &wifi_event_handler,
                                               NULL));

    wifi_config_t wifi_config = { 0 };
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASS);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG_WIFI, "WiFi init finished");
}

// ------------------------------------------------------
// WEBSOCKET SERVER
// ------------------------------------------------------
#if CONFIG_HTTPD_WS_SUPPORT

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ws_client_fd = httpd_req_to_sockfd(req);
        ESP_LOGI(TAG_WS, "Client WebSocket conectat, fd=%d", ws_client_fd);
        return ESP_OK;
    }
    return ESP_OK;
}

static httpd_handle_t start_websocket_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 8080;

    httpd_handle_t server = NULL;
    esp_err_t ret = httpd_start(&server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_WS, "Nu pot porni HTTP server: %s", esp_err_to_name(ret));
        return NULL;
    }

    httpd_uri_t ws_uri = {
        .uri          = "/ws",
        .method       = HTTP_GET,
        .handler      = ws_handler,
        .user_ctx     = NULL,
        .is_websocket = true
    };

    ret = httpd_register_uri_handler(server, &ws_uri);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_WS, "Nu pot inregistra /ws: %s", esp_err_to_name(ret));
        httpd_stop(server);
        return NULL;
    }

    ESP_LOGI(TAG_WS, "WebSocket server pe ws://<ESP_IP>:%d/ws", config.server_port);
    return server;
}

static void ws_send(const char *msg)
{
    if (!s_wifi_ready || ws_client_fd < 0 || !ws_server) {
        return;
    }

    httpd_ws_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.type    = HTTPD_WS_TYPE_TEXT;
    frame.payload = (uint8_t*)msg;
    frame.len     = strlen(msg);
    frame.final   = true;

    esp_err_t ret = httpd_ws_send_frame_async(ws_server, ws_client_fd, &frame);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG_WS, "ws_send failed: %s", esp_err_to_name(ret));
    }
}

#else
static httpd_handle_t start_websocket_server(void) { return NULL; }
static void ws_send(const char *msg) { (void)msg; }
#endif

// ------------------------------------------------------
// TASK 1: citire senzori + alarmă + LED/buzzer
// ------------------------------------------------------
void sensor_local_task(void *pvParameters)
{
    uint32_t sample = 0;

    int last_temp = 20;
    int last_hum  = 40;
    bool have_valid_dht = false;

    while (1) {
        system_data_t msg;
        int hum = 0, temp = 0;

        // DHT11
        esp_err_t res = dht11_read(&hum, &temp);
        if (res == ESP_OK) {
            if (temp >= 0 && temp <= 80 && hum >= 0 && hum <= 100) {
                last_temp = temp;
                last_hum  = hum;
                have_valid_dht = true;
            } else {
                ESP_LOGW(TAG_SENSOR, "DHT valori invalide: T=%d H=%d", temp, hum);
            }
        } else {
            ESP_LOGW(TAG_SENSOR, "DHT11 read error");
        }

        // MQ2
        int smoke = mq2_read_smoke_level();

        // Umple structura
        msg.temperature = (float)last_temp;
        msg.humidity    = (float)last_hum;
        msg.smoke_level = smoke;
        msg.sample_id   = sample++;

        // ALARM logic: temperatura OR fum
        bool temp_alarm  = (have_valid_dht && last_temp >= TEMP_ALARM_THRESHOLD_C);
        bool smoke_alarm = (smoke >= MQ2_SMOKE_THRESHOLD);

        if (temp_alarm || smoke_alarm) msg.alarm_state = 2;
        else msg.alarm_state = 0;

        // LED + BUZZER
        gpio_set_level(LED_GPIO,    (msg.alarm_state != 0));
        gpio_set_level(BUZZER_GPIO, (msg.alarm_state != 0));

        // Queue
        if (xQueueSend(xSystemDataQueue, &msg, 0) == pdTRUE) {
            ESP_LOGI(TAG_SENSOR,
                     "Sent sample=%lu, T=%dC, H=%d%%, smoke=%d, alarm=%d",
                     msg.sample_id, last_temp, last_hum, smoke, msg.alarm_state);
        } else {
            ESP_LOGW(TAG_SENSOR, "Queue full, lost sample %lu", msg.sample_id);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// ------------------------------------------------------
// TASK 2: WebSocket + OLED
// ------------------------------------------------------
void comm_display_task(void *pvParameters)
{
    system_data_t rx;
    char json_buffer[200];
    char line1[22];
    char line2[22];
    char line3[22];

    while (1) {
        if (xQueueReceive(xSystemDataQueue, &rx, portMAX_DELAY)) {

            snprintf(json_buffer, sizeof(json_buffer),
                     "{\"sample_id\":%lu,\"temperature\":%.1f,"
                     "\"humidity\":%.1f,\"smoke\":%d,"
                     "\"alarm_state\":%d}",
                     rx.sample_id, rx.temperature,
                     rx.humidity, rx.smoke_level,
                     rx.alarm_state);

            ESP_LOGI(TAG_COMM, "Sending remote: %s", json_buffer);

            // WebSocket către browser/telefon
            ws_send(json_buffer);

            // OLED
            snprintf(line1, sizeof(line1), "T:%2.0fC H:%2.0f%%", rx.temperature, rx.humidity);
            snprintf(line2, sizeof(line2), "SMK:%4d TH:%4d", rx.smoke_level, MQ2_SMOKE_THRESHOLD);

            if (rx.alarm_state == 0) snprintf(line3, sizeof(line3), "Alarm: OK");
            else snprintf(line3, sizeof(line3), "Alarm: FIRE!");

            oled_clear();
            oled_draw_text(0, 0, "Fire Smoke Det.");
            oled_draw_text(0, 2, line1);
            oled_draw_text(0, 4, line2);
            oled_draw_text(0, 6, line3);

            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// ------------------------------------------------------
// app_main
// ------------------------------------------------------
void app_main(void)
{
    ESP_LOGI(TAG_MAIN, "Aplicatie pornita");

    // coada pentru comunicatia intre task-uri
    xSystemDataQueue = xQueueCreate(10, sizeof(system_data_t));
    if (xSystemDataQueue == NULL) {
        ESP_LOGE(TAG_MAIN, "Queue creation failed!");
        return;
    }

    // LED + BUZZER
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);

    gpio_reset_pin(BUZZER_GPIO);
    gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BUZZER_GPIO, 0);

    // MQ2 init (ADC)
    mq2_init();

    // I2C + OLED
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                                       I2C_MASTER_RX_BUF_DISABLE,
                                       I2C_MASTER_TX_BUF_DISABLE, 0));

    ESP_LOGI(TAG_OLED, "I2C initializat, pornesc OLED...");
    oled_init();
    oled_clear();
    oled_draw_text(0, 0, "Fire Smoke Det.");
    oled_draw_text(0, 2, "Init...");

    // WiFi
    ESP_LOGI(TAG_MAIN, "Init WiFi...");
    wifi_init();

    // WebSocket server
    ws_server = start_websocket_server();

    // Task-uri
    xTaskCreate(sensor_local_task, "sensor_local_task",
                4096, NULL, 5, NULL);

    xTaskCreate(comm_display_task, "comm_display_task",
                4096, NULL, 5, NULL);
}
