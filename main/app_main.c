/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Adrian Bradianu (github.com/abradianu)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"

#include "lwip/apps/sntp.h"
#include "i2c.h"
#include "display.h"
#include "sensors.h"
#include "cmd_recv.h"
#include "ota.h"
#include "nvs_utils.h"
#include "http_server.h"

#define CLOCK_DISPLAY

#define OTA_SERVER_IP                  "192.168.1.140"
#define OTA_SERVER_PORT                8070
#define OTA_FILENAME                   "esp8266_iot.bin"

/* Led is connected to GPIO16 on NodeMcu board */
#define GPIO_BLUE_LED                  16

/* OTA button GPIO */
#define GPIO_OTA_BUTTON                0

/* Brightness level button GPIO */
#define GPIO_DISPLAY_BRIGHTNESS_BUTTON 2

/* Show temperature GPIO */
#define GPIO_DISPLAY_TEMP_BUTTON       3

/* Show humidity GPIO */
#define GPIO_DISPLAY_HUMIDITY_BUTTON   14

/* I2C pins */
#define GPIO_I2C_MASTER_SCL            4
#define GPIO_I2C_MASTER_SDA            5

#define GPIO_SENSORS_WAKE_GPIO         15

#define I2C_SENSORS_BUS                0
#define I2C_CLOCK_FREQ                 100000

/* DISPLAY GPIO pins*/
#define GPIO_DISPLAY_CLK               13
#define GPIO_DISPLAY_DATA              12

/* DISPLAY clock frequency */
#define DISPLAY_CLOCK_FREQ             100000

/* Display task settings */
#define DISPLAY_TASK_PRIORITY          10
#define DISPLAY_TASK_STACK_SIZE        2048
#define DISPLAY_TASK_LOOP_DELAY        50    /* in ms */

/* Display blink rates in ms */
#define BLINK_RATE_WIFI_AP_MODE_ON     50
#define BLINK_RATE_WIFI_AP_MODE_OFF    950
#define BLINK_RATE_WIFI_NOT_CONNECTED  100
#define BLINK_RATE_TIME_NOT_SYNCED     200
#define BLINK_RATE_TIME_SYNCED         500

/* Main task settings */
#define MAIN_TASK_LOOP_DELAY           100
#define MAIN_TASK_PRIORITY             10
#define MAIN_TASK_STACK_SIZE           4096

/* OTA button press time for OTA to start in ms */
#define OTA_BUTTON_PRESS_TIME          5000

/* Brightness button press time for incrementing brightness level */
#define BRIGHTNESS_BUTTON_PRESS_TIME   800

/* Sensors read interval in ms */
#define SENSORS_READ_TIME              (2 * 60 * 1000)
#define SENSORS_SEND_RETRY             100

/*
 * std offset dst [offset],start[/time],end[/time]
 * There are no spaces in the specification. The initial std and offset specify
 * the standard timezone. The dst string and offset specify the name and offset
 * for the corresponding daylight saving timezone. If the offset is omitted,
 * it default to one hour ahead of standard time. The start field specifies
 * when daylight saving time goes into effect and the end field specifies when
 * the change is made back to standard time. These fields may have the following
 * formats: Mm.w.d
 * This specifies day d (0 <= d <= 6) of week w (1 <= w <= 5) of month m
 * (1 <= m <= 12). Week 1 is the first week in which day d occurs and week 5 is
 * the last week in which day d occurs. Day 0 is a Sunday. 
 */
#define TIMEZONE                       "EET-2EEST-3,M3.5.0,M10.5.0"

#define FATAL_ERROR(fmt, args...)                     \
do {                                                  \
    ESP_LOGE(TAG, fmt,## args);                       \
    ESP_LOGE(TAG, "Rebooting in 5 seconds");          \
    vTaskDelay(5000 / portTICK_RATE_MS);              \
    esp_restart();                                    \
} while (0)

typedef enum {
    WIFI_AP_MODE,
    WIFI_STA_DISCONNECTED,
    WIFI_STA_CONNECTED,
} wifi_state_t;

typedef enum {
    DISPLAY_MODE_CLOCK,
    DISPLAY_MODE_TEMPERATURE,
    DISPLAY_MODE_HUMIDITY,
    DISPLAY_MODE_CO2,
}display_mode_t;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
static const int CONNECTED_BIT = BIT0;

static const char *TAG = "DC";
static wifi_state_t wifi_state;

static void sntp_start(void)
{
    ESP_LOGI(TAG, "SNTP start");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;

        case SYSTEM_EVENT_STA_GOT_IP:
            wifi_state = WIFI_STA_CONNECTED;
            ESP_LOGI(TAG, "WiFi connected");
            break;

        case SYSTEM_EVENT_AP_STACONNECTED:
            ESP_LOGI(TAG, "station:"MACSTR" join, AID=%d",
                 MAC2STR(event->event_info.sta_connected.mac),
                 event->event_info.sta_connected.aid);
            break;

        case SYSTEM_EVENT_AP_STADISCONNECTED:
            ESP_LOGI(TAG, "station:"MACSTR"leave, AID=%d",
                 MAC2STR(event->event_info.sta_disconnected.mac),
                 event->event_info.sta_disconnected.aid);
            break;
            
        case SYSTEM_EVENT_STA_DISCONNECTED:
            /* This is a workaround as ESP8266 WiFi libs don't currently
               auto-reassociate. */
            esp_wifi_connect();
            wifi_state = WIFI_STA_DISCONNECTED;
            ESP_LOGI(TAG, "WiFi disconnected");
            break;

        default:
            break;
    }

    return ESP_OK;
}

static void display_ota_progress_cb(uint32_t progress)
{
#ifdef CLOCK_DISPLAY
    char buf[6];

    sprintf(buf, " %03d", progress);
    display_write(buf);
#endif
}

static display_mode_t display_buttons_check()
{
    static uint32_t ota_btn_press_time = 0;
    static uint32_t brightness_btn_press_time = 0;
    static TickType_t prev_tick_count = 0;
    uint32_t time_passed;
    uint8_t brightness_level;
    display_mode_t display_mode = DISPLAY_MODE_CLOCK;

    /* Time passed since previous call in ms */
    time_passed = (xTaskGetTickCount() - prev_tick_count) * portTICK_RATE_MS;
    prev_tick_count = xTaskGetTickCount();

    /* Check if the display temperature button is pressed. */
    if (gpio_get_level(GPIO_DISPLAY_TEMP_BUTTON) == 0) {
        ESP_LOGI(TAG, "Display temperature button pressed");
        display_mode = DISPLAY_MODE_TEMPERATURE;
    }

    /* Check if the display humidity button is pressed. */
    if (gpio_get_level(GPIO_DISPLAY_HUMIDITY_BUTTON) == 0) {
        ESP_LOGI(TAG, "Display humidity button pressed");
        display_mode = DISPLAY_MODE_HUMIDITY;
    }

    /* Check if display brightness button is pressed */
    if (gpio_get_level(GPIO_DISPLAY_BRIGHTNESS_BUTTON) == 0) {
        /* Brightness button pressed */
        ESP_LOGI(TAG, "Brightness level button pressed");

        brightness_btn_press_time += time_passed;
        if (brightness_btn_press_time >= BRIGHTNESS_BUTTON_PRESS_TIME) {
            brightness_btn_press_time = 0;

            /* Get brightness level from flash */
            nvs_get_u8(nvs_get_handle(), NVS_DISPLAY_BRIGHTNESS, &brightness_level);

            /* Increment brightness level */
            brightness_level ++;
            if (display_set_brightness(brightness_level) != ESP_OK) {
                brightness_level = 0;
                display_set_brightness(brightness_level);
            }

            /* Save new brightness level to the NVS flash */
            nvs_set_u8(nvs_get_handle(), NVS_DISPLAY_BRIGHTNESS, brightness_level);
        }
    } else {
        brightness_btn_press_time = 0;
    }

    /* Check if OTA button is pressed for more than 5 seconds */
    if (gpio_get_level(GPIO_OTA_BUTTON) == 0) {
        /* OTA button pressed*/
        ESP_LOGI(TAG, "OTA button pressed");

        ota_btn_press_time += time_passed;
        if (ota_btn_press_time >= OTA_BUTTON_PRESS_TIME) {
            ota_btn_press_time = 0;

            if (ota_start(OTA_SERVER_IP, OTA_SERVER_PORT, OTA_FILENAME,
                display_ota_progress_cb) == ESP_OK) {
                ESP_LOGI(TAG, "Rebooting...!");
                vTaskDelay(100 / portTICK_RATE_MS);
                esp_restart();
            }
        }
    } else {
        ota_btn_press_time = 0;
    }
    
    /*
     * Check if both OTA and brightness buttons are pressed and restart 
     * in AP or Station mode if so.
     */
    if (gpio_get_level(GPIO_DISPLAY_BRIGHTNESS_BUTTON) == 0 &&
        gpio_get_level(GPIO_OTA_BUTTON) == 0) {

        /* Restart in AP or Station mode depending on the current mode */
        if (nvs_set_u8(nvs_get_handle(),
                       NVS_WIFI_AP_MODE,
                       wifi_state == WIFI_AP_MODE ? 0 : 1) == ESP_OK) {
            ESP_LOGI(TAG, "Rebooting in %s mode...",
                          wifi_state == WIFI_AP_MODE ? "Station" : "AP");
            vTaskDelay(500 / portTICK_RATE_MS);
            esp_restart();
        }
    }
    return display_mode;
}

static void display_task(void *arg)
{
    uint8_t blink_state = 0;
    char disp_buf[6];
    display_mode_t display_mode;
    TickType_t prev_tick_count = 0;
    uint32_t time_passed;
    int delay = BLINK_RATE_WIFI_NOT_CONNECTED;
    struct tm timeinfo;
    time_t now;

    while (1) {
        display_mode = display_buttons_check();

        if (display_mode == DISPLAY_MODE_CLOCK) {
            /* Time passed since previous display write */
            time_passed = (xTaskGetTickCount() - prev_tick_count) * portTICK_RATE_MS;
            if (time_passed >= delay) {
                prev_tick_count = xTaskGetTickCount();

                time(&now);
                localtime_r(&now, &timeinfo);

                blink_state ^= 1;
                if (blink_state) {
                    strftime(disp_buf, sizeof(disp_buf), "%H%M", &timeinfo);
                } else {
                    strftime(disp_buf, sizeof(disp_buf), "%H:%M", &timeinfo);
                }

#ifdef CLOCK_DISPLAY
                if (wifi_state == WIFI_AP_MODE) {
                    blink_state ? display_write("    ") : display_write("8888");
                } else {
                    display_write(disp_buf);
                }
#else
                gpio_set_level(GPIO_BLUE_LED, blink_state);
#endif
                /* Set blink rate depending on the program state */
                if (wifi_state == WIFI_AP_MODE) {
                    if (blink_state)
                        delay = BLINK_RATE_WIFI_AP_MODE_OFF;
                    else
                        delay = BLINK_RATE_WIFI_AP_MODE_ON;
                } else if (wifi_state == WIFI_STA_DISCONNECTED) {
                    delay = BLINK_RATE_WIFI_NOT_CONNECTED;
                } else {
                    if (timeinfo.tm_year < (2019 - 1900)) {
                        /* Clock is not synced */
                        delay = BLINK_RATE_TIME_NOT_SYNCED;
                    } else {
                        /* Clock is now synced */

                        if (delay != BLINK_RATE_TIME_SYNCED)
                            ESP_LOGI(TAG, "Time synced: %s", disp_buf);
                        delay = BLINK_RATE_TIME_SYNCED;
                    }
                }
            }
        } else {  /* display_mode != DISPLAY_MODE_CLOCK */
            sensors_data_t sensors_data;

            if (sensors_get_data(&sensors_data) == ESP_OK) {
                if (display_mode == DISPLAY_MODE_TEMPERATURE)
                    sprintf(disp_buf, "%02d:%1d ", (int)sensors_data.temp, (int)(sensors_data.temp * 10) % 10);
                else if (display_mode == DISPLAY_MODE_HUMIDITY)
                    sprintf(disp_buf, "  %02d", (int)sensors_data.humidity);
                else
                    sprintf(disp_buf, "%04d", sensors_data.eco2);
            } else {
                sprintf(disp_buf, "8888");
            }

            display_write(disp_buf);

            /* Display the sensors info for at least 500ms */
            vTaskDelay(500 / portTICK_RATE_MS);
        }
        
        vTaskDelay(DISPLAY_TASK_LOOP_DELAY / portTICK_RATE_MS);
    }
}

static void main_task(void *arg)
{
    bool init_done = false;
    uint32_t sensors_send_info_time = 0;
    int send_retry = 0;

    if (xTaskCreate(display_task, "display_task", DISPLAY_TASK_STACK_SIZE,
        NULL, DISPLAY_TASK_PRIORITY, NULL) != pdPASS) {
        FATAL_ERROR("Display task could not be created!");
    }

    while (1) {
        vTaskDelay(MAIN_TASK_LOOP_DELAY / portTICK_RATE_MS);

        if (wifi_state == WIFI_STA_CONNECTED && !init_done) {

            /* We are connected to WiFi now */
            sntp_start();

            /* Init the MQTT command receiving logic */
            if (cmd_recv_init() != ESP_OK) {
                FATAL_ERROR("CMD not started!");
            }
            
            init_done = true;
        }

        /* Actions done each MAIN_TASK_LOOP_DELAY interval */

        sensors_run();
        
        /* Check if should send sensors info */
        sensors_send_info_time += MAIN_TASK_LOOP_DELAY;
        if (sensors_send_info_time >= SENSORS_READ_TIME) {
            sensors_data_t sensors_data;

            sensors_send_info_time = 0;

            if (sensors_get_data(&sensors_data) != ESP_OK ||
                send_sensors_info(&sensors_data) != ESP_OK) {
                /* sensors info not sent */
                ESP_LOGE(TAG, "Failed to send sensor info!");

                /* Set retry interval to 1/10 of sensor read time. For 10min will retry in 1min */
                sensors_send_info_time = (SENSORS_READ_TIME * 9) / 10;
                send_retry ++;

                /* Reboot if we tried too many times */
                if (send_retry > SENSORS_SEND_RETRY) {
                    FATAL_ERROR("Too many send errors!");
                }
            } else {
                send_retry = 0;
            }
        }
    }
}

static void wifi_init_softap(void)
{
    char* base_mac = nvs_get_base_mac();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t wifi_config = {
        .ap = {
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    tcpip_adapter_init();
    if (esp_event_loop_init(event_handler, NULL) != ESP_OK ||
        esp_wifi_init(&cfg) != ESP_OK) {
        FATAL_ERROR("Could not init WiFi!");
    }

    /* Base mac is both the SSID and password */
    strcpy((char *)wifi_config.ap.ssid, base_mac);
    strcpy((char *)wifi_config.ap.password, base_mac);
    wifi_config.ap.ssid_len = strlen(base_mac);

    if (esp_wifi_set_mode(WIFI_MODE_AP) != ESP_OK                   ||
        esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config) != ESP_OK ||
        esp_wifi_start() != ESP_OK) {
        FATAL_ERROR("Could not start AP mode!");
    }

    ESP_LOGI(TAG, "Init softap with SSID %s pass %s",
             wifi_config.ap.ssid, wifi_config.ap.password);
}

static void wifi_init_sta(char* ssid, char* pass)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t wifi_config;

    tcpip_adapter_init();
    if (esp_event_loop_init(event_handler, NULL) != ESP_OK ||
        esp_wifi_init(&cfg) != ESP_OK                      ||
        esp_wifi_set_storage(WIFI_STORAGE_RAM) != ESP_OK) {
        FATAL_ERROR("Could not init WiFi!");
    }

    memset(&wifi_config, 0, sizeof(wifi_config));
    strcpy((char *)wifi_config.sta.ssid, ssid);
    strcpy((char *)wifi_config.sta.password, pass);
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);

    if (esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK                   ||
        esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) != ESP_OK ||
        esp_wifi_start() != ESP_OK) {
        FATAL_ERROR("Could not stat station mode!");
    }
}

static esp_err_t uart_init(void)
{
    ESP_LOGI(TAG, "UART init");

    /* Configure UART to 115200, 8N1*/   
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    return (uart_param_config(0, &uart_config));
}

static esp_err_t gpio_init(void)
{
    gpio_config_t io_conf;

    /* Config LED GPIO */
    ESP_LOGI(TAG, "GPIO init");

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1 << GPIO_BLUE_LED);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    if (gpio_config(&io_conf) != ESP_OK)
        return ESP_FAIL;
        
    /* Config buttons */
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1 << GPIO_OTA_BUTTON)                |
                           (1 << GPIO_DISPLAY_BRIGHTNESS_BUTTON) |
                           (1 << GPIO_DISPLAY_TEMP_BUTTON)       |
                           (1 << GPIO_DISPLAY_HUMIDITY_BUTTON);
    io_conf.pull_up_en = 1;
    return gpio_config(&io_conf);
}

void app_main()
{
    uint8_t ap_mode = 0;
    uint8_t brightness_level = 0;
    nvs_handle nvs;
    char wifi_ssid[24];
    char wifi_pass[24];

    /* Init drivers */
    if (uart_init() != ESP_OK         ||
        gpio_init() != ESP_OK         ||
        nvs_init() != ESP_OK          ||
        i2c_init(I2C_SENSORS_BUS,
                 GPIO_I2C_MASTER_SCL,
                 GPIO_I2C_MASTER_SDA,
                 I2C_CLOCK_FREQ) != 0 ||
        display_init(GPIO_DISPLAY_CLK,
                     GPIO_DISPLAY_DATA,
                     DISPLAY_CLOCK_FREQ) != ESP_OK) {
        FATAL_ERROR("Could not init drivers!");
    }

    ESP_LOGI(TAG, "FW VERSION: %s", FW_VERSION);
    ESP_LOGI(TAG, "BASE MAC  : %s", nvs_get_base_mac());

    /* Turn off blue LED */
    gpio_set_level(GPIO_BLUE_LED, 1);

    nvs = nvs_get_handle();

    /* Read display brightness_level from flash */
    nvs_get_u8(nvs, NVS_DISPLAY_BRIGHTNESS, &brightness_level);

    /* Blank the display and set brightness */
    display_write("    ");
    display_set_brightness(brightness_level);

    sensors_init(I2C_SENSORS_BUS, GPIO_SENSORS_WAKE_GPIO);

    /* Read WiFi mode from flash*/
    nvs_get_u8(nvs, NVS_WIFI_AP_MODE, &ap_mode);
    if (!ap_mode) {
        size_t ssid_len, pass_len;

        ssid_len = sizeof(wifi_ssid);
        pass_len = sizeof(wifi_pass);
        if (nvs_get_str(nvs, NVS_WIFI_SSID, wifi_ssid, &ssid_len) != ESP_OK ||
            nvs_get_str(nvs, NVS_WIFI_PASS, wifi_pass, &pass_len) != ESP_OK) {
            /* User or password not found in flash, set AP mode */
            ap_mode = 1;
        
            ESP_LOGE(TAG, "---------------------------------------------------------------------------");
            ESP_LOGE(TAG, "Station mode set but no SSID/password found in flash. Switching to AP mode!");
            ESP_LOGE(TAG, "---------------------------------------------------------------------------");
        }
    }

    if (ap_mode) {
        wifi_state = WIFI_AP_MODE;
        wifi_init_softap();
        http_server_init();
    } else {
        wifi_state = WIFI_STA_DISCONNECTED;
        wifi_init_sta(wifi_ssid, wifi_pass);
    }

    /* Set timezone */
    setenv("TZ", TIMEZONE, 1);
    tzset();

    if (xTaskCreate(main_task, "main_task", MAIN_TASK_STACK_SIZE, NULL,
        MAIN_TASK_PRIORITY, NULL) != pdPASS) {
        FATAL_ERROR("Main task could not be created!");
    }
}
