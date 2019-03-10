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
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "esp_system.h"
#include "esp_log.h"

#include "tm1637.h"

/* Data command: Normal mode, automatic address adding, write data */
#define TM1637_DATA_CMD_WRITE    0x40

/* Display register address 0 */
#define TM1637_ADDRESS_CMD       0xC0

#define TM1637_DISPLAY_CTRL_ON   0x88

/* Segment 8 is display point */
#define TM1637_DP_SEGMENT        0x80

static const char *TAG = "TM1637";

static const unsigned char bcd_2_segments[] = {0x3f, 0x6, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f};

static void tm1637_send_start(tm1637_display_t *dev)
{
    /*
     * When CLK is high level and DIO changes from high to low level,
     * data input start.
     */

    gpio_set_level(dev->data_pin, 1);
    gpio_set_level(dev->clk_pin, 1);
    usleep(dev->clk_delay);
    gpio_set_level(dev->data_pin, 0);
    usleep(dev->clk_delay);
}

static void tm1637_send_stop(tm1637_display_t *dev)
{
    /*
     * When CLK is high level and DIO changes from low level to high level,
     * data input ends.
     */

    gpio_set_level(dev->clk_pin, 0);
    gpio_set_level(dev->data_pin, 0);
    usleep(dev->clk_delay);
    gpio_set_level(dev->clk_pin, 1);
    usleep(dev->clk_delay);
    gpio_set_level(dev->data_pin, 1);
}

static esp_err_t tm1637_read_ack(tm1637_display_t *dev)
{
    int ack;

    /* 
     * For  a  right  data transfer, an answering  signal  ACK  is generated inside
     * the chip to lower the DIO pin at the failing edge of the 8th clock. 
     * DIO interface wire is released at the end of the 9th clock.
     */

    gpio_set_direction(dev->data_pin, GPIO_MODE_DEF_INPUT);
    gpio_set_level(dev->clk_pin, 0);
    usleep(dev->clk_delay);
   
    ack = gpio_get_level(dev->data_pin);
   
    gpio_set_level(dev->clk_pin, 1);
    usleep(dev->clk_delay);

    gpio_set_level(dev->clk_pin, 0);
    gpio_set_direction(dev->data_pin, GPIO_MODE_DEF_OUTPUT);
   
    return (ack == 0 ? ESP_OK : ESP_FAIL);
}

static void tm1637_send_byte(tm1637_display_t *dev, uint8_t b)
{
    uint8_t i;

    for (i = 0; i < 8; i++) {
        gpio_set_level(dev->clk_pin, 0);
        gpio_set_level(dev->data_pin, b & (1 << i));
        usleep(dev->clk_delay);
        gpio_set_level(dev->clk_pin, 1);
        usleep(dev->clk_delay);
    }
}

static esp_err_t tm1637_write_data(tm1637_display_t *dev, uint8_t *data, uint8_t len)
{
    esp_err_t rc = ESP_OK;
    uint8_t i;

    tm1637_send_start(dev);
    for (i = 0; i < len; i++) {
        tm1637_send_byte(dev, data[i]);
        if (tm1637_read_ack(dev) != ESP_OK) {
            ESP_LOGI(TAG, "Error reading ACK");
            rc = ESP_FAIL;
        }
    }
    tm1637_send_stop(dev);
    
    return rc;
}

/*
 * Display a string of 4 digits. Acceptable string characters are numbers from
 * 0 - 9 and ":" only as the third character. Ex.: "01:23" or "0123". To shut
 * down a specific digit, use any other character on that position.
 */
esp_err_t tm1637_write(tm1637_display_t *dev, char *str)
{
    uint8_t cmd_len = 0, cmd[6];
    uint8_t i, str_len;
    
    str_len = strlen (str);
    if (str_len > 5)
        return ESP_FAIL;
       
    /* Data command: Normal mode, automatic address adding, write data */
    cmd[0] = TM1637_DATA_CMD_WRITE;
    if (tm1637_write_data(dev, cmd, 1) != ESP_OK)
        return ESP_FAIL;

    /* Address command setting: Display address 0 */
    cmd[cmd_len++] = TM1637_ADDRESS_CMD;
    
    for (i = 0; i < str_len; i++) {
        if (str[i] >= '0' && str[i] <= '9') {
            cmd[cmd_len++] = bcd_2_segments[str[i] - '0'];
        } else if (i == 2 && str[i] == ':') {
            cmd[cmd_len - 1] |= TM1637_DP_SEGMENT;
        } else {
            cmd[cmd_len++] = 0;
        }
    }

    return tm1637_write_data(dev, cmd, cmd_len);
}

esp_err_t tm1637_set_brightness(tm1637_display_t *dev, uint8_t val)
{
    uint8_t cmd;

    if (val > TM1637_MAX_BRIGHTNESS)
        val = TM1637_MAX_BRIGHTNESS;

    cmd = TM1637_DISPLAY_CTRL_ON | val;
    return tm1637_write_data(dev, &cmd, 1);
}

tm1637_display_t * tm1637_init(gpio_num_t clk_pin, gpio_num_t data_pin, uint32_t clk_freq)
{
    gpio_config_t io_conf;
    tm1637_display_t *dev;

    if ((dev = malloc (sizeof(tm1637_display_t))) == NULL)
        return NULL;
    
    dev->clk_pin = clk_pin;
    dev->data_pin = data_pin;
    
    /* Compute clock period/2 in us */
    dev->clk_delay = (1000000 / clk_freq) / 2;
    if (!dev->clk_delay) {
        free (dev);
        return NULL;
    }
    
    ESP_LOGI(TAG, "Init: clk_pin %d, data_pin %d, clk_delay %d",
             dev->clk_pin, dev->data_pin, dev->clk_delay);

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1 << dev->clk_pin) | (1 << dev->data_pin);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    if (gpio_config(&io_conf) != ESP_OK) {
        free (dev);
        return NULL;
    }
   
    gpio_set_level(dev->clk_pin, 1);
    gpio_set_level(dev->data_pin, 1);
    
    return dev;
}
