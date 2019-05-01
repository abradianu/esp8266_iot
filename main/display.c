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

#include "esp_log.h"
#include "tm1637.h"

#include "display.h"

static const char *TAG = "DISP";

/* 4-Digits 7 segments display. Only TM1637 supported for now */

static tm1637_display_t *tm1637_display;

esp_err_t display_init(gpio_num_t clk_pin, gpio_num_t data_pin, uint32_t clk_freq)
{
    ESP_LOGI(TAG, "TM1637 display init");

    /* Init TM1637, 4 digit 7 Segments display */
    tm1637_display = tm1637_init(clk_pin, data_pin, clk_freq);
    if (!tm1637_display) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

void display_write(char *str)
{
    tm1637_write(tm1637_display, str);
}

esp_err_t display_set_brightness(uint8_t brightness_level)
{
    if (brightness_level > TM1637_MAX_BRIGHTNESS)
        return ESP_FAIL;

    return tm1637_set_brightness(tm1637_display, brightness_level);
}
