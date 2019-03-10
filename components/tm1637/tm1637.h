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

#ifndef __TM1637_H__
#define __TM1637_H__

#include "driver/gpio.h"

#ifdef	__cplusplus
extern "C" {
#endif

#define TM1637_MIN_BRIGHTNESS     0
#define TM1637_MAX_BRIGHTNESS     3 /* 7 in fact but only 4 used */

typedef struct {
    gpio_num_t data_pin;
    gpio_num_t clk_pin;
    uint32_t   clk_delay;
} tm1637_display_t;
    
tm1637_display_t * tm1637_init(gpio_num_t clk_pin, gpio_num_t data_pin, uint32_t clk_freq);
esp_err_t tm1637_write(tm1637_display_t *dev, char *str);
esp_err_t tm1637_set_brightness(tm1637_display_t *dev, uint8_t val);

#ifdef	__cplusplus
}
#endif

#endif /* __TM1637_H__ */
