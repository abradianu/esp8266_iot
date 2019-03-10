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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_log.h"
#include "i2c.h"

#include "hdc1080.h"

#define HDC1080_I2C_ADDRESS           0x40

/* HDC1080 register addresses */
#define HDC1080_REG_TEMPERATURE       0x00
#define HDC1080_REG_HUMIDITY          0x01
#define HDC1080_REG_CONFIG            0x02
#define HDC1080_REG_MANUF_ID          0xFE
#define HDC1080_REG_DEVICE_ID         0xFF

/* Temperatureand Humidityare acquiredin sequence,Temperature first. */
#define HDC1080_REG_CONFIG_MODE_BOTH  (1 << 4)
#define HDC1080_REG_CONFIG_TRES_11BIT (1 << 2)
#define HDC1080_REG_CONFIG_TRES_14BIT (0 << 2)
#define HDC1080_REG_CONFIG_HRES_8BIT  (2 << 0)
#define HDC1080_REG_CONFIG_HRES_11BIT (1 << 0)
#define HDC1080_REG_CONFIG_HRES_14BIT (0)

#define HDC1080_DEVICE_ID             0x1050
#define HDC1080_MANUF_ID              0x5449

static const char *TAG = "HDC1080";

hdc1080_sensor_t* hdc1080_init(uint8_t bus)
{
    uint8_t reg;
    uint8_t data[2];
    hdc1080_sensor_t* dev;
    
    if ((dev = malloc (sizeof(hdc1080_sensor_t))) == NULL)
        return NULL;

    // init sensor data structure
    dev->bus  = bus;

    reg = HDC1080_REG_MANUF_ID;
    if (i2c_slave_read(bus, HDC1080_I2C_ADDRESS, &reg, data, 2) != 0 ||
        ((data[0] << 8) | data[1]) != HDC1080_MANUF_ID) {
        return NULL;
    }

    reg = HDC1080_REG_DEVICE_ID;
    if (i2c_slave_read(bus, HDC1080_I2C_ADDRESS, &reg, data, 2) != 0 ||
        ((data[0] << 8) | data[1]) != HDC1080_DEVICE_ID) {
        return NULL;
    }

    reg = HDC1080_REG_CONFIG;
    data[0] =   HDC1080_REG_CONFIG_TRES_14BIT |
                HDC1080_REG_CONFIG_HRES_11BIT |
                HDC1080_REG_CONFIG_MODE_BOTH;
    data[1] = 0;
    if (i2c_slave_write(bus, HDC1080_I2C_ADDRESS, &reg, data, 2) != 0) {
        return NULL;
    }

    ESP_LOGI(TAG, "HDC1080 init done, I2C bus %d, addr 0x%x", bus, HDC1080_I2C_ADDRESS);
    
    return dev;
}

/* Read temperature and humidity */
esp_err_t hdc1080_read(hdc1080_sensor_t* sensor, float * temp, float * humidity)
{
    uint8_t reg;
    uint8_t data[4];

    if (!temp || !humidity)
        return ESP_FAIL;

    /* 
     * Trigger the measurements by executing a pointer write transaction with
     * the address pointer set to 0x00
     */
    reg = HDC1080_REG_TEMPERATURE;
    if (i2c_slave_write(sensor->bus, HDC1080_I2C_ADDRESS, &reg, NULL, 0) != 0) {
        return ESP_FAIL;
    }

    /* Wait for the measurements to complete */
    vTaskDelay(20 / portTICK_PERIOD_MS);

    /* Read both temperature and humidity in a single read transaction */
    if (i2c_slave_read(sensor->bus, HDC1080_I2C_ADDRESS, NULL, data, 4) != 0) {
        return ESP_FAIL;
    }

    /* Temp with one decimal */
    *temp = ((float)(165 * ((data[0] << 8) | data[1]))) / (1 << 16) - 40;
    *humidity = (100 * ((data[2] << 8) | data[3])) / (1 << 16);

    return ESP_OK;
}