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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c.h"
#include "hdc1080.h"
#include "ccs811.h"
#include "esp_log.h"
#include "nvs_utils.h"

#include "sensors.h"

#define SENSORS_CCS811
#define SENSORS_HDC1080

/* Sensors read intervals in seconds */
#define SENSORS_CO2_READ_INTERVAL        (2 * 60 * 1000)
#define SENSORS_TEMP_READ_INTERVAL       (10 * 1000)

/* CCS811 baseline saving interval 24h */
#define SENSORS_CCS811_BASELINE_INTERVAL (24 * 60 * 60 * 1000)

#ifdef SENSORS_CCS811
static ccs811_sensor_t* ccs811_sensor;
#endif
#ifdef SENSORS_HDC1080
static hdc1080_sensor_t* hdc1080_sensor;
#endif
static bool sensors_initialized;
static sensors_data_t *sensors_data;

static const char *TAG = "SENSORS";

esp_err_t sensors_get_data(sensors_data_t *data)
{
    if (!sensors_initialized)
        return ESP_FAIL;

    /* Return last sensors data */
    memcpy(data, sensors_data, sizeof(sensors_data_t));
    
    return ESP_OK;
}

void sensors_run()
{
    uint32_t now;
    static uint32_t last_temp_read = 0;
    static uint32_t last_co2_read = 0;
#ifdef SENSORS_CCS811
    static uint32_t last_baseline_read = 0;
#endif

    if (!sensors_initialized)
        return;

    now = xTaskGetTickCount() * portTICK_RATE_MS;

    if (now - last_temp_read > SENSORS_TEMP_READ_INTERVAL) {
#ifdef SENSORS_HDC1080
        /* Read temperature and humidity first */
        if (hdc1080_read(hdc1080_sensor, &sensors_data->temp, &sensors_data->humidity) != ESP_OK) {
            /* Set retry time */
            last_temp_read += SENSORS_TEMP_READ_INTERVAL / 10;
        } else {
            ESP_LOGI(TAG, "Temp %d.%d, Humidity %d",
                (int)(sensors_data->temp), (int)(sensors_data->temp * 10) % 10,
                (int)(sensors_data->humidity));

            last_temp_read = now;
        }
#endif
    }

    if (now - last_co2_read > SENSORS_CO2_READ_INTERVAL) {
#ifdef SENSORS_CCS811
        if (ccs811_get_results (ccs811_sensor, &sensors_data->tvoc, &sensors_data->eco2, NULL, NULL)) {
            ESP_LOGI(TAG, "TVOC %d ppb, eCO2 %d ppm, err 0x%x",
                    sensors_data->tvoc, sensors_data->eco2, ccs811_sensor->error_code);
            last_co2_read = now;
        } else {
            /* Set retry time */
            last_co2_read += SENSORS_CO2_READ_INTERVAL / 10;
        }

        /* CCS811 Temperature and Humidity Compensation */
        ccs811_set_environmental_data(ccs811_sensor, sensors_data->temp, sensors_data->humidity);

        /* Save the baseline each 24h */
        if (now - last_baseline_read > SENSORS_CCS811_BASELINE_INTERVAL) {
            uint16_t baseline;

            last_baseline_read = now;

            baseline = ccs811_get_baseline(ccs811_sensor);
            if (baseline) {
                /* Save the baseline in flash */
                nvs_set_u16(nvs_get_handle(), NVS_CCS811_BASELINE, baseline);
            }
        }
#endif
    }
}

esp_err_t sensors_init(uint8_t i2c_bus, uint8_t wake_gpio)
{
#ifdef SENSORS_CCS811
    /* longer clock stretching is required for CCS811 */
    i2c_set_clock_stretch (i2c_bus, CCS811_I2C_CLOCK_STRETCH);

    /* Init CCS811, for monitoring Indoor Air Quality */
    ccs811_sensor = ccs811_init_sensor(i2c_bus,
                                       CCS811_I2C_ADDRESS_1,
                                       wake_gpio);
    if (ccs811_sensor) {
        uint16_t baseline;

        /* Set the baseline saved in flash if any */
        if (nvs_get_u16(nvs_get_handle(), NVS_CCS811_BASELINE, &baseline) == ESP_OK &&
            ccs811_set_baseline(ccs811_sensor, baseline)) {
            ESP_LOGI(TAG, "CCS811 baseline set");
        }

        ccs811_set_mode(ccs811_sensor, ccs811_mode_60s);
    } else {
        ESP_LOGE(TAG, "Failed to init CCS811 sensor!");
        return ESP_FAIL;
    }
#endif

#ifdef SENSORS_HDC1080
    /* Init temperature and humidity sensor */
    hdc1080_sensor = hdc1080_init(i2c_bus);
    if (!hdc1080_sensor) {
        ESP_LOGE(TAG, "Failed to init HDC1080 sensor!");
        return ESP_FAIL;
    }
#endif

    sensors_data = calloc(1, sizeof(sensors_data_t));
    if (!sensors_data)
        return ESP_FAIL;

    ESP_LOGI(TAG, "Sensors init done");
    sensors_initialized = true;
    
    return ESP_OK;
}
