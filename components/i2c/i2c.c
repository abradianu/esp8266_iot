/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Johan Kanflo (github.com/kanflo)
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

/*
 * Adapted for Espressif SDK
 */

#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "i2c.h"

static const char *TAG = "I2C";

// Bus settings
typedef struct i2c_bus_description
{
  uint8_t g_scl_pin;     // SCL pin
  uint8_t g_sda_pin;     // SDA pin
  uint16_t delay;
  bool started;
  bool flag;
  bool force;
  TickType_t clk_stretch;
} i2c_bus_description_t;

static i2c_bus_description_t i2c_bus[I2C_MAX_BUS];

inline bool i2c_status(uint8_t bus)
{
    return i2c_bus[bus].started;
}

int i2c_init(uint8_t bus, uint8_t scl_pin, uint8_t sda_pin, uint32_t freq)
{
    ESP_LOGI(TAG, "Init I2C bus %d, scl pin %d, sda pin %d, freq %d",
             bus, scl_pin, sda_pin, freq);

    if (bus >= I2C_MAX_BUS) {
        ESP_LOGE(TAG, "Invalid bus %d", bus);
        return -EINVAL;
    }

    if (!GPIO_IS_VALID_GPIO(scl_pin) || 
        !GPIO_IS_VALID_GPIO(sda_pin))
    {
        ESP_LOGE(TAG, "Invalid GPIOs");
        return -EINVAL;
    }

    i2c_bus[bus].started = false;
    i2c_bus[bus].flag = false;
    i2c_bus[bus].g_scl_pin = scl_pin;
    i2c_bus[bus].g_sda_pin = sda_pin;
    i2c_bus[bus].clk_stretch = I2C_DEFAULT_CLK_STRETCH;

    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT_OD;
    io_conf.pin_bit_mask = (1 << sda_pin) | (1 << scl_pin);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    
    // I2C bus idle state.
    gpio_set_level(scl_pin, 1);
    gpio_set_level(sda_pin, 1);

    // Inform user if the desired frequency is not supported.
    if (i2c_set_frequency(bus, freq) != 0) {
         ESP_LOGE(TAG, "Frequency not supported");
        return -ENOTSUP;
    }

    return 0;
}

int i2c_set_frequency(uint8_t bus, uint32_t freq)
{
    if (freq == 0) return -EINVAL;

    i2c_bus[bus].delay = 1000000 / (freq * 2);
    if (!i2c_bus[bus].delay)
        i2c_bus[bus].delay = 1;

    ESP_LOGI(TAG, "freq %d, delay set to %d", freq, i2c_bus[bus].delay);
    
    return 0;
}

void i2c_set_clock_stretch(uint8_t bus, TickType_t clk_stretch)
{
    i2c_bus[bus].clk_stretch = clk_stretch;
}

static inline void i2c_delay(uint8_t bus)
{
    os_delay_us(i2c_bus[bus].delay);
}

static inline bool read_scl(uint8_t bus)
{
    return gpio_get_level(i2c_bus[bus].g_scl_pin);
}

static inline bool read_sda(uint8_t bus)
{
    return gpio_get_level(i2c_bus[bus].g_sda_pin);
}

// Actively drive SCL signal low
static inline void clear_scl(uint8_t bus)
{
    gpio_set_level(i2c_bus[bus].g_scl_pin, 0);
}

// Actively drive SDA signal low
static inline void clear_sda(uint8_t bus)
{
    gpio_set_level(i2c_bus[bus].g_sda_pin, 0);
}

#define I2C_CLK_STRETCH_SPIN 1024

static void set_scl(uint8_t bus)
{
    gpio_set_level(i2c_bus[bus].g_scl_pin, 1);

    // Clock stretching.

    // Spin sampling frequently.
    uint32_t clk_stretch_spin = I2C_CLK_STRETCH_SPIN;
    do {
        if (read_scl(bus)) {
            return;
        }

        clk_stretch_spin--;
    } while (clk_stretch_spin);

    // Fall back to a longer wait, sampling less frequently.
    TickType_t clk_stretch = i2c_bus[bus].clk_stretch;
    TickType_t start = xTaskGetTickCount();

    do {
        vTaskDelay(20 / portTICK_PERIOD_MS);

        if (read_scl(bus)) {
            return;
        }

        TickType_t elapsed = xTaskGetTickCount() - start;
        if (elapsed > clk_stretch) {
            break;
        }
    } while (1);

    ESP_LOGE(TAG, "bus %u clock stretch timeout", bus);
}

static inline void set_sda(uint8_t bus)
{
    gpio_set_level(i2c_bus[bus].g_sda_pin, 1);
}

// Output start condition
void i2c_start(uint8_t bus)
{
    if (i2c_bus[bus].started) { // if started, do a restart cond
        // Set SDA to 1
        set_sda(bus);
        i2c_delay(bus);
        set_scl(bus);
        // Repeated start setup time, minimum 4.7us
        i2c_delay(bus);
    }
    i2c_bus[bus].started = true;
    set_sda(bus);
    if (read_sda(bus) == 0) {
        ESP_LOGE(TAG, "arbitration lost in i2c_start from bus %u", bus);
    }
    // SCL is high, set SDA from 1 to 0.
    clear_sda(bus);
    i2c_delay(bus);
    clear_scl(bus);
}

// Output stop condition
bool i2c_stop(uint8_t bus)
{
    // Set SDA to 0
    clear_sda(bus);
    i2c_delay(bus);
    set_scl(bus);
    // Stop bit setup time, minimum 4us
    i2c_delay(bus);
    // SCL is high, set SDA from 0 to 1
    set_sda(bus);
    // additional delay before testing SDA value to avoid wrong state
    i2c_delay(bus); 
    if (read_sda(bus) == 0) {
        ESP_LOGE(TAG, "arbitration lost in i2c_stop from bus %u", bus);
    }
    i2c_delay(bus);
    if (!i2c_bus[bus].started) {
        ESP_LOGE(TAG, "bus %u link was break!", bus);
        return false; // If bus was stop in other way, the current transmission Failed
    }
    i2c_bus[bus].started = false;
    return true;
}

// Write a bit to I2C bus
static void i2c_write_bit(uint8_t bus, bool bit)
{
    if (bit) {
        set_sda(bus);
    } else {
        clear_sda(bus);
    }
    i2c_delay(bus);
    set_scl(bus);
    // SCL is high, now data is valid
    // If SDA is high, check that nobody else is driving SDA
    if (bit && read_sda(bus) == 0) {
        ESP_LOGE(TAG, "arbitration lost in i2c_write_bit from bus %u", bus);
    }
    i2c_delay(bus);
    clear_scl(bus);
}

// Read a bit from I2C bus
static bool i2c_read_bit(uint8_t bus)
{
    bool bit;
    // Let the slave drive data
    set_sda(bus);
    i2c_delay(bus);
    set_scl(bus);
    // SCL is high, now data is valid
    bit = read_sda(bus);
    i2c_delay(bus);
    clear_scl(bus);
    return bit;
}

bool i2c_write(uint8_t bus, uint8_t byte)
{
    bool nack;
    uint8_t bit;
    for (bit = 0; bit < 8; bit++) {
        i2c_write_bit(bus, (byte & 0x80) != 0);
        byte <<= 1;
    }
    nack = i2c_read_bit(bus);
    return !nack;
}

uint8_t i2c_read(uint8_t bus, bool ack)
{
    uint8_t byte = 0;
    uint8_t bit;
    for (bit = 0; bit < 8; bit++) {
        byte = ((byte << 1)) | (i2c_read_bit(bus));
    }
    i2c_write_bit(bus, ack);
    return byte;
}

void i2c_force_bus(uint8_t bus, bool state)
{
    i2c_bus[bus].force = state;
}

static int i2c_bus_test(uint8_t bus)
{
    taskENTER_CRITICAL(); // To prevent task swaping after checking flag and before set it!
    bool status = i2c_bus[bus].flag; // get current status
    if (i2c_bus[bus].force) {
        i2c_bus[bus].flag = true; // force bus on
        taskEXIT_CRITICAL();
        if (status)
           i2c_stop(bus); //Bus was busy, stop it.
    }
    else {
        if (status) {
            taskEXIT_CRITICAL();
            ESP_LOGE(TAG, "busy");
            taskYIELD(); // If bus busy, change task to try finish last com.
            return -EBUSY;  // If bus busy, inform user
        }
        else {
            i2c_bus[bus].flag = true; // Set Bus busy
            taskEXIT_CRITICAL();
        }
    }
    return 0;
}

int i2c_slave_write(uint8_t bus, uint8_t slave_addr, const uint8_t *reg, const uint8_t *buf, uint32_t len)
{
    if (i2c_bus_test(bus))
        return -EBUSY;

    i2c_start(bus);
    if (!i2c_write(bus, slave_addr << 1))
        goto error;
    if (reg != NULL)
        if (!i2c_write(bus, *reg))
            goto error;
    while (len--) {
        if (!i2c_write(bus, *buf++))
            goto error;
    }
    if (!i2c_stop(bus))
        goto error;
    i2c_bus[bus].flag = false; // Bus free
    return 0;

error:
    ESP_LOGE(TAG, "Bus %u Write Error", bus);
    i2c_stop(bus);
    i2c_bus[bus].flag = false; // Bus free
    return -EIO;
}

int i2c_slave_read(uint8_t bus, uint8_t slave_addr, const uint8_t *reg, uint8_t *buf, uint32_t len)
{
    if (i2c_bus_test(bus))
        return -EBUSY;

    if (reg != NULL) {
        i2c_start(bus);
        if (!i2c_write(bus, slave_addr << 1))
            goto error;
        if (!i2c_write(bus, *reg))
            goto error;
    }
    i2c_start(bus);
    if (!i2c_write(bus, (slave_addr << 1) | 1)) // Slave address + read
        goto error;
    while(len) {
        *buf = i2c_read(bus, len == 1);
        buf++;
        len--;
    }
    if (!i2c_stop(bus))
        goto error;

    i2c_bus[bus].flag = false; // Bus free
    return 0;

error:
    ESP_LOGE(TAG, "Read Error");
    i2c_stop(bus);
    i2c_bus[bus].flag = false; // Bus free
    return -EIO;
}
