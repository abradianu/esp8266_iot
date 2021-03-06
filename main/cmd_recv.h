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

#ifndef __CMD_PARSE_H__
#define __CMD_PARSE_H__

#ifdef __cplusplus
extern "C"
{
#endif

#define FW_VERSION                "0.2.1"

typedef enum {
    CMD_DO_REBOOT,
    /*
     * Command JSON format:
     * {
     *        "cmd":  0,
     *        "ap"    0
     * }
     * 
     * Action: Set AP mode and reboot device
     */

   CMD_DO_OTA,
    /*
     * Command JSON format:
     * {
     *        "cmd":  1,
     *        "server": "192.168.1.140",
     *        "port":   8070,
     *        "file":  "DigitalClock"
     * }
     * 
     * Action: perform OTA and reboot
     */

    CMD_GET_SYS_INFO,
    /*
     * Command JSON format: "{"cmd": 2}"
     * 
     * Action: publish stats info, JSON format:
     * {
     *        "cmd":  2,
     *        "id":   "84f3eb23bcd5",
     *        "mac":  "010203040506"
     *        "baseline":0xf949
     *        "time": 1549735713,
     *        "fw_v": "0.0.1"
     *        "heap": 60100,
     *        "up":   120
     * }
     */

    CMD_GET_SENSORS_INFO,
    /*
     * Command JSON format: "{"cmd": 3}"
     * 
     * Action: publish sensors info, JSON format:
     * {
     *        "cmd":  3,
     *        "id":   "84f3eb23bcd5",
     *        "time": 1549735713,
     *        "temperature":  2345,
     *        "humidity":     5123,
     *        "co2":  123
     * }
     */

   CMD_SET_MQTT_CLIENT_NAME,
    /*
     * Command JSON format:
     * {
     *        "cmd":  4,
     *        "name"  "dormitor"
     * }
     * 
     * Action: Save the new MQTT client name in the flash memory and reboot.
     */

   CMD_SET_MQTT_SERVER_IP,
    /*
     * Command JSON format:
     * {
     *        "cmd":  5,
     *        "ip"   "192.168.1.135"
     * }
     *
     * Action: Save the new MQTT broker in the flash memory and reboot.
     */

   CMD_SET_DISPLAY_BRIGHTNESS,
    /*
     * Command JSON format:
     * {
     *        "cmd":  6,
     *        "b"     1
     * }
     *
     * Action: Set and save the new brightness level.
     */

   CMD_SET_CCS811_BASELINE,
    /*
     * Command JSON format:
     * {
     *        "cmd":  7,
     *        "baseline"     1234
     * }
     *
     * Action: Set and save the new CCS811 baseline.
     */
} cmd_number_t;

esp_err_t send_sensors_info();
esp_err_t send_sys_info();
esp_err_t cmd_recv_init();

#ifdef __cplusplus
}
#endif

#endif /* __CMD_PARSE_H__ */
