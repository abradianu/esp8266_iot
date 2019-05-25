# Digital Clock + temperature, humidity and CO2 monitor.

## Components:
* CPU board: ESP8266 NodeMCU
* Digital clock display: TM1637 4-digits display
* Temperature & Humidity sensor: HDC1080
* CO2 sensor: CCS811

![alt text](https://github.com/abradianu/esp8266_iot/blob/master/docs/Schematic_esp8266_iot.png)

## Toolchain
Download xtensa linux64 toolchain from
https://dl.espressif.com/dl/xtensa-lx106-elf-linux64-1.22.0-92-g8facf4c-5.2.0.tar.gz
Extract it to ~/work/esp8266/xtensa-lx106-elf

## Build
Clone the ESP8266_RTOS_SDK sdk and esp8266_iot projects:
```
cd ~/work/esp8266/
git clone https://github.com/abradianu/ESP8266_RTOS_SDK.git
git clone https://github.com/abradianu/esp8266_iot.git
```

Setup path to xtensa toolchain and Espressif IDF:
```
export PATH=$PATH:~/work/esp8266/xtensa-lx106-elf/bin
export IDF_PATH=~/work/esp8266/ESP8266_RTOS_SDK
```

Build the project:
```
cd ~/work/esp8266/esp8266_iot
make
```

## MQTT commands
#### Set MQTT broker ip (default 192.168.1.135):
```
mosquitto_pub -h 192.168.1.135 -t "sensors/cmd/840d8e8ff6da" -m "{\"cmd\": 5,\"ip\": \"192.168.1.112\"}"
```
#### Set MQTT client name:
```
mosquitto_pub -h 192.168.1.135 -t "sensors/cmd/840d8e8ff6da" -m "{\"cmd\": 4,\"name\": \"room1\"}"
```
#### Get device info:
```
mosquitto_pub -h 192.168.1.135 -t "sensors/cmd/room1" -m "{\"cmd\": 2}"
```
#### OTA:

Start a HTTP server:
```
cd ~/work/esp8266/esp8266_iot/build
python -m SimpleHTTPServer 8070
```

Send the MQTT OTA upgrade command:
```
mosquitto_pub -h 192.168.1.135 -t "sensors/cmd/room1" -m "{\"cmd\": 1,\"server\": \"192.168.1.140\",\"port\": 8070,\"file\": \"esp8266_iot.bin\"}"
```
