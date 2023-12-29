# littP

An example thermostat based on the [Raspberry Pi Pico W](https://www.raspberrypi.com/products/raspberry-pi-pico/) and OpenTherm boilers. It receives updates from temperature sensors via [MQTT](https://mqtt.org/) over Wi-Fi and adjusts the flow temperature of the boiler via [PID controllers](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller). It provides a simple webpage for status information.

## Prerequisites

You need the [Pico SDK](https://www.raspberrypi.com/documentation/pico-sdk/).

## Build

The Wi-Fi authentication data and the sensors' MQTT topics that littP subscribes to are currently fixed at compile time, so the configuration command is rather lengthy.

```
mkdir build
cd build
cmake -GNinja -DCMAKE_BUILD_TYPE=Debug -DWIFI_SSID=<...> -DWIFI_PASSWORD=<...> -DMQTT_BROKER_HOST=<IP> -DMQTT_CLIENT_ID=littP -DMQTT_USER=<...> -DMQTT_PASS=<...> -DTEMPERATURE_TOPICS="SensorTopic1|LocationName1,SensorTopic2|LocationName2,..." ..
ninja
```