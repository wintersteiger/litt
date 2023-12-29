cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(littP VERSION 0.1 LANGUAGES C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico_w)
pico_sdk_init()

set(WIFI_SSID "my-wifi" CACHE STRING "Wifi network SSID")
set(WIFI_PASSWORD "my-wifi-password" CACHE STRING "Wifi network password")
set(WIFI_AUTH "CYW43_AUTH_WPA2_AES_PSK" CACHE STRING "Wifi authentication method")

set(MQTT_BROKER_HOST "my-broker-host" CACHE STRING "MQTT broker host")
set(MQTT_CLIENT_ID "my-client-id" CACHE STRING "MQTT client ID")
set(MQTT_USER "my-mqtt-user" CACHE STRING "MQTT user ID")
set(MQTT_PASS "my-mqtt-password" CACHE STRING "MQTT password")

set(TEMPERATURE_TOPICS "Temperature" CACHE STRING "MQTT topic(s) for temperature measurements")
set(TEMPERATURE_TOPICS "Demands" CACHE STRING "MQTT topic(s) for temperature demand measurements")

add_executable(littP)

pico_add_extra_outputs(littP)
pico_enable_stdio_usb(littP 1)
pico_enable_stdio_uart(littP 0)
pico_generate_pio_header(littP ${CMAKE_CURRENT_LIST_DIR}/opentherm.pio)

target_sources(littP PUBLIC main.cpp ${PICO_SDK_PATH}/lib/lwip/src/apps/mqtt/mqtt.c)

target_include_directories(littP PRIVATE ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/../../include)

target_link_libraries(littP PRIVATE
  pico_stdlib
  pico_multicore
  pico_platform
  hardware_adc
  hardware_pio
  hardware_flash
  hardware_exception
  hardware_rtc
  pico_cyw43_arch_lwip_threadsafe_background
  pico_lwip_http
  pico_lwip_sntp)

target_compile_definitions(littP PRIVATE
  LITT_HAVE_STL
  WIFI_SSID=\"${WIFI_SSID}\"
  WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
  WIFI_AUTH=${WIFI_AUTH}
  MQTT_BROKER_HOST=\"${MQTT_BROKER_HOST}\"
  MQTT_CLIENT_ID=\"${MQTT_CLIENT_ID}\"
  MQTT_USER=\"${MQTT_USER}\"
  MQTT_PASS=\"${MQTT_PASS}\"
  TEMPERATURE_TOPICS=\"${TEMPERATURE_TOPICS}\"
  DEMAND_TOPICS=\"${DEMAND_TOPICS}\"
)

target_compile_options(littP PRIVATE -Wall)