# littZ

littZ is a thermostat based on [Nordic nRF52](https://infocenter.nordicsemi.com/index.jsp?topic=%2Fstruct_nrf52%2Fstruct%2Fnrf52.html) series SoCs that feature zigbee connectivity. It collects heating demands from a number of zigbee thermostats (e.g. smart thermostatic radiator valves) in a building and mixes and translates them into a flow setpoint for an OpenTherm boiler.

It is actively tested on a [Seeed Xiao BLE](https://www.seeedstudio.com/Seeed-XIAO-BLE-nRF52840-p-5201.html) featuring an nRF52840 and a [BBC micro:bit v2](https://microbit.org/) featuring an nRF52833, with an [OpenTherm adapter](https://ihormelnyk.com/opentherm_adapter), connected to an [Ideal Vogue](https://idealheating.com/products/boiler-range/vogue) boiler.

## How it works

Smart TRVs on your radiators publish their heating demand on a zigbee network (in percent, ZCL cluster 0x0201, attribute 0x0008). littZ receives them via end-device binds, to ensure delivery even without a coordinator/gateway. Those are then mixed into a combined demand that is translated into a flow setpoint for the boiler. The thermostat itself requires little configuration as, once connected to your zigbee network, it simply listens to all demands that are reported to it. Which TRVs to include and their settings, e.g. the frequency of heating demand reports and setpoints, or TRV-specific heating schedules are all managed by your favourite, existing zigbee network management system.

Optionally, all OpenTherm data can be logged via an attribute report on a custom OpenTherm cluster. It supports on/off schedules to override TRV schedules, e.g. to turn off the heating system entirely for some parts of a day or week where TRVs don't support schedules.

## Prerequisites

You need the [nRF Connect SDK](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/index.html).

## Configure

### For the Xiao BLE:

```
west build --build-dir build --pristine --board xiao_ble --no-sysbuild -- -DNCS_TOOLCHAIN_VERSION=2.5.0 -DCONF_FILE=prj_xiao_ble.conf
```

### For the BBC micro:bit v2:

```
west build --build-dir build --pristine --board bbc_microbit_v2 --no-sysbuild -- -DNCS_TOOLCHAIN_VERSION=2.5.0 -DCONF_FILE=prj_bbc_microbit_v2.conf
```

## Build

```
cd build
west build
```

The nRF Connect SDK builds a [Zephyr RTOS](https://github.com/zephyrproject-rtos/zephyr) application. The final output is a binary file in the build directory (`zephyr/merged.hex` and/or `zephyr/zephyr.elf`) that can be flashed onto your chip using `west flash`.