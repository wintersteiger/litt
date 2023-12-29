# litt

litt is a library of C++ templates for rapid prototyping of boiler thermostats and to provide a common infrastructure for heating experiments. See [Examples](examples/README.md).

## Disclaimers

- litt and its example applications are incomplete research projects, not finished products.
- There is no backwards compatibility and there are no fancy installers.
- There are no plans to obtain any certifications.
- Any experiments with uncertified custom boards and/or software will invalidate you boiler's warranty.
- There may be serious bugs. Make sure you can afford to replace your boiler if something goes horribly wrong.
- Without your active involvement, the example applications will not save you money.

## Requirements

Most application of litt will require some type of microcontroller, SoC, or similar, as well as some type of boiler interface. For instance, a [Raspberry Pi Pico W](https://www.raspberrypi.com/products/raspberry-pi-pico/) with an [OpenTherm adapter](https://ihormelnyk.com/opentherm_adapter) (or similar) is an inexpensive way to get started. Whatever your choice, make sure that you are comfortable with building and flashing your chip's SDK example applications and that your toolchain supports C++.

## Contributing

This project welcomes contributions and suggestions. To avoid disappointment, please submit an issue outlining the changes or additions you would like to contribute before investing a significant amount of time.