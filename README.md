# `ESPHome Component for Tigo TAP`
This Project implements a ESPHome Component for monitoring a [Tigo Tap](https://tigoenergy.com/product/tigo-access-point) and connected Solar Modules.
It thereby eleminates the need for the proprietary Tigo Cloud for Monitoring, while retaining more Information at a higher resolution. 
Data can be read serveral times per minute as opposed to 15min/1min(unpaid/paid) readout off the Tigo Cloud.

** Based on the reverse-engineering Projekt [TapTap](https://github.com/willglynn/taptap) by Will Glynn. **

** This is not a reimplementaion of the [Tigo CCA](https://tigoenergy.com/product/cloud-connect-advanced), you still need a Tigo CCA and one or more connected Tigo Tap Modules **
As the Tigo CCA also implements important safety features such as a "Rapid Shutdown" functionality which we don't want to mess with.
Therefore this project is (fore now) just focused on Monitoring the Data provided by connected Solar Modules.

## Hardware Requirements
- ESP32 board
- TTL to RS485 Converter
