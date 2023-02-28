# ATC BLE Thermometer repeater

A simple BLE messages repeater for Xiaomi Thermometer (LYWSD03MMC) flashed with [awesome pvvx's/ATC firmware](https://github.com/pvvx/ATC_MiThermometer).
A primary use case is to extend working range for [our thermostat](https://diyless.com/product/opentherm-thermostat), because quite often a thermometer is placed at some distance from the boiler with obstacles for BLE signal (concrete walls/different floors etc).

To use the repeater - upload the sketch to your board, then plug the board into some power adapter (any decent wall brick charger would be enought). Repeater should be placed in the halfway between BLE Thermometer and signal receiver or closer to the receiver, if midpoint isnt reachable.

It can run on ESP32 BLE enabled SoC's like ESP32, ESP32-S3, ESP32-C3, ESP32-C6. Its better to use S3/C3/C6 versions, they provide much better BLE performance over old fashioned ESP32. Of course, you can modify the sketch to run it on any BLE SoC i.e. Nordic's nRF5x, Telink's TLSR825 etc.

Also you can use some compact board like [Beetle ESP32-C3](https://www.dfrobot.com/product-2566.html) to create a tiny repeater which can be easily hidden somewhere (i.e. plug into and place behind a TV or near the router).

If you need some extended functionality like routing BLE messages to some other target like HomeAssistant, MQTT or other system - consider ready products like [BTHome](https://bthome.io/), [ESPHome](https://esphome.io/), [Tasmota](https://tasmota.github.io/), [Passive BLE Monitor integration](https://github.com/custom-components/ble_monitor), it might be faster than developing your own.

Suggestions and/or improvements are welcome.
