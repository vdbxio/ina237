# Overview

This is the [INA237][1] External Component for ESPHome specifically for the [PwrTool 500][3] hardware from [VDBX.io][4]

## YAML Config
```yaml
external_components:
    - source: github://vdbxio/ina237@main
      components: [ina237]
```
and 

```yaml
sensor:
  - platform: ina237
    address: 0x40
    shunt_resistance: 0.0001 ohm
    temperature:
      name: INA237 Temperature
    current:
      name: INA237 Current
    power:
      name: INA237 Power  
    bus_voltage:
      name: INA237 Bus Voltage
    shunt_voltage:
      name: INA237 Shunt Voltage
      accuracy_decimals: 6
    max_voltage: 60.0V
    max_current: 500A
    gain: 1x
    update_interval: 500ms

```
## TODO & Notes

- Current and Power seem to be returning as 100x too small - more testing needed
- ADC Config needs to be fixed
      - gain doesn't work
      - need access to in-built conversion times & averaging settings

## Reference
[Ilvesmaki/INA237][2]


[1]: https://www.ti.com/document-viewer/INA237/datasheet/GUID-C4950780-1AF9-4205-AD23-94DEC98F74B5
[2]: https://github.com/Ilvesmaki/INA237
[3]: https://wiki.vdbx.io/products/pwrtool500
[4]: https://www.vdbx.io

