import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import i2c, sensor
from esphome.const import (
    CONF_TEMPERATURE,
    CONF_BUS_VOLTAGE,
    CONF_CURRENT,
    CONF_GAIN,
    CONF_ID,
    CONF_MAX_CURRENT,
    CONF_MAX_VOLTAGE,
    CONF_POWER,
    CONF_SHUNT_RESISTANCE,
    CONF_SHUNT_VOLTAGE,
    UNIT_CELSIUS,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_WATT,
    STATE_CLASS_MEASUREMENT,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_POWER,
)

UNIT_MILLIVOLT = "mV"

DEPENDENCIES = ["i2c"]

ina237_ns = cg.esphome_ns.namespace("ina237")
ina237_gain = ina237_ns.enum("INA237ADCRange")
INA237_GAIN = {
    "4x": ina237_gain.INA237_ADCRANGE_GAIN4,
    "1x": ina237_gain.INA237_ADCRANGE_GAIN1,
}

INA237Component = ina237_ns.class_("INA237Component", cg.PollingComponent, i2c.I2CDevice)

CONFIG_SCHEMA = (
    cv.Schema({
        cv.GenerateID(): cv.declare_id(INA237Component),
        cv.Optional(CONF_BUS_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_SHUNT_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLIVOLT,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CURRENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_POWER): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_SHUNT_RESISTANCE, default=0.1): cv.All(cv.resistance, cv.Range(min=0.0, max=1.0)),
        cv.Optional(CONF_MAX_VOLTAGE, default=30.0): cv.All(cv.voltage, cv.Range(min=0.0, max=60.0)),
        cv.Optional(CONF_MAX_CURRENT, default=3.2): cv.All(cv.current, cv.Range(min=0.0)),
        cv.Optional(CONF_GAIN, default='1x'): cv.enum(INA237_GAIN),
    })
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x40))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_shunt_resistance_ohm(config[CONF_SHUNT_RESISTANCE]))
    cg.add(var.set_max_current(config[CONF_MAX_CURRENT]))
    cg.add(var.set_max_voltage(config[CONF_MAX_VOLTAGE]))
    cg.add(var.set_adc_range(config[CONF_GAIN]))

    if CONF_BUS_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_BUS_VOLTAGE])
        cg.add(var.set_bus_voltage_sensor(sens))

    if CONF_SHUNT_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_SHUNT_VOLTAGE])
        cg.add(var.set_shunt_voltage_sensor(sens))

    if CONF_CURRENT in config:
        sens = await sensor.new_sensor(config[CONF_CURRENT])
        cg.add(var.set_current_sensor(sens))

    if CONF_POWER in config:
        sens = await sensor.new_sensor(config[CONF_POWER])
        cg.add(var.set_power_sensor(sens))

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))
