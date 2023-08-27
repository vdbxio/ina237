#pragma once

#include <cstring>
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace ina237 {

enum INA237ADCRange : uint8_t {
  INA237_ADCRANGE_GAIN4 = 0b00,
  INA237_ADCRANGE_GAIN1 = 0b01,
};

struct INA237Component : public PollingComponent, public i2c::I2CDevice {
  template<class T> bool write_structure(uint8_t a_register, T const &data) {
    return this->write_bytes(a_register, reinterpret_cast<uint8_t const *>(&data), sizeof(T));
  }

  template <class T> bool read_structure(uint8_t a_register, T& data) {
    return this->read_bytes(a_register, reinterpret_cast<uint8_t*>(&data), sizeof(T));
  }

  void setup() override;
  void setup_configuration();
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;

  void set_shunt_resistance_ohm(float value) { this->shunt_resistance_ohm_ = value; }
  void set_max_current(float value) { this->max_current_ = value; }
  void set_max_voltage(float value) { this->max_voltage_ = value; }
  void set_shunt_voltage_sensor(sensor::Sensor *value) { this->shunt_voltage_sensor_ = value; }
  void set_bus_voltage_sensor(sensor::Sensor *value) { this->bus_voltage_sensor_ = value; }
  void set_temperature_sensor(sensor::Sensor *value) { this->temperature_sensor_ = value; }
  void set_current_sensor(sensor::Sensor *value) { this->current_sensor_ = value; }
  void set_power_sensor(sensor::Sensor *value) { this->power_sensor_ = value; }
  void set_adc_range(INA237ADCRange value) { this->adc_range_ = value; }

  // no-ops for now
  // TODO: Change this to maybe use `std::chrono`?
  void set_conversion_delay(uint8_t value) {}

  void set_shunt_undervoltage_threshold(float value) {}
  void set_shunt_overvoltage_threshold(float value) {}

  void set_bus_undervoltage_threshold(float value) {}
  void set_bus_overvoltage_threshold(float value) {}

  void set_temperature_overlimit_threshold(float value) {}
  void set_power_overlimit_threshold(float value) {}
  void set_undercurrent_threshold(float value) {}
  void set_overcurrent_threshold(float value) {}

 protected:
  float max_current_lsb_() const;
  float r_shunt_() const;

  float shunt_resistance_ohm_;
  float max_current_;
  float max_voltage_;
  INA237ADCRange adc_range_;

  sensor::Sensor *shunt_voltage_sensor_{nullptr};
  sensor::Sensor *bus_voltage_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *power_sensor_{nullptr};
};

}  // namespace ina237
}  // namespace esphome
