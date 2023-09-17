#include "ina237.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#include <cmath>
#include <cstring>
#include <cstdint>
#include <type_traits>
#include <bitset>

namespace {

// flip all bits, add 1 to obtain the binary value, then convert to decimal from there.
// we read the MSB to see if we add a negative value or not.
template <class T=int16_t, typename ::std::enable_if<sizeof(T) == 2, int>::type=__LINE__>
T to_decimal(uint16_t value) noexcept {
  std::bitset<16> bits { value };
  auto negative = bits[15];
  bits.flip();
  T result = static_cast<T>(bits.to_ulong()) + 1;
  if (negative) { return -result; }
  return result;
}

} /* nameless namespace */

namespace esphome {
namespace ina237 {

template<class T> typename std::underlying_type<T>::type enum_cast(T const &value) {
  return static_cast<typename std::underlying_type<T>::type>(value);
}

// https://www.ti.com/document-viewer/INA237/datasheet/GUID-EF86D8BB-7305-46E2-B3E2-C8EB2D26865C#TITLE-SBOSA20T4877401-63

[[maybe_unused]] static constexpr char const *TAG = "ina237";

[[maybe_unused]] static constexpr uint8_t INA237_REGISTER_CONFIGURATION = 0x00;
[[maybe_unused]] static constexpr uint8_t INA237_REGISTER_ADC_CONFIGURATION = 0x01;
[[maybe_unused]] static constexpr uint8_t INA237_REGISTER_SHUNT_CALIBIRATION = 0x02;
[[maybe_unused]] static constexpr uint8_t INA237_REGISTER_SHUNT_VOLTAGE = 0x04;
[[maybe_unused]] static constexpr uint8_t INA237_REGISTER_BUS_VOLTAGE = 0x05;
[[maybe_unused]] static constexpr uint8_t INA237_REGISTER_TEMPERATURE = 0x06;
[[maybe_unused]] static constexpr uint8_t INA237_REGISTER_CURRENT = 0x07;
[[maybe_unused]] static constexpr uint8_t INA237_REGISTER_POWER = 0x08;
[[maybe_unused]] static constexpr uint8_t INA237_REGISTER_DIAGNOSTIC_ALERT = 0x0b;
[[maybe_unused]] static constexpr uint8_t INA237_REGISTER_SHUNT_OVERVOLTAGE_THRESHOLD = 0x0c;
[[maybe_unused]] static constexpr uint8_t INA237_REGISTER_SHUNT_UNDERVOLTAGE_THRESHOLD = 0x0d;
[[maybe_unused]] static constexpr uint8_t INA237_REGISTER_BUS_OVERVOLTAGE_THRESHOLD = 0x0e;
[[maybe_unused]] static constexpr uint8_t INA237_REGISTER_BUS_UNDERVOLTAGE_THRESHOLD = 0x0f;
[[maybe_unused]] static constexpr uint8_t INA237_REGISTER_TEMPERATURE_LIMIT = 0x10;
[[maybe_unused]] static constexpr uint8_t INA237_REGISTER_POWER_LIMIT = 0x11;
[[maybe_unused]] static constexpr uint8_t INA237_REGISTER_MANUFACTURER_ID = 0x3e;

static constexpr float INA237_SHUNT_VOLTAGE_LSB_RESOLUTION[2] = {5e-6f, 1.25e-6f};
static constexpr float INA237_TEMPERATURE_LSB_RESOLUTION = 125e-3f;
static constexpr float INA237_BUS_VOLTAGE_LSB_RESOLUTION = 3.125e-3f;
static constexpr float INA237_CURRENT_LSB_RESOLUTION = 305.176e-6f;
static constexpr float INA237_POWER_LSB_RESOLUTION = 61.035156e-6f;

/* Reversed in order according to 7.6.1.1 table because of endian-ness */
struct INA237Config {
  uint16_t reserved2 : 4;
  uint16_t adcrange : 1;
  uint16_t reserved1 : 1;
  uint16_t conversion_delay : 8;
  uint16_t reserved0 : 1;
  uint16_t reset : 1;
};

/* Reversed in order according to 7.6.1.2 table because of endian-ness */
//struct INA237ADCConfig {
//  uint16_t sample_average : 3;
//  uint16_t temperature_conversion : 3;
//  uint16_t shunt_conversion : 3;
//  uint16_t bus_conversion : 3;
//  uint16_t mode : 4;
//};

/* Nothing makes sense anymore */
struct INA237ADCConfig {
  uint16_t mode : 4;
  uint16_t bus_conversion : 3;
  uint16_t shunt_conversion : 3;
  uint16_t temperature_conversion : 3;
  uint16_t sample_average : 3;
};

struct INA237Temperature {
  uint16_t reserved : 4;
  uint16_t dietemp : 12;
};

void INA237Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up INA237...");
  INA237Config clear{};
  clear.reset = 0x01;
  // Config Register
  // 0b1000'0000'0000'0000 << 15 RESET bit (1 -> trigger reset)
  if (!this->write_structure(INA237_REGISTER_CONFIGURATION, clear)) {
    this->mark_failed();
    return;
  }

  delay(1);

  INA237ADCConfig adcconfig{};
  adcconfig.mode = 0x0f; /* continuous bus voltage, shunt voltage, and temperature */
  adcconfig.sample_average = 0x02; /* 16 samples */
  adcconfig.temperature_conversion = 0x05; // default conversion time
  adcconfig.shunt_conversion = 0x05;
  adcconfig.bus_conversion = 0x05;
  if (!this->write_structure(INA237_REGISTER_ADC_CONFIGURATION, adcconfig)) {
    this->mark_failed();
    return;
  }


  this->setup_configuration();
}

void INA237Component::setup_configuration() {
  //
  //  INA237Config config{};
  //  config.adcrange = enum_cast<INA237ADCRange>(this->adc_range_);
  //  std::bitset<16> bits = *reinterpret_cast<uint16_t*>(&config);
  //  ESP_LOGCONFIG(TAG, "Checking raw config dump to uint16_t: %s", bits.to_string().c_str());
  //
  //  // 0b00xx'xxxx'xx00'0000 << ADC Conversion delay in steps of 2ms
  //  // 0b0000'0000'000x'0000 << ADC Range selection (0 = 1.6386mV | 1 = 40.96mV)
  //  INA237ADCConfig adc_config{};
  //  adc_config.sample_average = 0x00; /* 0h = 1 */
  //  adc_config.mode = 0x0f; /* continuous bus voltage, shunt voltage, and temperature */
  //
  //  /* TODO: Make sure this is the correct order. Check by swapping each blit */
  //  ESP_LOGVV(TAG, "Attempting to set ADC configuration register");
  //  if (!this->write_structure(INA237_REGISTER_ADC_CONFIGURATION, &adc_config)) {
  //    this->mark_failed();
  //    return;
  //  }
  //
  //  ESP_LOGVV(TAG, "Attempting to set configuration register");
  //  if (!this->write_structure(INA237_REGISTER_CONFIGURATION, &config)) {
  //    this->mark_failed();
  //    return;
  //  }
  //
  // Equation 1 => SHUNT_CAL = (819.2 * 10**6) * CURRENT_LSB * Rshunt
  auto shunt_calibration = uint16_t(819.2e6f * this->max_current_lsb_() * this->r_shunt_());

  ESP_LOGVV(TAG, "Attempting to set shunt calibration register");
  if (!this->write_byte_16(INA237_REGISTER_SHUNT_CALIBIRATION, shunt_calibration)) {
    this->mark_failed();
    return;
  }
}

void INA237Component::dump_config() {
  if (this->is_failed()) {
    ESP_LOGCONFIG(TAG, "INA237 is marked as failed, attempting emergency setup");
    this->setup_configuration();
  }

  ESP_LOGCONFIG(TAG, "INA237:");
  LOG_I2C_DEVICE(this);

  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with INA237 failed!");
    return;
  }
  LOG_UPDATE_INTERVAL(this);

  LOG_SENSOR("  ", "Bus Voltage", this->bus_voltage_sensor_);
  LOG_SENSOR("  ", "Shunt Voltage", this->shunt_voltage_sensor_);
  LOG_SENSOR("  ", "Current", this->current_sensor_);
  LOG_SENSOR("  ", "Power", this->power_sensor_);
}

float INA237Component::get_setup_priority() const { return setup_priority::DATA; }

// Equation 2 => Max_Current_LSB = Maximum Expected Current / 2**15
// max current times 2**-15 is the same equation.
float INA237Component::max_current_lsb_() const { return ldexpf(this->max_current_, -15); }


// Rshunt is the resistance value of the external shunt used to develop the differential voltage
float INA237Component::r_shunt_() const { return this->shunt_resistance_ohm_ * (this->adc_range_ ? 4 : 1); }

void INA237Component::update() {
  if (this->shunt_voltage_sensor_ != nullptr) {
    uint16_t raw_shunt_voltage;
    if (!this->read_byte_16(INA237_REGISTER_SHUNT_VOLTAGE, &raw_shunt_voltage)) {
      this->status_set_warning();
      return;
    }
    auto value = to_decimal(raw_shunt_voltage) * INA237_SHUNT_VOLTAGE_LSB_RESOLUTION[this->adc_range_];
    // We publish this as milli-volts
    this->shunt_voltage_sensor_->publish_state(value * 1000.0f);
  }

  if (this->bus_voltage_sensor_ != nullptr) {
    uint16_t raw_bus_voltage;
    if (!this->read_byte_16(INA237_REGISTER_BUS_VOLTAGE, &raw_bus_voltage)) {
      this->status_set_warning();
      return;
    }
    this->bus_voltage_sensor_->publish_state(to_decimal<uint16_t>(raw_bus_voltage) * INA237_BUS_VOLTAGE_LSB_RESOLUTION);
  }

  if (this->temperature_sensor_ != nullptr) {
    // INA237Temperature raw_temperature {};
    // if (!this->read_structure(INA237_REGISTER_TEMPERATURE, raw_temperature)) {
    //   this->status_set_warning();
    //   return;
    // }
    uint16_t raw_temperature;
    if (!this->read_byte_16(INA237_REGISTER_TEMPERATURE, &raw_temperature)) {
      this->status_set_warning();
      return;
    }
    auto value = static_cast<float>(to_decimal(raw_temperature >>= 4)) * INA237_TEMPERATURE_LSB_RESOLUTION;
    this->temperature_sensor_->publish_state(value);
  }

  if (this->current_sensor_ != nullptr) {
    uint16_t raw_current;
    if (!this->read_byte_16(INA237_REGISTER_CURRENT, &raw_current)) {
      this->status_set_warning();
      return;
    }
    this->current_sensor_->publish_state(to_decimal(raw_current) * INA237_CURRENT_LSB_RESOLUTION);
  }

  if (this->power_sensor_ != nullptr) {
    uint8_t array[3] = {0};
    if (!this->read_bytes(INA237_REGISTER_POWER, array, 3)) {
      this->status_set_warning();
      return;
    }
    auto power = static_cast<float>(encode_uint24(array[0], array[1], array[2]));
    this->power_sensor_->publish_state(power * INA237_POWER_LSB_RESOLUTION);
  }

  this->status_clear_warning();
}

} /* namespace ina237 */
} /* namespace esphome */
