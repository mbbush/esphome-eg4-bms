#include "eg4_bms.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace eg4_bms {

static const char *const TAG = "eg4_bms";

static const uint8_t FUNCTION_READ_HOLDING = 0x03;
static const uint8_t MAX_NO_RESPONSE_COUNT = 5;

// Register addresses (from EG4 protocol documentation)
static const uint16_t REG_VOLTAGE = 0x0000;          // Total voltage (10mV)
static const uint16_t REG_CURRENT = 0x0001;          // Current (10mA, signed)
static const uint16_t REG_CELL_VOLTAGE = 0x0002;     // Cell voltages start (mV, 16 cells)
static const uint16_t REG_TEMP_PCB = 0x0018;         // PCB temperature (°C)
static const uint16_t REG_TEMP_AVG = 0x0019;         // Average temperature (°C)
static const uint16_t REG_TEMP_MAX = 0x001A;         // Max temperature (°C)
static const uint16_t REG_CAP_REMAINING = 0x001B;    // Remaining capacity
static const uint16_t REG_MAX_CHARGE_CURRENT = 0x001C;  // Max charging current
static const uint16_t REG_SOH = 0x001D;              // State of health (0-100%)
static const uint16_t REG_SOC = 0x001E;              // State of charge (0-100%)
static const uint16_t REG_STATUS = 0x001F;           // Status
static const uint16_t REG_WARNING = 0x0020;          // Warnings
static const uint16_t REG_PROTECTION = 0x0021;       // Protection status
static const uint16_t REG_ERROR = 0x0022;            // Error code
static const uint16_t REG_CYCLE_COUNT = 0x0023;      // Cycle counts (4 bytes)
static const uint16_t REG_FULL_CAPACITY = 0x0025;    // Full capacity (4 bytes)
static const uint16_t REG_TEMPS = 0x0029;            // Temperature sensors (6 bytes)
static const uint16_t REG_CELL_NUM = 0x002C;         // Cell count
static const uint16_t REG_DESIGNED_CAPACITY = 0x002D;  // Designed capacity
static const uint16_t REG_MODEL = 0x0105;            // Model (24 bytes)
static const uint16_t REG_FW_VERSION = 0x0117;       // Firmware version (6 bytes)
static const uint16_t REG_SERIAL_NO = 0x0120;        // Serial number (16 bytes)

void EG4Bms::dump_config() {
  ESP_LOGCONFIG(TAG, "EG4 BMS:");
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_);
  LOG_BINARY_SENSOR("  ", "Online Status", this->online_status_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "Heating", this->heating_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "Charging", this->charging_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "Discharging", this->discharging_binary_sensor_);
  LOG_SENSOR("  ", "Total Voltage", this->total_voltage_sensor_);
  LOG_SENSOR("  ", "Current", this->current_sensor_);
  LOG_SENSOR("  ", "Power", this->power_sensor_);
  LOG_SENSOR("  ", "State of Charge", this->state_of_charge_sensor_);
  LOG_TEXT_SENSOR("  ", "Status", this->status_text_sensor_);
}

float EG4Bms::get_setup_priority() const { return setup_priority::DATA; }

void EG4Bms::update() {
  if (this->no_response_count_ >= MAX_NO_RESPONSE_COUNT) {
    this->publish_device_unavailable_();
    ESP_LOGW(TAG, "No response from BMS (address 0x%02X)", this->address_);
  }

  this->track_online_status_();
  this->no_response_count_++;

  // Request different register blocks in sequence
  switch (this->request_step_) {
    case 0:
      // Request main data block (voltage, current, cell voltages, temps, SOC, status, warnings, etc.)
      this->send(FUNCTION_READ_HOLDING, REG_VOLTAGE, 0x30);  // Read 48 registers
      break;
    case 1:
      // Request cycle count and capacities (4-byte values)
      this->send(FUNCTION_READ_HOLDING, REG_CYCLE_COUNT, 0x06);  // Read 6 registers
      break;
    case 2:
      // Request model info (every 10th update to reduce traffic)
      if ((this->request_step_ % 30) == 0) {
        this->send(FUNCTION_READ_HOLDING, REG_MODEL, 0x0C);  // Read 12 registers (24 bytes)
      }
      break;
    case 3:
      // Request firmware version (every 10th update)
      if ((this->request_step_ % 30) == 0) {
        this->send(FUNCTION_READ_HOLDING, REG_FW_VERSION, 0x03);  // Read 3 registers (6 bytes)
      }
      break;
    case 4:
      // Request serial number (every 10th update)
      if ((this->request_step_ % 30) == 0) {
        this->send(FUNCTION_READ_HOLDING, REG_SERIAL_NO, 0x08);  // Read 8 registers (16 bytes)
      }
      break;
  }

  this->request_step_ = (this->request_step_ + 1) % 5;
}

void EG4Bms::on_modbus_data(const std::vector<uint8_t> &data) {
  this->reset_online_status_tracker_();

  if (data.size() < 5) {
    ESP_LOGW(TAG, "Invalid Modbus response length: %d", data.size());
    return;
  }

  uint8_t address = data[0];
  uint8_t function = data[1];

  // Check for error response
  if ((function & 0x80) != 0) {
    ESP_LOGW(TAG, "Modbus error response: 0x%02X", data[2]);
    return;
  }

  if (address != this->address_) {
    return;  // Not for this device
  }

  if (function != FUNCTION_READ_HOLDING) {
    ESP_LOGW(TAG, "Unexpected function code: 0x%02X", function);
    return;
  }

  uint8_t byte_count = data[2];
  
  if (data.size() < (size_t)(3 + byte_count + 2)) {
    ESP_LOGW(TAG, "Incomplete response");
    return;
  }

  this->on_status_data_(data);
}

void EG4Bms::on_status_data_(const std::vector<uint8_t> &data) {
  auto get_16bit = [&](size_t i) -> uint16_t {
    return (uint16_t(data[i]) << 8) | uint16_t(data[i + 1]);
  };

  auto get_32bit = [&](size_t i) -> uint32_t {
    return (uint32_t(data[i]) << 24) | (uint32_t(data[i + 1]) << 16) | 
           (uint32_t(data[i + 2]) << 8) | uint32_t(data[i + 3]);
  };

  const uint8_t *payload = &data[3];  // Skip address, function, byte count
  size_t byte_count = data[2];

  ESP_LOGV(TAG, "Processing %d bytes of data", byte_count);

  // Determine which register block this is based on byte count
  if (byte_count == 0x60) {  // 96 bytes = 48 registers (main data block)
    // Total voltage (register 0x0000) - 10mV units
    float total_voltage = get_16bit(0) * 0.01f;
    this->publish_state_(this->total_voltage_sensor_, total_voltage);

    // Current (register 0x0001) - 10mA units, signed
    int16_t current_raw = (int16_t) get_16bit(2);
    float current = current_raw * 0.01f;
    this->publish_state_(this->current_sensor_, current);

    // Calculate power
    float power = total_voltage * current;
    this->publish_state_(this->power_sensor_, power);
    this->publish_state_(this->charging_power_sensor_, power > 0 ? power : 0);
    this->publish_state_(this->discharging_power_sensor_, power < 0 ? -power : 0);

    // Cell voltages (registers 0x0002-0x0011) - mV units
    float min_cell_voltage = 10.0f;
    float max_cell_voltage = 0.0f;
    float sum_cell_voltage = 0.0f;
    uint8_t valid_cells = 0;

    for (uint8_t i = 0; i < 16; i++) {
      uint16_t cell_mv = get_16bit(4 + i * 2);
      if (cell_mv > 0 && cell_mv < 5000) {  // Valid range check
        float cell_voltage = cell_mv * 0.001f;
        this->publish_state_(this->cells_[i].cell_voltage_sensor_, cell_voltage);
        
        if (cell_voltage < min_cell_voltage) min_cell_voltage = cell_voltage;
        if (cell_voltage > max_cell_voltage) max_cell_voltage = cell_voltage;
        sum_cell_voltage += cell_voltage;
        valid_cells++;
      }
    }

    if (valid_cells > 0) {
      this->publish_state_(this->min_cell_voltage_sensor_, min_cell_voltage);
      this->publish_state_(this->max_cell_voltage_sensor_, max_cell_voltage);
      this->publish_state_(this->delta_cell_voltage_sensor_, max_cell_voltage - min_cell_voltage);
      this->publish_state_(this->average_cell_voltage_sensor_, sum_cell_voltage / valid_cells);
    }

    // Temperatures (registers 0x0018-0x001A) - °C, signed
    int16_t pcb_temp = (int16_t) get_16bit(36);
    int16_t avg_temp = (int16_t) get_16bit(38);
    int16_t max_temp = (int16_t) get_16bit(40);
    
    this->publish_state_(this->pcb_temperature_sensor_, (float) pcb_temp);
    this->publish_state_(this->avg_temperature_sensor_, (float) avg_temp);
    this->publish_state_(this->max_temperature_sensor_, (float) max_temp);

    // Remaining capacity (register 0x001B)
    uint16_t remaining_cap = get_16bit(42);
    this->publish_state_(this->remaining_capacity_sensor_, (float) remaining_cap);

    // Max charge current (register 0x001C)
    uint16_t max_charge_current = get_16bit(44);
    this->publish_state_(this->max_charge_current_sensor_, (float) max_charge_current);

    // SOH (register 0x001D) - percentage
    uint16_t soh = get_16bit(46);
    this->publish_state_(this->state_of_health_sensor_, (float) soh);

    // SOC (register 0x001E) - percentage
    uint16_t soc = get_16bit(48);
    this->publish_state_(this->state_of_charge_sensor_, (float) soc);

    // Status (register 0x001F)
    uint16_t status = get_16bit(50);
    this->publish_state_(this->status_text_sensor_, this->decode_status_(status));
    
    // Decode heating, charging, discharging from status
    bool heating = (status & 0x8000) != 0;
    bool charging = (status & 0x000F) == 0x0001;
    bool discharging = (status & 0x000F) == 0x0002;
    
    this->publish_state_(this->heating_binary_sensor_, heating);
    this->publish_state_(this->charging_binary_sensor_, charging);
    this->publish_state_(this->discharging_binary_sensor_, discharging);

    // Warnings (register 0x0020)
    uint16_t warnings = get_16bit(52);
    this->publish_state_(this->warnings_text_sensor_, this->decode_warnings_(warnings));

    // Protection (register 0x0021)
    uint16_t protection = get_16bit(54);
    this->publish_state_(this->protection_text_sensor_, this->decode_protection_(protection));

    // Error (register 0x0022)
    uint16_t error = get_16bit(56);
    this->publish_state_(this->error_text_sensor_, this->decode_error_(error));

  } else if (byte_count == 0x0C) {  // 12 bytes = 6 registers (cycle count and capacities)
    // Cycle count (registers 0x0023-0x0024) - 32-bit
    uint32_t cycle_count = get_32bit(0);
    this->publish_state_(this->cycle_count_sensor_, (float) cycle_count);

    // Full capacity (registers 0x0025-0x0026) - 32-bit, mAh units
    uint32_t full_capacity = get_32bit(4);
    this->publish_state_(this->full_capacity_sensor_, full_capacity / 3600.0f);  // Convert to Ah

  } else if (byte_count == 0x18) {  // 24 bytes = 12 registers (model)
    std::string model((char *) payload, 24);
    // Trim null characters and whitespace
    model.erase(model.find_last_not_of(" \0") + 1);
    this->publish_state_(this->model_text_sensor_, model);

  } else if (byte_count == 0x06) {  // 6 bytes = 3 registers (firmware version)
    std::string firmware((char *) payload, 6);
    firmware.erase(firmware.find_last_not_of(" \0") + 1);
    this->publish_state_(this->firmware_text_sensor_, firmware);

  } else if (byte_count == 0x10) {  // 16 bytes = 8 registers (serial number)
    std::string serial((char *) payload, 16);
    serial.erase(serial.find_last_not_of(" \0") + 1);
    this->publish_state_(this->serial_number_text_sensor_, serial);
  }
}

std::string EG4Bms::decode_status_(uint16_t status) {
  std::string result;
  
  bool heating = (status & 0x8000) != 0;
  uint8_t state = status & 0x000F;
  
  if (heating) result += "Heating+";
  
  switch (state) {
    case 0x00: result += "Standby"; break;
    case 0x01: result += "Charging"; break;
    case 0x02: result += "Discharging"; break;
    case 0x04: result += "Protect"; break;
    case 0x08: result += "Charging Limited"; break;
    default: result += "Unknown"; break;
  }
  
  return result;
}

std::string EG4Bms::decode_warnings_(uint16_t warnings) {
  if (warnings == 0) return "None";
  
  std::string result;
  if (warnings & 0x0001) result += "Pack OV;";
  if (warnings & 0x0002) result += "Cell OV;";
  if (warnings & 0x0004) result += "Pack UV;";
  if (warnings & 0x0008) result += "Cell UV;";
  if (warnings & 0x0010) result += "Charge OC;";
  if (warnings & 0x0020) result += "Discharge OC;";
  if (warnings & 0x0040) result += "Abnormal Temp;";
  if (warnings & 0x0080) result += "MOS Overheat;";
  if (warnings & 0x0100) result += "Charge OT;";
  if (warnings & 0x0200) result += "Discharge OT;";
  if (warnings & 0x0400) result += "Charge UT;";
  if (warnings & 0x0800) result += "Discharge UT;";
  if (warnings & 0x1000) result += "Low Capacity;";
  if (warnings & 0x2000) result += "Float Stopped;";
  
  if (!result.empty() && result.back() == ';') result.pop_back();
  return result;
}

std::string EG4Bms::decode_protection_(uint16_t protection) {
  if (protection == 0) return "None";
  
  std::string result;
  if (protection & 0x0001) result += "Pack OV;";
  if (protection & 0x0002) result += "Cell OV;";
  if (protection & 0x0004) result += "Pack UV;";
  if (protection & 0x0008) result += "Cell UV;";
  if (protection & 0x0010) result += "Charge OC;";
  if (protection & 0x0020) result += "Discharge OC;";
  if (protection & 0x0040) result += "Abnormal Temp;";
  if (protection & 0x0080) result += "MOS Overheat;";
  if (protection & 0x0100) result += "Charge OT;";
  if (protection & 0x0200) result += "Discharge OT;";
  if (protection & 0x0400) result += "Charge UT;";
  if (protection & 0x0800) result += "Discharge UT;";
  if (protection & 0x1000) result += "Low Capacity;";
  if (protection & 0x2000) result += "Discharge SC;";
  
  if (!result.empty() && result.back() == ';') result.pop_back();
  return result;
}

std::string EG4Bms::decode_error_(uint16_t error) {
  if (error == 0) return "None";
  
  std::string result;
  if (error & 0x0001) result += "Voltage Error;";
  if (error & 0x0002) result += "Temperature Error;";
  if (error & 0x0004) result += "Current Flow Error;";
  if (error & 0x0010) result += "Cell Unbalance;";
  
  if (!result.empty() && result.back() == ';') result.pop_back();
  return result;
}

void EG4Bms::publish_state_(binary_sensor::BinarySensor *binary_sensor, const bool &state) {
  if (binary_sensor != nullptr) {
    binary_sensor->publish_state(state);
  }
}

void EG4Bms::publish_state_(sensor::Sensor *sensor, float value) {
  if (sensor != nullptr && !std::isnan(value)) {
    sensor->publish_state(value);
  }
}

void EG4Bms::publish_state_(text_sensor::TextSensor *text_sensor, const std::string &state) {
  if (text_sensor != nullptr && !state.empty()) {
    text_sensor->publish_state(state);
  }
}

void EG4Bms::reset_online_status_tracker_() {
  this->no_response_count_ = 0;
  this->publish_state_(this->online_status_binary_sensor_, true);
}

void EG4Bms::track_online_status_() {
  if (this->no_response_count_ < MAX_NO_RESPONSE_COUNT) {
    this->publish_state_(this->online_status_binary_sensor_, true);
  }
}

void EG4Bms::publish_device_unavailable_() {
  this->publish_state_(this->online_status_binary_sensor_, false);
}

}  // namespace eg4_bms
}  // namespace esphome
