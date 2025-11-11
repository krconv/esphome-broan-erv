#include "broan_erv.h"
#include "esphome/core/log.h"
#include "esphome/components/select/select.h"
#include "esphome/components/sensor/sensor.h"
#include <cstring>
#include <algorithm>

namespace esphome {
namespace broan_erv {

static const char *TAG = "broan_erv";

void BroanERVComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Broan ERV...");
  ESP_LOGCONFIG(TAG, "  Source Address: 0x%02X (%s)", src_address_, get_device_name(src_address_).c_str());
  ESP_LOGCONFIG(TAG, "  Destination Address: 0x%02X (%s)", dst_address_, get_device_name(dst_address_).c_str());
  ESP_LOGCONFIG(TAG, "  Poll Interval: %u ms", poll_interval_);
  ESP_LOGCONFIG(TAG, "  Listen Only Mode: %s", listen_only_ ? "YES" : "NO");

  if (re_de_pin_ != nullptr) {
    re_de_pin_->setup();
    re_de_pin_->pin_mode(gpio::FLAG_OUTPUT | gpio::FLAG_PULLDOWN);
    re_de_pin_->digital_write(false);
  }

  last_poll_time_ = millis();
}

void BroanERVComponent::loop() {
  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);
    process_byte(byte);
  }

  process_flow_control_state();
}

void BroanERVComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Broan ERV Component:");
  ESP_LOGCONFIG(TAG, "  Source Address: 0x%02X (%s)", src_address_, get_device_name(src_address_).c_str());
  ESP_LOGCONFIG(TAG, "  Destination Address: 0x%02X (%s)", dst_address_, get_device_name(dst_address_).c_str());
  ESP_LOGCONFIG(TAG, "  Poll Interval: %u ms", poll_interval_);
  ESP_LOGCONFIG(TAG, "  Listen Only Mode: %s", listen_only_ ? "YES" : "NO");
  LOG_PIN("  RE/DE Pin: ", re_de_pin_);
}

void BroanERVComponent::process_byte(uint8_t byte) {
  rx_buffer_.push_back(byte);

  if (rx_buffer_.size() > RX_BUFFER_MAX_SIZE) {
    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + RX_BUFFER_TRIM_SIZE);
  }

  try_parse_frames();
}

void BroanERVComponent::try_parse_frames() {
  while (!rx_buffer_.empty()) {
    size_t start_pos = 0;
    bool found_start = false;

    for (size_t i = 0; i <= rx_buffer_.size() - 1; i++) {
      if (i + 4 < rx_buffer_.size() &&
          rx_buffer_[i] == FRAME_START_BYTE &&
          rx_buffer_[i + 3] == FRAME_SEPARATOR_BYTE) {
        start_pos = i;
        found_start = true;
        break;
      }
    }

    if (!found_start) {
      if (rx_buffer_.size() > 4) {
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.end() - 4);
      }
      break;
    }

    if (start_pos > 0) {
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + start_pos);
    }

    if (rx_buffer_.size() < FRAME_HEADER_SIZE) {
      break;
    }

    uint8_t payload_len = rx_buffer_[4];
    size_t frame_size = MIN_FRAME_SIZE + payload_len;

    if (rx_buffer_.size() < frame_size) {
      break;
    }

    if (rx_buffer_[frame_size - 1] != FRAME_END_BYTE) {
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }

    ParsedFrame frame;
    if (try_parse_frame(&rx_buffer_[0], frame_size, frame)) {
      log_frame(frame.src, frame.dst, frame.payload);
      handle_received_frame(frame);
    }

    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + frame_size);
  }
}

bool BroanERVComponent::try_parse_frame(const uint8_t* data, size_t len, ParsedFrame& frame) {
  if (len < MIN_FRAME_SIZE) {
    return false;
  }

  if (data[0] != FRAME_START_BYTE || data[len - 1] != FRAME_END_BYTE || data[3] != FRAME_SEPARATOR_BYTE) {
    return false;
  }

  frame.raw_bytes.assign(data, data + len);
  frame.dst = data[1];
  frame.src = data[2];

  uint8_t payload_len = data[4];
  frame.payload.assign(data + FRAME_HEADER_SIZE, data + FRAME_HEADER_SIZE + payload_len);

  uint8_t received_checksum = data[FRAME_HEADER_SIZE + payload_len];
  uint8_t expected_checksum = calc_checksum(data, payload_len);
  frame.checksum_valid = (received_checksum == expected_checksum);

  return true;
}

uint8_t BroanERVComponent::calc_checksum(const uint8_t* data, size_t payload_len) {
  uint32_t sum = 0;
  size_t checksum_boundary = FRAME_HEADER_SIZE + payload_len;

  for (size_t i = 0; i < checksum_boundary; i++) {
    sum += data[i];
  }

  sum &= 0xFF;
  return (-(sum - 1)) & 0xFF;
}

void BroanERVComponent::log_frame(uint8_t src_addr, uint8_t dst_addr, const std::vector<uint8_t>& payload) {
  if (payload.empty()) {
    return;
  }

  uint8_t type = payload[0];
  std::string msg_type = get_message_type(type);


  bool is_flow_control = (type == static_cast<uint8_t>(MessageType::FLOW_CONTROL_RELEASE) ||
                          type == static_cast<uint8_t>(MessageType::FLOW_CONTROL_TAKE));

  std::string src_name = get_device_name(src_addr);
  std::string dst_name = get_device_name(dst_addr);
  if (is_flow_control) {
    ESP_LOGVV(TAG, "%s -> %s : %s", src_name.c_str(), dst_name.c_str(), msg_type.c_str());
  } else {
    ESP_LOGD(TAG, "%s -> %s : %s", src_name.c_str(), dst_name.c_str(), msg_type.c_str());
  }

  switch (type) {
    case static_cast<uint8_t>(MessageType::FLOW_CONTROL_RELEASE):
    case static_cast<uint8_t>(MessageType::FLOW_CONTROL_TAKE):
      break;
    case static_cast<uint8_t>(MessageType::READ_REQUEST):
      log_register_read_request(payload);
      break;
    case static_cast<uint8_t>(MessageType::READ_RESPONSE):
      log_register_read_response(payload);
      break;
    case static_cast<uint8_t>(MessageType::WRITE_REQUEST):
      log_register_write_request(payload);
      break;
    case static_cast<uint8_t>(MessageType::WRITE_RESPONSE):
      log_register_write_response(payload);
      break;
    case static_cast<uint8_t>(MessageType::PING):
    case static_cast<uint8_t>(MessageType::PONG):
      log_ping_pong(payload);
      break;
    default:
      std::string payload_hex;
      for (size_t i = 0; i < payload.size(); i++) {
        char buf[4];
        snprintf(buf, sizeof(buf), "%02X", payload[i]);
        if (i > 0) payload_hex += " ";
        payload_hex += buf;
      }
      ESP_LOGV(TAG, "  Payload: %s", payload_hex.c_str());
      break;
  }
}

std::string BroanERVComponent::get_message_type(uint8_t type_byte) {
  switch (type_byte) {
    case static_cast<uint8_t>(MessageType::READ_REQUEST): return "READ_REQ";
    case static_cast<uint8_t>(MessageType::READ_RESPONSE): return "READ_RESP";
    case static_cast<uint8_t>(MessageType::WRITE_REQUEST): return "WRITE_REQ";
    case static_cast<uint8_t>(MessageType::WRITE_RESPONSE): return "WRITE_RESP";
    case static_cast<uint8_t>(MessageType::FLOW_CONTROL_RELEASE): return "FLOW_CTRL_RELEASE";
    case static_cast<uint8_t>(MessageType::FLOW_CONTROL_TAKE): return "FLOW_CTRL_TAKE";
    case static_cast<uint8_t>(MessageType::PING): return "PING";
    case static_cast<uint8_t>(MessageType::PONG): return "PONG";
    default: return "UNKNOWN";
  }
}

std::string BroanERVComponent::get_device_name(uint8_t addr) {
  switch (addr) {
    case DEVICE_ADDR_ERV:
      return "ERV";
    case DEVICE_ADDR_CONTROLLER:
      return "Controller";
    default:
      char buf[8];
      snprintf(buf, sizeof(buf), "0x%02X", addr);
      return std::string(buf);
  }
}

std::string BroanERVComponent::get_register_name(uint16_t reg) {
  switch (reg) {
    case REG_FAN_MODE:
    case REG_FAN_MODE_ALT:
      return "FanMode";
    case REG_SUPPLY_FAN_RPM:
      return "SupplyFanRPM";
    case REG_EXHAUST_FAN_RPM:
      return "ExhaustFanRPM";
    case REG_SUPPLY_FAN_CFM:
      return "SupplyFanCFM";
    case REG_EXHAUST_FAN_CFM:
      return "ExhaustFanCFM";
    case REG_POWER_DRAW:
      return "PowerDraw";
    case REG_HEARTBEAT:
      return "Heartbeat";
    default:
      char buf[8];
      snprintf(buf, sizeof(buf), "%04X", reg);
      return std::string(buf);
  }
}

std::string BroanERVComponent::get_mode_name(uint8_t mode) {
  switch (mode) {
    case FAN_MODE_OFF:
    case FAN_MODE_INITIALIZING:
      return "Off";
    case FAN_MODE_INTERMITTENT:
      return "Intermittent";
    case FAN_MODE_MINIMUM:
      return "Minimum";
    case FAN_MODE_MAXIMUM:
      return "Maximum";
    case FAN_MODE_MEDIUM:
      return "Medium";
    case FAN_MODE_TURBO:
      return "Turbo";
    case FAN_MODE_AUTO:
      return "Auto";
    default:
      char buf[8];
      snprintf(buf, sizeof(buf), "0x%02X", mode);
      return std::string(buf);
  }
}

void BroanERVComponent::log_register_read_request(const std::vector<uint8_t>& payload) {
  log_register_pair_list(payload);
}

void BroanERVComponent::log_register_read_response(const std::vector<uint8_t>& payload) {
  log_register_value_blocks(payload);
}

void BroanERVComponent::log_register_value(uint16_t reg, const uint8_t* data, size_t len) {
  std::string hex;
  for (size_t j = 0; j < len; j++) {
    char buf[4];
    snprintf(buf, sizeof(buf), "%02X", data[j]);
    if (j > 0) hex += " ";
    hex += buf;
  }

  std::string reg_name = get_register_name(reg);
  if (len == 4) {
    float value;
    memcpy(&value, data, sizeof(float));
    ESP_LOGD(TAG, "  %s (0x%02X%02X) = %.2f", reg_name.c_str(), reg >> 8, reg & 0xFF, value);
  } else if (len == 2) {
    uint16_t val16 = data[0] | (data[1] << 8);
    ESP_LOGD(TAG, "  %s (0x%02X%02X) = %u (0x%04X)", reg_name.c_str(), reg >> 8, reg & 0xFF, val16, val16);
  } else if (len == 1) {
    uint8_t val = data[0];
    ESP_LOGD(TAG, "  %s (0x%02X%02X) = %u (0x%02X)", reg_name.c_str(), reg >> 8, reg & 0xFF, val, val);
  } else {
    ESP_LOGD(TAG, "  %s (0x%02X%02X) = %s", reg_name.c_str(), reg >> 8, reg & 0xFF, hex.c_str());
  }
}

void BroanERVComponent::process_read_response(const std::vector<uint8_t>& payload) {
  size_t i = 1;

  while (i + 2 < payload.size()) {
    uint8_t rlo = payload[i];
    uint8_t rhi = payload[i + 1];
    uint8_t len = payload[i + 2];
    i += 3;

    if (i + len > payload.size()) {
      break;
    }

    uint16_t reg = (rhi << 8) | rlo;
    if (reg == REG_FAN_MODE || reg == REG_FAN_MODE_ALT) {
      uint8_t mode = payload[i];
      if (mode_select_ != nullptr) {
        update_mode_state(mode);
      }
    } else {
      update_sensor_from_register(reg, &payload[i], len);
    }

    i += len;
  }
}

void BroanERVComponent::update_sensor_from_register(uint16_t reg, const uint8_t* data, size_t len) {
  sensor::Sensor *sensor = nullptr;

  switch (reg) {
    case REG_SUPPLY_FAN_RPM:
      sensor = supply_fan_rpm_sensor_;
      break;
    case REG_EXHAUST_FAN_RPM:
      sensor = exhaust_fan_rpm_sensor_;
      break;
    case REG_SUPPLY_FAN_CFM:
      sensor = supply_fan_cfm_sensor_;
      break;
    case REG_EXHAUST_FAN_CFM:
      sensor = exhaust_fan_cfm_sensor_;
      break;
    case REG_POWER_DRAW:
      sensor = power_sensor_;
      break;
    default:
      return;
  }

  if (len == 4) {
    float value;
    memcpy(&value, data, sizeof(float));
    sensor->publish_state(value);
  }
}

void BroanERVComponent::log_register_write_request(const std::vector<uint8_t>& payload) {
  log_register_value_blocks(payload);
}

void BroanERVComponent::log_register_write_response(const std::vector<uint8_t>& payload) {
  log_register_pair_list(payload);
}

void BroanERVComponent::log_register_pair_list(const std::vector<uint8_t>& payload) {
  for (size_t i = 1; i + 1 < payload.size(); i += 2) {
    uint8_t hi = payload[i + 1];
    uint8_t lo = payload[i];
    uint16_t reg = (hi << 8) | lo;
    std::string reg_name = get_register_name(reg);
    ESP_LOGD(TAG, "  %s (0x%02X%02X)", reg_name.c_str(), hi, lo);
  }
}

void BroanERVComponent::log_register_value_blocks(const std::vector<uint8_t>& payload) {
  size_t i = 1;

  while (i + 2 < payload.size()) {
    uint8_t hi = payload[i + 1];
    uint8_t lo = payload[i];
    uint8_t len = payload[i + 2];
    i += 3;

    if (i + len > payload.size()) {
      break;
    }

    uint16_t reg = (hi << 8) | lo;
    log_register_value(reg, &payload[i], len);

    i += len;
  }
}

void BroanERVComponent::log_ping_pong(const std::vector<uint8_t>& payload) {
  if (payload.empty()) {
    return;
  }

  std::string msg_type = (payload[0] == static_cast<uint8_t>(MessageType::PING)) ? "PING" : "PONG";
  std::string text;

  for (size_t i = 1; i < payload.size(); i++) {
    if (payload[i] >= 32 && payload[i] <= 126) {
      text += static_cast<char>(payload[i]);
    }
  }

  ESP_LOGV(TAG, "  text='%s'", text.c_str());
}

void BroanERVComponent::build_and_send_frame(uint8_t dst, uint8_t src, const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> frame;

  frame.push_back(FRAME_START_BYTE);
  frame.push_back(dst);
  frame.push_back(src);
  frame.push_back(FRAME_SEPARATOR_BYTE);
  frame.push_back(static_cast<uint8_t>(payload.size()));

  for (uint8_t byte : payload) {
    frame.push_back(byte);
  }

  uint32_t sum = 0;
  for (uint8_t byte : frame) {
    sum += byte;
  }
  sum &= 0xFF;
  uint8_t checksum = (-(sum - 1)) & 0xFF;
  frame.push_back(checksum);

  frame.push_back(FRAME_END_BYTE);
  frame.push_back(FRAME_TERMINATOR_BYTE);

  if (re_de_pin_ != nullptr) {
    re_de_pin_->digital_write(true);
  }

  for (uint8_t byte : frame) {
    this->write_byte(byte);
  }

  this->flush();

  if (re_de_pin_ != nullptr) {
    re_de_pin_->digital_write(false);
  }

  last_tx_time_ = millis();
}

void BroanERVComponent::send_frame(const std::vector<uint8_t>& payload) {
  build_and_send_frame(dst_address_, src_address_, payload);

  log_frame(src_address_, dst_address_, payload);
}

void BroanERVComponent::send_flow_control(uint8_t flow_byte) {
  std::vector<uint8_t> payload = {flow_byte};
  send_frame(payload);

  if (flow_byte == static_cast<uint8_t>(MessageType::FLOW_CONTROL_TAKE)) {
    flow_state_ = FlowControlState::WE_HAVE_CONTROL;
  } else if (flow_byte == static_cast<uint8_t>(MessageType::FLOW_CONTROL_RELEASE)) {
    flow_state_ = FlowControlState::IDLE;
  }
}

void BroanERVComponent::send_pong() {
  std::vector<uint8_t> payload = {static_cast<uint8_t>(MessageType::PONG)};
  for (size_t i = 0; i < strlen(PONG_IDENTIFIER); i++) {
    payload.push_back(PONG_IDENTIFIER[i]);
  }
  send_frame(payload);
}

void BroanERVComponent::send_read_request(const std::vector<uint16_t>& registers) {
  std::vector<uint8_t> payload;
  payload.push_back(static_cast<uint8_t>(MessageType::READ_REQUEST));

  for (uint16_t reg : registers) {
    payload.push_back(reg & 0xFF);
    payload.push_back((reg >> 8) & 0xFF);
  }

  send_frame(payload);
  flow_state_ = FlowControlState::WAITING_FOR_RESPONSE;
}

void BroanERVComponent::send_heartbeat() {
  std::vector<uint8_t> payload = {
    static_cast<uint8_t>(MessageType::WRITE_REQUEST),
    0x00, 0x50, 0x00
  };
  send_frame(payload);

  flow_state_ = FlowControlState::WAITING_FOR_RESPONSE;
  last_heartbeat_time_ = millis();
}

void BroanERVComponent::handle_received_frame(const ParsedFrame& frame) {
  if (frame.payload.empty()) {
    return;
  }

  if (frame.src != src_address_ && frame.src != dst_address_) {
    return;
  }

  uint8_t type = frame.payload[0];

  if (type == static_cast<uint8_t>(MessageType::FLOW_CONTROL_RELEASE) ||
      type == static_cast<uint8_t>(MessageType::FLOW_CONTROL_TAKE)) {
    handle_flow_control(type, frame.src, frame.dst);
  } else if (type == static_cast<uint8_t>(MessageType::PING) && frame.dst == src_address_) {
    handle_ping(frame.payload);
  } else if (type == static_cast<uint8_t>(MessageType::READ_RESPONSE) && frame.dst == src_address_) {
    process_read_response(frame.payload);
    flow_state_ = FlowControlState::WE_HAVE_CONTROL;
  } else if (type == static_cast<uint8_t>(MessageType::WRITE_RESPONSE) && frame.dst == src_address_) {
    flow_state_ = FlowControlState::WE_HAVE_CONTROL;
  }
}

void BroanERVComponent::handle_flow_control(uint8_t flow_byte, uint8_t src_addr, uint8_t dst_addr) {
  if (flow_byte == static_cast<uint8_t>(MessageType::FLOW_CONTROL_RELEASE)) {
    if (dst_addr == src_address_) {
      if (this->listen_only_) {
        flow_state_ = FlowControlState::ERV_HAS_CONTROL;
      } else {
        send_flow_control(static_cast<uint8_t>(MessageType::FLOW_CONTROL_TAKE));
        flow_state_ = FlowControlState::WE_HAVE_CONTROL;
      }
    } else {
      flow_state_ = FlowControlState::ERV_HAS_CONTROL;
    }
  } else if (flow_byte == static_cast<uint8_t>(MessageType::FLOW_CONTROL_TAKE)) {
    if (src_addr == src_address_) {
      flow_state_ = FlowControlState::WE_HAVE_CONTROL;
    } else {
      flow_state_ = FlowControlState::IDLE;
    }
  }
}

void BroanERVComponent::handle_ping(const std::vector<uint8_t>& payload) {
  if (this->listen_only_) {
    return;
  }
  send_pong();
}

void BroanERVComponent::process_flow_control_state() {
  if (this->listen_only_) {
    return;
  }

  switch (flow_state_) {
    case FlowControlState::ERV_HAS_CONTROL:
      break;

    case FlowControlState::WE_HAVE_CONTROL:
      if (!message_queue_.empty()) {
        PendingMessage msg = message_queue_.front();
        message_queue_.pop();
        send_frame(msg.payload);
        return;
      }

      if (millis() - last_heartbeat_time_ >= heartbeat_interval_) {
        send_heartbeat();
        return;
      }

      if (millis() - last_poll_time_ >= poll_interval_) {
        poll_registers();
        last_poll_time_ = millis();
        return;
      }

      send_flow_control(static_cast<uint8_t>(MessageType::FLOW_CONTROL_RELEASE));
      break;

    case FlowControlState::WAITING_FOR_RESPONSE:
      break;

    case FlowControlState::IDLE:
      break;
  }
}

void BroanERVComponent::poll_registers() {
  std::vector<uint16_t> registers;

  if (mode_select_ != nullptr) registers.push_back(REG_FAN_MODE);
  if (supply_fan_rpm_sensor_ != nullptr) registers.push_back(REG_SUPPLY_FAN_RPM);
  if (exhaust_fan_rpm_sensor_ != nullptr) registers.push_back(REG_EXHAUST_FAN_RPM);
  if (supply_fan_cfm_sensor_ != nullptr) registers.push_back(REG_SUPPLY_FAN_CFM);
  if (exhaust_fan_cfm_sensor_ != nullptr) registers.push_back(REG_EXHAUST_FAN_CFM);
  if (power_sensor_ != nullptr) registers.push_back(REG_POWER_DRAW);

  for (uint16_t reg : poll_registers_) {
    registers.push_back(reg);
  }

  if (!registers.empty()) {
    send_read_request(registers);
  }
}

void BroanERVComponent::set_mode(const std::string &mode) {
  uint8_t mode_hex;

  if (mode == "Off") {
    mode_hex = FAN_MODE_OFF;
  } else if (mode == "Intermittent") {
    mode_hex = FAN_MODE_INTERMITTENT;
  } else if (mode == "Minimum") {
    mode_hex = FAN_MODE_MINIMUM;
  } else if (mode == "Maximum") {
    mode_hex = FAN_MODE_MAXIMUM;
  } else if (mode == "Medium") {
    mode_hex = FAN_MODE_MEDIUM;
  } else if (mode == "Turbo") {
    mode_hex = FAN_MODE_TURBO;
  } else if (mode == "Auto") {
    mode_hex = FAN_MODE_AUTO;
  } else {
    ESP_LOGE(TAG, "Unknown fan mode: %s", mode.c_str());
    return;
  }

  ESP_LOGI(TAG, "Setting fan mode to %s (0x%02X)", mode.c_str(), mode_hex);
  send_write_mode(mode_hex);
}

void BroanERVComponent::send_write_mode(uint8_t mode_hex) {
  std::vector<uint8_t> payload;
  payload.push_back(static_cast<uint8_t>(MessageType::WRITE_REQUEST));
  payload.push_back(REG_FAN_MODE & 0xFF);
  payload.push_back((REG_FAN_MODE >> 8) & 0xFF);
  payload.push_back(0x01);
  payload.push_back(mode_hex);
  // void BroanERVComponent::send_write_fan_mode(uint8_t mode_hex) {
  // std::vector<uint8_t> payload;
  // payload.push_back(0x40);  // WRITE_REQ
  // payload.push_back(0x00);  // Register 0x0020 high byte
  // payload.push_back(0x20);  // Register 0x0020 low byte
  // payload.push_back(0x01);  // 4 bytes of data
  // payload.push_back(mode_hex);

  PendingMessage msg;
  msg.payload = payload;
  msg.timestamp = millis();
  message_queue_.push(msg);
}

void BroanERVComponent::update_mode_state(uint8_t mode_value) {
  if (mode_select_ == nullptr) {
    return;
  }

  std::string mode_name = get_mode_name(mode_value);
  mode_select_->publish_state(mode_name);
}

}  // namespace broan_erv
}  // namespace esphome
