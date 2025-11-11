/**
 * This component implements communication with Broan ERV units using a proprietary
 * RS-485 protocol. It supports monitoring sensors (fan speeds, running mode, power)
 * and controlling the fan mode.
 *
 * Protocol Overview:
 * - Physical layer: RS-485 half-duplex serial communication
 * - Frame structure: [START][DST][SRC][SEP][LEN][PAYLOAD...][CHECKSUM][END][TERM]
 * - Flow control: Token-passing mechanism where devices request control before transmitting
 * - Addressing: ERV unit (0x10), Controller (0x11)
 *
 * The protocol uses a flow control mechanism where devices must take control of the bus
 * before sending commands. The component can operate in two modes:
 * - Active mode: Participates in bus communication, polls registers, and sends commands
 * - Listen-only mode: Passively monitors bus traffic without transmitting
 */

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/gpio.h"
#include <vector>
#include <map>
#include <string>
#include <queue>

namespace esphome {

namespace select {
class Select;
}

namespace sensor {
class Sensor;
}

namespace broan_erv {

static constexpr uint8_t FRAME_START_BYTE = 0x01;
static constexpr uint8_t FRAME_SEPARATOR_BYTE = 0x01;
static constexpr uint8_t FRAME_END_BYTE = 0x04;
static constexpr uint8_t FRAME_TERMINATOR_BYTE = 0xFF;

enum class MessageType : uint8_t {
  PING = 0x02,
  PONG = 0x03,
  FLOW_CONTROL_RELEASE = 0x04,
  FLOW_CONTROL_TAKE = 0x05,
  READ_REQUEST = 0x20,
  READ_RESPONSE = 0x21,
  WRITE_REQUEST = 0x40,
  WRITE_RESPONSE = 0x41
};

static constexpr uint16_t REG_FAN_MODE = 0x2000;
static constexpr uint16_t REG_FAN_MODE_ALT = 0x2002;
static constexpr uint16_t REG_TURBO_ENABLE = 0x0022;
static constexpr uint16_t REG_TURBO_DISABLE = 0x2003;
static constexpr uint16_t REG_TURBO = 0x3002;

static constexpr uint16_t REG_SUPPLY_FAN_RPM = 0x1003;
static constexpr uint16_t REG_EXHAUST_FAN_RPM = 0x1004;
static constexpr uint16_t REG_SUPPLY_FAN_CFM = 0x1005;
static constexpr uint16_t REG_EXHAUST_FAN_CFM = 0x1006;
static constexpr uint16_t REG_POWER_DRAW = 0x5023;
static constexpr uint16_t REG_HEARTBEAT = 0x5000;

static constexpr uint8_t DEVICE_ADDR_ERV = 0x10;
static constexpr uint8_t DEVICE_ADDR_CONTROLLER = 0x11;

static constexpr uint8_t FAN_MODE_OFF = 0x01;
static constexpr uint8_t FAN_MODE_INTERMITTENT = 0x08;
static constexpr uint8_t FAN_MODE_MINIMUM = 0x09;
static constexpr uint8_t FAN_MODE_MAXIMUM = 0x0A;
static constexpr uint8_t FAN_MODE_MEDIUM = 0x0B;
static constexpr uint8_t FAN_MODE_TURBO = 0x0C;
static constexpr uint8_t FAN_MODE_AUTO = 0x10;
static constexpr uint8_t FAN_MODE_INITIALIZING = 0x14;

static constexpr size_t RX_BUFFER_MAX_SIZE = 1000;
static constexpr size_t RX_BUFFER_TRIM_SIZE = 500;
static constexpr size_t MIN_FRAME_SIZE = 7;
static constexpr size_t FRAME_HEADER_SIZE = 5;
static constexpr size_t FRAME_FOOTER_SIZE = 2;

static constexpr uint32_t DEFAULT_HEARTBEAT_INTERVAL = 5000; // 5 seconds

static constexpr const char* PONG_IDENTIFIER = "ESP32";

/**
 * Parsed protocol frame
 */
struct ParsedFrame {
  std::vector<uint8_t> raw_bytes;
  uint8_t dst;
  uint8_t src;
  std::vector<uint8_t> payload;
  bool checksum_valid;
};

/**
 * Flow control states for bus arbitration
 *
 * The protocol uses token-passing for bus control. Devices must accept control
 * before transmitting data and release it when done.
 */
enum class FlowControlState {
  IDLE,
  ERV_HAS_CONTROL,
  WE_HAVE_CONTROL,
  WAITING_FOR_RESPONSE
};

/**
 * Message queued for transmission
 */
struct PendingMessage {
  std::vector<uint8_t> payload;
  uint32_t timestamp;
};

/**
 * Main component for Broan ERV communication
 *
 * This component handles all communication with the ERV unit including:
 * - Protocol frame parsing and validation
 * - Flow control and bus arbitration
 * - Register polling for sensor updates
 * - Fan mode control commands
 * - Sensor entity updates
 */
class BroanERVComponent : public uart::UARTDevice, public Component {
 public:
  void setup() override;

  void loop() override;

  void dump_config() override;

  void set_src_address(uint8_t addr) { src_address_ = addr; }
  void set_dst_address(uint8_t addr) { dst_address_ = addr; }

  void set_poll_interval(uint32_t interval) { poll_interval_ = interval; }

  void set_re_de_pin(GPIOPin *pin) { re_de_pin_ = pin; }

  void set_listen_only(bool value) { listen_only_ = value; }

  void set_mode_select(select::Select *select) { mode_select_ = select; }

  void add_poll_register(uint16_t reg) { poll_registers_.push_back(reg); }

  void set_supply_fan_rpm_sensor(sensor::Sensor *sensor) { supply_fan_rpm_sensor_ = sensor; }
  void set_exhaust_fan_rpm_sensor(sensor::Sensor *sensor) { exhaust_fan_rpm_sensor_ = sensor; }
  void set_supply_fan_cfm_sensor(sensor::Sensor *sensor) { supply_fan_cfm_sensor_ = sensor; }
  void set_exhaust_fan_cfm_sensor(sensor::Sensor *sensor) { exhaust_fan_cfm_sensor_ = sensor; }

  void set_power_sensor(sensor::Sensor *sensor) { power_sensor_ = sensor; }

  void set_mode(const std::string &mode);
  void update_mode_state(uint8_t mode_value);

 protected:
  std::vector<uint8_t> rx_buffer_;

  uint8_t src_address_{0};
  uint8_t dst_address_{0};
  uint32_t poll_interval_{0};
  GPIOPin *re_de_pin_{nullptr};
  bool listen_only_{false};
  select::Select *mode_select_{nullptr};
  std::vector<uint16_t> poll_registers_;

  sensor::Sensor *supply_fan_rpm_sensor_{nullptr};
  sensor::Sensor *exhaust_fan_rpm_sensor_{nullptr};
  sensor::Sensor *supply_fan_cfm_sensor_{nullptr};
  sensor::Sensor *exhaust_fan_cfm_sensor_{nullptr};
  sensor::Sensor *power_sensor_{nullptr};

  FlowControlState flow_state_{FlowControlState::IDLE};
  std::queue<PendingMessage> message_queue_;
  uint32_t last_poll_time_{0};
  uint32_t last_tx_time_{0};
  uint32_t last_heartbeat_time_{0};
  uint32_t heartbeat_interval_{DEFAULT_HEARTBEAT_INTERVAL};

  void process_byte(uint8_t byte);
  void try_parse_frames();
  bool try_parse_frame(const uint8_t* data, size_t len, ParsedFrame& frame);
  uint8_t calc_checksum(const uint8_t* data, size_t payload_len);
  void log_frame(uint8_t src_addr, uint8_t dst_addr, const std::vector<uint8_t>& payload);
  std::string get_message_type(uint8_t type_byte);

  std::string get_device_name(uint8_t addr);
  std::string get_register_name(uint16_t reg);
  std::string get_mode_name(uint8_t mode);

  void log_register_read_request(const std::vector<uint8_t>& payload);
  void log_register_read_response(const std::vector<uint8_t>& payload);
  void log_register_write_request(const std::vector<uint8_t>& payload);
  void log_register_write_response(const std::vector<uint8_t>& payload);
  void log_register_pair_list(const std::vector<uint8_t>& payload);
  void log_register_value_blocks(const std::vector<uint8_t>& payload);
  void log_ping_pong(const std::vector<uint8_t>& payload);

  void log_register_value(uint16_t reg, const uint8_t* data, size_t len);

  void process_read_response(const std::vector<uint8_t>& payload);
  void update_sensor_from_register(uint16_t reg, const uint8_t* data, size_t len);

  void handle_received_frame(const ParsedFrame& frame);
  void handle_flow_control(uint8_t flow_byte, uint8_t src_addr, uint8_t dst_addr);
  void handle_ping(const std::vector<uint8_t>& payload);

  void process_flow_control_state();
  void send_frame(const std::vector<uint8_t>& payload);
  void build_and_send_frame(uint8_t dst, uint8_t src, const std::vector<uint8_t>& payload);
  void send_flow_control(uint8_t flow_byte);
  void send_pong();
  void send_read_request(const std::vector<uint16_t>& registers);
  void send_heartbeat();

  void poll_registers();
  void send_write_mode(uint8_t mode_hex);
};

}  // namespace broan_erv
}  // namespace esphome
