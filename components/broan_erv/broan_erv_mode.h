/**
 * Select entity for controlling Broan ERV fan mode
 *
 * This file implements the ESPHome Select entity for controlling the mode of
 * the Broan ERV system. It provides a dropdown interface in Home Assistant
 * with available fan modes.
 */

#pragma once

#include "esphome/components/select/select.h"
#include "broan_erv.h"

namespace esphome {
namespace broan_erv {

/**
 * Select entity for Broan ERV fan mode control
 *
 * This class implements an ESPHome Select entity that allows users to control
 * the ERV fan mode through Home Assistant. Available modes include:
 * - Off: Fan is off
 * - Minimum: Lowest fan speed
 * - Medium: Medium fan speed
 * - Maximum: Highest fan speed
 * - Auto: Automatic mode based on sensor readings
 */
class BroanERVModeSelect : public select::Select, public Component {
 public:
  void set_parent(BroanERVComponent *parent) { parent_ = parent; }

 protected:
  void control(const std::string &value) override;

  BroanERVComponent *parent_{nullptr};
};

}  // namespace broan_erv
}  // namespace esphome
