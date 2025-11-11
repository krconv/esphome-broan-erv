#include "broan_erv_mode.h"
#include "esphome/core/log.h"

namespace esphome {
namespace broan_erv {

static const char *TAG = "broan_erv.select";

void BroanERVModeSelect::control(const std::string &value) {
  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "Parent component not set for fan mode select");
    return;
  }

  this->parent_->set_mode(value);
}

}  // namespace broan_erv
}  // namespace esphome
