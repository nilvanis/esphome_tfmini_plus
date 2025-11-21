#include "tfmini_plus.h"

#include <array>
#include <cmath>
#include <cstring>

#include "esphome/core/hal.h"

namespace esphome {
namespace tfmini_plus {

namespace {
static const char *const TAG = "tfmini_plus";

// Device always streams at 100 Hz by default. Allow a short window to find a frame.
static const uint32_t READ_TIMEOUT_MS = 150;
static const uint32_t COMMAND_TIMEOUT_MS = 1000;
static const uint32_t OFFLINE_RETRY_INTERVAL_MS = 60000;

std::string status_to_string(StatusCode status) {
  switch (status) {
    case StatusCode::READY:
      return "READY";
    case StatusCode::SERIAL_ERR:
      return "SERIAL";
    case StatusCode::HEADER:
      return "HEADER";
    case StatusCode::CHECKSUM:
      return "CHECKSUM";
    case StatusCode::TIMEOUT:
      return "TIMEOUT";
    case StatusCode::PASS:
      return "PASS";
    case StatusCode::FAIL:
      return "FAIL";
    case StatusCode::I2C_READ:
      return "I2CREAD";
    case StatusCode::I2C_WRITE:
      return "I2CWRITE";
    case StatusCode::I2C_LENGTH:
      return "I2CLENGTH";
    case StatusCode::WEAK:
      return "WEAK";
    case StatusCode::STRONG:
      return "STRONG";
    case StatusCode::FLOOD:
      return "FLOOD";
    case StatusCode::MEASURE:
      return "MEASURE";
    case StatusCode::OFFLINE:
      return "OFFLINE";
    case StatusCode::SLEEPING:
      return "SLEEPING";
    case StatusCode::OTHER:
    default:
      return "OTHER";
  }
}
}  // namespace

void TFMiniPlusComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TFmini Plus...");
  this->check_uart_settings(115200);

#if defined(USE_API) && defined(USE_API_CUSTOM_SERVICES)
  this->register_service(&TFMiniPlusComponent::sleep_service, "tfmini_plus_sleep");
  this->register_service(&TFMiniPlusComponent::wake_service, "tfmini_plus_wake");
#elif defined(USE_API)
  ESP_LOGW(TAG, "API custom services are not enabled; sleep/wake services not registered");
#endif

  this->flush_input_();

  if (this->soft_reset_) {
    if (!this->send_command_(SOFT_RESET, 0)) {
      ESP_LOGW(TAG, "Soft reset command failed");
    } else {
      delay(50);  // Give the sensor a short moment to reboot
    }
  }

  this->apply_frame_rate_(this->frame_rate_);

  this->mark_offline_("Waiting for first frame");
  this->last_good_frame_ms_ = 0;
  const uint32_t now = millis();
  this->error_window_start_ms_ = now;
  this->last_error_log_ms_ = now;
}

void TFMiniPlusComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "TFmini Plus LiDAR:");
  ESP_LOGCONFIG(TAG, "  Frame rate: %u", this->frame_rate_);
  ESP_LOGCONFIG(TAG, "  Soft reset at boot: %s", YESNO(this->soft_reset_));
  ESP_LOGCONFIG(TAG, "  Save settings on config change: %s", YESNO(this->save_settings_));

  LOG_SENSOR("  ", "Distance", this->distance_sensor_);
  LOG_SENSOR("  ", "Signal Strength", this->signal_strength_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
#ifdef USE_TFMINI_PLUS_STATUS_SENSOR
  LOG_TEXT_SENSOR("  ", "Status", this->status_sensor_);
#endif

  if (this->state_ == DeviceState::SLEEPING) {
    ESP_LOGCONFIG(TAG, "  Current state: sleeping");
  } else if (this->state_ == DeviceState::OFFLINE) {
    ESP_LOGCONFIG(TAG, "  Current state: offline");
  }
}

void TFMiniPlusComponent::update() {
  const uint32_t now = millis();

  if (this->state_ == DeviceState::SLEEPING) {
    this->set_status_(StatusCode::SLEEPING);
    this->publish_unavailable_();
    return;
  }

  if (this->state_ == DeviceState::OFFLINE) {
    // Immediately after wake, keep retrying every cycle for a short grace period.
    if (this->wake_grace_until_ == 0 || now > this->wake_grace_until_) {
      if (this->last_retry_ms_ != 0 && (now - this->last_retry_ms_) < OFFLINE_RETRY_INTERVAL_MS) {
        return;
      }
    }
  }

  this->last_retry_ms_ = now;
  FrameData data{};

  bool got_frame = this->read_frame_(data);

  if (got_frame) {
    // First data after being offline
    if (this->state_ != DeviceState::ONLINE) {
      ESP_LOGI(TAG, "TFmini Plus came online");
      this->state_ = DeviceState::ONLINE;
      this->published_unavailable_ = false;
      this->set_status_(StatusCode::READY);
    }
    this->last_good_frame_ms_ = now;
    this->publish_online_(data);
  }

  // If no good frame for more than 1s, declare offline.
  const bool in_wake_grace = (this->wake_grace_until_ != 0 && now <= this->wake_grace_until_);
  const uint32_t offline_timeout_ms = in_wake_grace ? 5000 : 1000;
  if (this->last_good_frame_ms_ == 0 || (now - this->last_good_frame_ms_) > offline_timeout_ms) {
    if (this->state_ != DeviceState::OFFLINE) {
      this->mark_offline_("No valid frame within 1s", this->last_status_);
      // After declaring offline due to missing frames, retry scale doubles as needed.
      this->last_retry_ms_ = now;
    }
  }
}

bool TFMiniPlusComponent::read_frame_(FrameData &data) {
  std::array<uint8_t, TFMP_FRAME_SIZE + 1> frame{};
  const uint32_t deadline = millis() + READ_TIMEOUT_MS;

  while (millis() < deadline) {
    if (this->available() <= 0) {
      delay(1);
      continue;
    }

    uint8_t byte;
    if (!this->read_byte(&byte))
      continue;

    frame[TFMP_FRAME_SIZE] = byte;
    memmove(frame.data(), frame.data() + 1, TFMP_FRAME_SIZE);

    if (frame[0] != 0x59 || frame[1] != 0x59)
      continue;

    uint16_t checksum = 0;
    for (size_t i = 0; i < (TFMP_FRAME_SIZE - 1); i++)
      checksum += frame[i];

  if (static_cast<uint8_t>(checksum) != frame[TFMP_FRAME_SIZE - 1]) {
    ESP_LOGW(TAG, "Checksum error while reading frame");
    this->set_status_(StatusCode::CHECKSUM);
    this->record_error_(StatusCode::CHECKSUM, millis());
    data.status = MeasurementStatus::CHECKSUM;
    return false;
  }

    data.distance_cm = static_cast<int16_t>(frame[2] + (static_cast<uint16_t>(frame[3]) << 8));
    data.strength = static_cast<int16_t>(frame[4] + (static_cast<uint16_t>(frame[5]) << 8));
    const int16_t raw_temp = static_cast<int16_t>(frame[6] + (static_cast<uint16_t>(frame[7]) << 8));
    data.temperature_c = (raw_temp >> 3) - 256;
    data.status = MeasurementStatus::OK;

    if (data.distance_cm == -1) {
      data.status = MeasurementStatus::WEAK_SIGNAL;
    } else if (data.strength == -1) {
      data.status = MeasurementStatus::STRONG_SIGNAL;
    } else if (data.distance_cm == -4) {
      data.status = MeasurementStatus::FLOOD_LIGHT;
    }

    return true;
  }

  ESP_LOGW(TAG, "Timeout waiting for TFmini Plus frame");
  this->set_status_(StatusCode::TIMEOUT);
  this->record_error_(StatusCode::TIMEOUT, millis());
  data.status = MeasurementStatus::HEADER;
  return false;
}

bool TFMiniPlusComponent::apply_frame_rate_(uint16_t frame_rate) {
  if (!this->send_command_(SET_FRAME_RATE, frame_rate == 0 ? FRAME_0 : frame_rate)) {
    ESP_LOGW(TAG, "Failed to set frame rate to %u", frame_rate);
    return false;
  }

  if (this->save_settings_) {
    this->send_command_(SAVE_SETTINGS, 0);
  }
  return true;
}

bool TFMiniPlusComponent::send_command_(uint32_t command, uint32_t param) {
  std::array<uint8_t, TFMP_COMMAND_MAX> cmnd_data{};
  memcpy(cmnd_data.data(), &command, 4);

  const uint8_t reply_len = cmnd_data[0];
  const uint8_t cmnd_len = cmnd_data[1];
  cmnd_data[0] = 0x5A;

  if (command == SET_FRAME_RATE) {
    memcpy(&cmnd_data[3], &param, 2);
  } else if (command == SET_BAUD_RATE) {
    memcpy(&cmnd_data[3], &param, 4);
  }

  uint16_t checksum = 0;
  for (uint8_t i = 0; i < (cmnd_len - 1); i++)
    checksum += cmnd_data[i];
  cmnd_data[cmnd_len - 1] = static_cast<uint8_t>(checksum);

  this->flush_input_();
  this->write_array(cmnd_data.data(), cmnd_len);

  if (reply_len == 0) {
    this->set_status_(StatusCode::PASS);
    return true;
  }

  const uint32_t deadline = millis() + COMMAND_TIMEOUT_MS;
  std::array<uint8_t, TFMP_REPLY_SIZE + 1> reply{};

  while (millis() < deadline) {
    if (this->available() <= 0) {
      delay(1);
      continue;
    }

    uint8_t byte;
    if (!this->read_byte(&byte))
      continue;

    reply[reply_len] = byte;
    memmove(reply.data(), reply.data() + 1, TFMP_REPLY_SIZE);

    if (reply[0] != 0x5A || reply[1] != reply_len)
      continue;

    uint16_t reply_checksum = 0;
    for (uint8_t i = 0; i < (reply_len - 1); i++)
      reply_checksum += reply[i];

    if (static_cast<uint8_t>(reply_checksum) != reply[reply_len - 1]) {
      ESP_LOGW(TAG, "Checksum error receiving reply for command 0x%08X", command);
      this->set_status_(StatusCode::CHECKSUM);
      return false;
    }

    if ((command == SOFT_RESET || command == HARD_RESET || command == SAVE_SETTINGS) && reply[3] == 1) {
      ESP_LOGW(TAG, "TFmini Plus reported failure for command 0x%08X", command);
      this->set_status_(StatusCode::FAIL);
      return false;
    }

    this->set_status_(StatusCode::PASS);
    return true;
  }

  ESP_LOGW(TAG, "Timeout waiting for reply to command 0x%08X", command);
  this->set_status_(StatusCode::TIMEOUT);
  return false;
}

void TFMiniPlusComponent::publish_online_(const FrameData &data) {
  this->published_unavailable_ = false;

  // Short-circuit special measurement statuses but keep the device marked as online.
  if (data.status != MeasurementStatus::OK) {
    StatusCode status_code = StatusCode::OTHER;
    switch (data.status) {
      case MeasurementStatus::WEAK_SIGNAL:
        status_code = StatusCode::WEAK;
        break;
      case MeasurementStatus::STRONG_SIGNAL:
        status_code = StatusCode::STRONG;
        break;
      case MeasurementStatus::FLOOD_LIGHT:
        status_code = StatusCode::FLOOD;
        break;
      case MeasurementStatus::HEADER:
        status_code = StatusCode::HEADER;
        break;
      case MeasurementStatus::CHECKSUM:
        status_code = StatusCode::CHECKSUM;
        break;
      case MeasurementStatus::OK:
        status_code = StatusCode::READY;
        break;
    }
    ESP_LOGW(TAG, "Measurement flagged as %s", status_to_string(status_code).c_str());
    this->set_status_(status_code);
    this->publish_unavailable_();
    return;
  }

  if (this->distance_sensor_ != nullptr) {
    float distance_cm = static_cast<float>(data.distance_cm);
    if (!this->have_distance_ || fabsf(distance_cm - this->last_distance_) >= 0.1f) {
      this->distance_sensor_->publish_state(distance_cm);
      this->last_distance_ = distance_cm;
      this->have_distance_ = true;
    }
  }

  if (this->signal_strength_sensor_ != nullptr) {
    float strength = static_cast<float>(data.strength);
    if (!this->have_signal_ || fabsf(strength - this->last_signal_) >= 1.0f) {
      this->signal_strength_sensor_->publish_state(strength);
      this->last_signal_ = strength;
      this->have_signal_ = true;
    }
  }

  if (this->temperature_sensor_ != nullptr) {
    float temperature_c = static_cast<float>(data.temperature_c);
    if (!this->have_temperature_ || fabsf(temperature_c - this->last_temperature_) >= 0.05f) {
      this->temperature_sensor_->publish_state(temperature_c);
      this->last_temperature_ = temperature_c;
      this->have_temperature_ = true;
    }
  }

  this->set_status_(StatusCode::READY);
}

void TFMiniPlusComponent::publish_unavailable_() {
  if (this->published_unavailable_)
    return;
  this->published_unavailable_ = true;

  if (this->distance_sensor_ != nullptr) {
    this->distance_sensor_->publish_state(NAN);
  }
  if (this->signal_strength_sensor_ != nullptr) {
    this->signal_strength_sensor_->publish_state(NAN);
  }
  if (this->temperature_sensor_ != nullptr) {
    this->temperature_sensor_->publish_state(NAN);
  }
  this->have_distance_ = false;
  this->have_signal_ = false;
  this->have_temperature_ = false;
}

void TFMiniPlusComponent::set_status_(StatusCode status) {
  this->last_status_ = status;
#ifdef USE_TFMINI_PLUS_STATUS_SENSOR
  if (this->status_sensor_ != nullptr) {
    if (!this->has_published_status_ || this->last_published_status_ != status) {
      this->status_sensor_->publish_state(status_to_string(status));
      this->last_published_status_ = status;
      this->has_published_status_ = true;
    }
  }
#endif
}

void TFMiniPlusComponent::mark_offline_(const char *reason, StatusCode status) {
  if (this->state_ != DeviceState::OFFLINE) {
    ESP_LOGW(TAG, "TFmini Plus marked offline: %s", reason);
  }

  this->state_ = DeviceState::OFFLINE;
  this->set_status_(status);
  this->publish_unavailable_();
  // Allow immediate retries during wake grace.
  if (this->wake_grace_until_ != 0 && millis() <= this->wake_grace_until_) {
    this->last_retry_ms_ = 0;
  } else if (status == StatusCode::TIMEOUT) {
    // Retry sooner after a timeout by resetting the backoff.
    this->last_retry_ms_ = 0;
  }
}

void TFMiniPlusComponent::flush_input_() {
  while (this->available() > 0) {
    this->read();
  }
  this->flush();
}

#ifdef USE_API
void TFMiniPlusComponent::sleep_service() {
  ESP_LOGI(TAG, "Putting TFmini Plus into sleep (frame rate 0)");
  if (this->send_command_(SET_FRAME_RATE, FRAME_0)) {
    this->state_ = DeviceState::SLEEPING;
    this->published_unavailable_ = false;  // allow a single unavailable publish on transition
    this->set_status_(StatusCode::SLEEPING);
    this->publish_unavailable_();
  } else {
    ESP_LOGW(TAG, "Sleep command failed");
  }
}

void TFMiniPlusComponent::wake_service() {
  ESP_LOGI(TAG, "Waking TFmini Plus with frame rate %u", this->frame_rate_);
  if (this->apply_frame_rate_(this->frame_rate_)) {
    this->state_ = DeviceState::OFFLINE;
    this->published_unavailable_ = false;
    const uint32_t now = millis();
    this->last_retry_ms_ = 0;  // force immediate retry
    this->wake_grace_until_ = now + 5000;  // quick retry window after wake
    this->set_status_(StatusCode::READY);
  } else {
    ESP_LOGW(TAG, "Wake command failed");
  }
}
#endif

void TFMiniPlusComponent::record_error_(StatusCode status, uint32_t now) {
  if ((now - this->error_window_start_ms_) > 60000) {
    this->error_window_start_ms_ = now;
    this->error_count_window_ = 0;
  }
  this->error_count_window_++;
  if ((now - this->last_error_log_ms_) >= 60000) {
    ESP_LOGW(TAG, "TFmini Plus frame errors: %u in last minute (last status %s)", this->error_count_window_,
             status_to_string(status).c_str());
    this->last_error_log_ms_ = now;
  }
}

}  // namespace tfmini_plus
}  // namespace esphome
