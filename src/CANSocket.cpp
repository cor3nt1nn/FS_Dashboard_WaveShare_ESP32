#include "CANSocket.h"
#include "Debug.hpp"

// CANSocket class initialization : 20km race, battery of 10.2kWh capacity by default
CANSocket::CANSocket(gpio_num_t tx, gpio_num_t rx, twai_timing_config_t timing)
    : _tx(tx), _rx(rx), _timing(timing), _state(20.0f, 10.2f) {}

// Initialize the CAN socket and start the TWAI driver
bool CANSocket::begin() {
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(_tx, _rx, TWAI_MODE_NORMAL);

  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  LOG_I(TAG_CAN, "Installing TWAI driver (TX=GPIO%d, RX=GPIO%d)...", _tx, _rx);
  if (twai_driver_install(&g_config, &_timing, &f_config) != ESP_OK) {
    LOG_E(TAG_CAN, "TWAI driver install FAILED");
    return false;
  }
  LOG_I(TAG_CAN, "TWAI driver installed OK");

  LOG_I(TAG_CAN, "Starting TWAI driver...");
  if (twai_start() != ESP_OK) {
    LOG_E(TAG_CAN, "TWAI driver start FAILED");
    return false;
  }
  LOG_I(TAG_CAN, "TWAI driver started OK - CAN bus ready");
  return true;
}

// set the frame handler callback
void CANSocket::onFrame(FrameHandler handler) { _handler = handler; }

// get the current car state
CarState &CANSocket::state() { return _state; }

// main loop to receive CAN messages and process them
void CANSocket::loop() {
  twai_message_t msg;
  esp_err_t result = twai_receive(&msg, pdMS_TO_TICKS(10));
  if (result == ESP_OK) {
    char dataStr[32] = {0};
    int offset = 0;
    for (int i = 0; i < msg.data_length_code && i < 8; i++) {
      offset += snprintf(dataStr + offset, sizeof(dataStr) - offset, "%02X ", msg.data[i]);
    }
    LOG_I(TAG_CAN, "RX ID=0x%08X  Ext=%d  DLC=%u  Data=[%s]",
          msg.identifier, msg.extd ? 1 : 0, msg.data_length_code, dataStr);

    if (_handler) {
      _handler(msg, _state);
    } else {
      LOG_W(TAG_CAN, "Frame received but no handler registered! ID=0x%08X", msg.identifier);
    }
  } else if (result == ESP_ERR_TIMEOUT) {
    LOG_EVERY_MS(5000, LOG_W(TAG_CAN, "No CAN frame received in the last 5s - bus silent?"));
  } else {
    LOG_E(TAG_CAN, "twai_receive error: 0x%X", result);
  }
}
