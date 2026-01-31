#include "CANSocket.h"

// CANSocket class initialization : 20km race, battery of 10.2kWh capacity by default
CANSocket::CANSocket(gpio_num_t tx, gpio_num_t rx, twai_timing_config_t timing)
    : _tx(tx), _rx(rx), _timing(timing), _state(20.0f, 10.2f) {}

// Initialize the CAN socket and start the TWAI driver
bool CANSocket::begin() {
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(_tx, _rx, TWAI_MODE_NORMAL);

  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &_timing, &f_config) != ESP_OK)
    return false;

  return twai_start() == ESP_OK;
}

// set the frame handler callback
void CANSocket::onFrame(FrameHandler handler) { _handler = handler; }

// get the current car state
CarState &CANSocket::state() { return _state; }

// main loop to receive CAN messages and process them
void CANSocket::loop() {
  twai_message_t msg;
  if (twai_receive(&msg, pdMS_TO_TICKS(10)) == ESP_OK) {
    if (_handler) {
      _handler(msg, _state);
    }
  }
}
