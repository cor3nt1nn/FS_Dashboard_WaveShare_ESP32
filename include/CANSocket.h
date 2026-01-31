#ifndef __CAN_SOCKET_H_
#define __CAN_SOCKET_H_

#include "driver/twai.h"
#include <Arduino.h>
#include "CarState.hpp"

// CANSocket class handles CAN communication and maintains car state
typedef std::function<void(const twai_message_t &msg, CarState &state)>
    FrameHandler;

class CANSocket {
public:
  CANSocket(gpio_num_t tx = GPIO_NUM_15, gpio_num_t rx = GPIO_NUM_16,
            twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS());

  bool begin();
  void loop();                // call in main loop
  void onFrame(FrameHandler); // set the frame handler callback to handle incoming CAN frames
  CarState &state();

private:
  gpio_num_t _tx, _rx;
  twai_timing_config_t _timing;
  FrameHandler _handler;
  CarState _state;
};

#endif
