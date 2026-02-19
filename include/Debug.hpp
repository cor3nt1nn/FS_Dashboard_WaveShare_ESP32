#ifndef DEBUG_HPP
#define DEBUG_HPP

#include <Arduino.h>

#define DEBUG_ENABLED

#define LOG_LEVEL_INFO
#define LOG_LEVEL_WARN
#define LOG_LEVEL_ERROR

#define TAG_MAIN    "[MAIN   ]"
#define TAG_CAN     "[CAN    ]"
#define TAG_LOGGER  "[LOGGER ]"
#define TAG_STATE   "[STATE  ]"
#define TAG_UI      "[UI     ]"
#define TAG_CALC    "[CALC   ]"

#ifdef DEBUG_ENABLED

  #ifdef LOG_LEVEL_INFO
    #define LOG_I(tag, fmt, ...) \
      Serial0.printf("[%8lu] %s [INFO ] " fmt "\n", millis(), tag, ##__VA_ARGS__)
  #else
    #define LOG_I(tag, fmt, ...)
  #endif

  #ifdef LOG_LEVEL_WARN
    #define LOG_W(tag, fmt, ...) \
      Serial0.printf("[%8lu] %s [WARN ] " fmt "\n", millis(), tag, ##__VA_ARGS__)
  #else
    #define LOG_W(tag, fmt, ...)
  #endif

  #ifdef LOG_LEVEL_ERROR
    #define LOG_E(tag, fmt, ...) \
      Serial0.printf("[%8lu] %s [ERROR] " fmt "\n", millis(), tag, ##__VA_ARGS__)
  #else
    #define LOG_E(tag, fmt, ...)
  #endif

#else
  #define LOG_I(tag, fmt, ...)
  #define LOG_W(tag, fmt, ...)
  #define LOG_E(tag, fmt, ...)
#endif

#define LOG_EVERY_N(n, logCall)         \
  do {                                  \
    static uint32_t _log_counter = 0;  \
    if ((_log_counter++ % (n)) == 0) { \
      logCall;                          \
    }                                   \
  } while (0)

#define LOG_EVERY_MS(interval_ms, logCall)          \
  do {                                              \
    static uint32_t _log_last_ms = 0;              \
    uint32_t _log_now = millis();                  \
    if (_log_now - _log_last_ms >= (interval_ms)) { \
      _log_last_ms = _log_now;                     \
      logCall;                                      \
    }                                               \
  } while (0)

#endif // DEBUG_HPP
