#ifndef CAN_LOGGER_H
#define CAN_LOGGER_H

#include <SD.h>
#include <SPI.h>
#include "driver/twai.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <Arduino.h>
#include <atomic>

// pins of SD card interface
#define SD_CS_PIN    10
#define SD_MOSI_PIN  11
#define SD_MISO_PIN  13
#define SD_SCK_PIN   12

// Size of the queue to hold CAN log entries before writing to SD card
#define LOG_QUEUE_SIZE 500

// Structure to hold a CAN log entry
struct CANLogEntry {
    uint32_t timestamp;
    uint32_t identifier;
    uint8_t data[8];
    uint8_t dlc;
    bool extd;
};

class CANLogger {
public:
    CANLogger();
    ~CANLogger();

    bool begin();
    void loop(); // main loop to process log queue and write to SD card

    bool logFrame(const twai_message_t &msg); // add a CAN frame to the log queue

    void startLogging(); // start logging CAN frames to SD card
    void stopLogging(); // stop logging CAN frames to SD card
    bool isLogging() const { return logging; }

    uint32_t getFramesLogged() const { return framesLogged; } // get the number of frames successfully logged
    uint32_t getFramesDropped() const { return framesDropped; } // get the number of frames dropped due to queue full

private:
    QueueHandle_t logQueue;
    SemaphoreHandle_t sdMutex;
    File logFile;

    bool logging;
    bool sdInitialized;
    std::atomic<uint32_t> framesLogged;
    std::atomic<uint32_t> framesDropped;

    char currentFilename[32];
    uint32_t fileStartTime;
    uint32_t lastFlushMs;

    void createNewLogFile(); // create a new log file on the SD card
    void writeHeader(); // write the header to the current log file
    void writeEntry(const CANLogEntry &entry); // write a CAN log entry to the current log file
    void flushAndRotate(); // flush the current log file and rotate to a new one if needed
    void processQueue(); // process the log queue and write entries to the SD card
};

#endif
