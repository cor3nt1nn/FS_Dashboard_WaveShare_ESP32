#ifndef CANLOGGER_HPP
#define CANLOGGER_HPP

#include <SD_MMC.h>
#include "driver/twai.h"
#include <atomic>

#define LOG_QUEUE_SIZE 500
#define FLUSH_INTERVAL_MS 2000
#define FILE_ROTATION_MS 600000
#define MAX_BATCH_SIZE 50

struct CANLogEntry {
    uint32_t timestamp;
    uint32_t identifier;
    uint8_t dlc;
    uint8_t data[8];
    bool extd;
};

class CANLogger {
public:
    CANLogger();
    ~CANLogger();

    bool begin();
    void loop();

    bool logFrame(const twai_message_t &msg);
    void startLogging();
    void stopLogging();

    bool isLogging() const { return logging; }
    uint32_t getFramesLogged() const { return framesLogged.load(); }
    uint32_t getFramesDropped() const { return framesDropped.load(); }
    float getDropRate() const {
        uint32_t total = framesLogged.load() + framesDropped.load();
        return total > 0 ? (100.0f * framesDropped.load() / total) : 0.0f;
    }

private:
    void createNewLogFile();
    void writeHeader();
    void writeEntry(const CANLogEntry &entry);
    void flushAndRotate();
    void processQueue();

    bool logging;
    bool sdInitialized;

    File logFile;
    char currentFilename[32];

    QueueHandle_t logQueue;
    SemaphoreHandle_t sdMutex;

    std::atomic<uint32_t> framesLogged;
    std::atomic<uint32_t> framesDropped;

    uint32_t fileStartTime;
    uint32_t lastFlushMs;
};

#endif
