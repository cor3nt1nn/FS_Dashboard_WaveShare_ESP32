#ifndef CANLOGGER_HPP
#define CANLOGGER_HPP

#include <SD_MMC.h>
#include "driver/twai.h"
#include <atomic>

#define LOG_QUEUE_SIZE 500 // Maximum number of CAN frames that can be queued before writing to SD card
#define FLUSH_INTERVAL_MS 2000 // Interval in milliseconds between two SD card flushes (2 seconds)
#define FILE_ROTATION_MS 420000 // Time before creating a new log file (7 minutes)
#define MAX_BATCH_SIZE 50 // Maximum number of entries to process in one batch operation

// Structure representing a single CAN message to be logged
struct CANLogEntry {
    uint32_t timestamp;    // Time when message was received (in milliseconds)
    uint32_t identifier;   // CAN message identifier (11-bit or 29-bit)
    uint8_t dlc;          // Data Length Code - number of data bytes (0-8)
    uint8_t data[8];      // Actual CAN message payload (up to 8 bytes)
    bool extd;            // True if extended frame (29-bit ID), false for standard (11-bit ID)
};

class CANLogger {
public:
    CANLogger();
    ~CANLogger();

    // Initialize the logger (SD card, queue, mutex)
    // Returns true if successful, false otherwise
    bool begin();

    void loop(); // Main thread loop

    // Add a CAN frame to the logging queue (non-blocking)
    // Returns true if successfully queued, false if queue is full
    bool logFrame(const twai_message_t &msg);

    // Start/stop the logging process
    void startLogging();
    void stopLogging();

    // Getters for logger status and statistics
    bool isLogging() const { return logging; }  // Check if currently logging
    uint32_t getFramesLogged() const { return framesLogged.load(); }  // Total frames successfully logged
    uint32_t getFramesDropped() const { return framesDropped.load(); }  // Total frames dropped (queue full)

    // Calculate the percentage of dropped frames
    float getDropRate() const {
        uint32_t total = framesLogged.load() + framesDropped.load();
        return total > 0 ? (100.0f * framesDropped.load() / total) : 0.0f;
    }

private:
    void createNewLogFile(); // Create a new log file with timestamp-based filename
    void writeHeader(); // Write CSV or binary header to the log file
    void writeEntry(const CANLogEntry &entry); // Write a single CAN entry to the file
    void flushAndRotate(); // Flush buffered data to SD card and rotate file if needed
    void processQueue(); // Process entries from the queue and write them to file

    // State flags
    bool logging;          // True when actively logging
    bool sdInitialized;    // True when SD card is successfully initialized

    // File handling
    File logFile;                  // Current open log file handle
    char currentFilename[32];      // Name of the current log file

    // Thread-safe queue and synchronization
    QueueHandle_t logQueue;        // Queue for buffering CAN messages
    SemaphoreHandle_t sdMutex;     // Mutex to protect SD card access from multiple tasks

    // Statistics (atomic for thread safety)
    std::atomic<uint32_t> framesLogged;   // Count of successfully logged frames
    std::atomic<uint32_t> framesDropped;  // Count of dropped frames (queue overflow)

    // Timing variables
    uint32_t fileStartTime;   // Timestamp when current file was created (for rotation)
    uint32_t lastFlushMs;     // Timestamp of last flush operation
};

#endif
