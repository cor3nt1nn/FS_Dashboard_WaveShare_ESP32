#include "CANLogger.hpp"
#include "Debug.hpp"

CANLogger::CANLogger()
    : logging(false),           // Start with logging disabled
      sdInitialized(false),     // SD card not yet initialized
      framesLogged(0),          // No frames logged yet
      framesDropped(0),         // No frames dropped yet
      fileStartTime(0),         // No file created yet
      lastFlushMs(0) {          // No flush performed yet
    logQueue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(CANLogEntry)); // Queue to buffer CAN messages
    sdMutex = xSemaphoreCreateMutex(); // Mutex to protect SD card access from concurrent tasks
}

CANLogger::~CANLogger() {
    stopLogging();  // Stop logging and flush remaining data

    if (sdInitialized) { // Deinitialize SD card if it was initialized
        SD_MMC.end();
    }
    if (logQueue) {  // Delete FreeRTOS queue
        vQueueDelete(logQueue);
    }
    if (sdMutex) { // Delete mutex
        vSemaphoreDelete(sdMutex);
    }
}

// Initialize SD card with specific pins
bool CANLogger::begin() {
    LOG_I(TAG_LOGGER, "Mounting SD card (CLK=12, CMD=11, DATA0=13)...");
    SD_MMC.setPins(12, 11, 13); // CLK=12, CMD=11, DATA0=13
    delay(500);

    if(!SD_MMC.begin("/sdcard", true)){ // Mount point is "/sdcard"
        LOG_E(TAG_LOGGER, "SD_MMC.begin() FAILED - SD card not mounted");
        SD_MMC.end();  // Clean up on failure
        sdInitialized = false;
        return false;  // Initialization failed
    }

    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    LOG_I(TAG_LOGGER, "SD card mounted OK - size: %llu MB", cardSize);
    sdInitialized = true;
    return true;  // Initialization successful
}

// Main loop function that will be called in the thread
void CANLogger::loop() {
    if (!logging) { // If not logging, just wait and return
        vTaskDelay(pdMS_TO_TICKS(100));  // Sleep for 100ms
        return;
    }
    // Log queue stats periodically
    LOG_EVERY_MS(10000, LOG_I(TAG_LOGGER,
        "Stats - logged=%lu  dropped=%lu  dropRate=%.1f%%  queueWaiting=%u",
        framesLogged.load(), framesDropped.load(), getDropRate(),
        (unsigned)uxQueueMessagesWaiting(logQueue)));
    // Process queued CAN frames
    processQueue();
}

// Add a CAN frame to the logging queue (non-blocking)
bool CANLogger::logFrame(const twai_message_t &msg) {
    // Don't log if logging is disabled
    if (!logging) {
        return false;
    }

    // Create log entry from CAN message
    CANLogEntry entry;
    entry.timestamp = millis();           // Current time in milliseconds
    entry.identifier = msg.identifier;    // CAN ID
    entry.dlc = msg.data_length_code;     // Number of data bytes
    entry.extd = msg.extd;                // Extended frame flag

    // Copy data bytes (up to 8 bytes)
    for (int i = 0; i < entry.dlc && i < 8; i++) {
        entry.data[i] = msg.data[i];
    }

    // Try to add to queue without blocking (timeout = 0)
    if (xQueueSend(logQueue, &entry, 0) != pdTRUE) {
        framesDropped++;  // Queue is full, increment dropped counter
        LOG_W(TAG_LOGGER, "Queue FULL - frame dropped! total dropped=%lu  queueSize=%d",
              framesDropped.load(), LOG_QUEUE_SIZE);
        return false;     // Failed to queue
    }

    return true;  // Successfully queued
}

// Start the logging process
void CANLogger::startLogging() {
    // Don't start if SD card not initialized or already logging
    if (!sdInitialized) {
        LOG_E(TAG_LOGGER, "startLogging() called but SD card not initialized");
        return;
    }
    if (logging) {
        LOG_W(TAG_LOGGER, "startLogging() called but already logging");
        return;
    }

    // Create the first log file
    createNewLogFile();

    // Enable logging
    logging = true;
    LOG_I(TAG_LOGGER, "Logging started - file: %s", currentFilename);
}

// Stop logging and flush all remaining data
void CANLogger::stopLogging() {
    // Already stopped
    if (!logging) {
        LOG_W(TAG_LOGGER, "stopLogging() called but not currently logging");
        return;
    }

    LOG_I(TAG_LOGGER, "Stopping logging - flushing remaining frames...");

    // Disable logging flag
    logging = false;

    // Wait for queue to empty (with timeout to prevent infinite loop)
    int timeout = 0;
    while (uxQueueMessagesWaiting(logQueue) > 0 && timeout < 100) {
        vTaskDelay(pdMS_TO_TICKS(50));  // Wait 50ms
        timeout++;
    }

    if (timeout >= 100) {
        LOG_W(TAG_LOGGER, "Queue drain timed out - %u frames may be lost",
              (unsigned)uxQueueMessagesWaiting(logQueue));
    }

    // Acquire mutex to safely access SD card
    if (xSemaphoreTake(sdMutex, portMAX_DELAY)) {
        // Close log file if open
        if (logFile) {
            logFile.flush();  // Write any buffered data
            logFile.close();  // Close the file
            LOG_I(TAG_LOGGER, "Log file closed - total logged=%lu  dropped=%lu",
                  framesLogged.load(), framesDropped.load());
        }
        xSemaphoreGive(sdMutex);  // Release mutex
    }
}

// Create a new log file with timestamp-based name
void CANLogger::createNewLogFile() {
    // Try to acquire mutex (wait forever if necessary)
    if (!xSemaphoreTake(sdMutex, portMAX_DELAY)) {
        LOG_E(TAG_LOGGER, "createNewLogFile() - failed to acquire mutex");
        return;
    }

    // Close previous log file if open
    if (logFile) {
        LOG_I(TAG_LOGGER, "Closing previous log file: %s", currentFilename);
        logFile.flush();
        logFile.close();
    }

    // Record when this file was created (for rotation)
    fileStartTime = millis();

    // Generate filename based on timestamp: /canlog_12345678.csv
    snprintf(currentFilename, sizeof(currentFilename),
             "/canlog_%lu.csv", fileStartTime);

    LOG_I(TAG_LOGGER, "Creating new log file: %s", currentFilename);

    // Open file for writing (creates if doesn't exist)
    logFile = SD_MMC.open(currentFilename, FILE_WRITE);

    // Write CSV header if file opened successfully
    if (logFile) {
        writeHeader();
        LOG_I(TAG_LOGGER, "Log file created OK: %s", currentFilename);
    } else {
        LOG_E(TAG_LOGGER, "Failed to open log file: %s", currentFilename);
    }

    xSemaphoreGive(sdMutex);  // Release mutex
}

// Write CSV header line to the log file
void CANLogger::writeHeader() {
    logFile.println("Timestamp,ID,Ext,DLC,Data");
}

// Write a single CAN entry to the log file in CSV format
void CANLogger::writeEntry(const CANLogEntry &entry) {
    // No file open
    if (!logFile) {
        return;
    }

    // Write: timestamp, ID (hex), extended flag, DLC
    logFile.printf("%lu,0x%08X,%d,%d",
                   entry.timestamp,
                   entry.identifier,
                   entry.extd ? 1 : 0,  // 1 for extended, 0 for standard
                   entry.dlc);

    // Write data bytes in hex format
    for (int i = 0; i < entry.dlc && i < 8; i++) {
        logFile.printf(",%02X", entry.data[i]);  // Two-digit hex
    }

    logFile.println();  // End of line

    framesLogged++;  // Increment logged frames counter
}

// Flush buffered data to SD card and rotate file if needed
void CANLogger::flushAndRotate() {
    if (!logFile) {
        LOG_W(TAG_LOGGER, "flushAndRotate() called but no file is open");
        return;
    }

    // Force write buffered data to SD card
    logFile.flush();
    LOG_EVERY_MS(30000, LOG_I(TAG_LOGGER, "Flushed to SD - logged=%lu  dropped=%lu  file=%s",
        framesLogged.load(), framesDropped.load(), currentFilename));

    // Check if file rotation is needed (file age > FILE_ROTATION_MS)
    uint32_t fileAge = millis() - fileStartTime;
    if (fileAge > FILE_ROTATION_MS) {
        LOG_I(TAG_LOGGER, "File rotation triggered (age=%lu ms > %d ms) - creating new file",
              fileAge, FILE_ROTATION_MS);
        createNewLogFile();  // Start a new file
    }
}

// Process queued CAN messages and write them to file
void CANLogger::processQueue() {
    CANLogEntry entry;
    int processed = 0;  // Count of entries processed in this batch

    // Process up to MAX_BATCH_SIZE entries from the queue
    while (processed < MAX_BATCH_SIZE &&
           xQueueReceive(logQueue, &entry, 0) == pdTRUE) {  // Non-blocking receive

        // Try to acquire mutex with 10ms timeout
        if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(10))) {
            writeEntry(entry);  // Write entry to file
            xSemaphoreGive(sdMutex);  // Release mutex
            processed++;
        } else {
            // Couldn't get mutex, put entry back at front of queue
            LOG_W(TAG_LOGGER, "processQueue() - mutex timeout, requeueing entry 0x%08X", entry.identifier);
            xQueueSendToFront(logQueue, &entry, 0);
            break;  // Stop processing this batch
        }
    }

    if (processed > 0) {
        LOG_EVERY_MS(5000, LOG_I(TAG_LOGGER, "Batch processed %d frames  queueRemaining=%u",
            processed, (unsigned)uxQueueMessagesWaiting(logQueue)));
    }

    // Check if it's time to flush (every FLUSH_INTERVAL_MS = 2 seconds)
    if (millis() - lastFlushMs > FLUSH_INTERVAL_MS) {
        // Try to acquire mutex with 10ms timeout
        if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(10))) {
            flushAndRotate();  // Flush data and rotate file if needed
            xSemaphoreGive(sdMutex);
        } else {
            LOG_W(TAG_LOGGER, "processQueue() - mutex timeout during flush");
        }
        lastFlushMs = millis();  // Update last flush time
    }

    // Yield to other tasks
    if (processed > 0) {
        vTaskDelay(pdMS_TO_TICKS(1));  // Short delay if we processed data
    } else {
        vTaskDelay(pdMS_TO_TICKS(5));  // Longer delay if queue was empty
    }
}
