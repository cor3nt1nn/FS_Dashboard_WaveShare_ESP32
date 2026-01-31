#include "CANLogger.hpp"

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
    SD_MMC.setPins(12, 11, 13); // CLK=12, CMD=11, DATA0=13
    delay(500);

    if(!SD_MMC.begin("/sdcard", true)){ // Mount point is "/sdcard"
        SD_MMC.end();  // Clean up on failure
        sdInitialized = false;
        return false;  // Initialization failed
    }

    sdInitialized = true;
    return true;  // Initialization successful
}

// Main loop function that will be called in the thread
void CANLogger::loop() {
    if (!logging) { // If not logging, just wait and return
        vTaskDelay(pdMS_TO_TICKS(100));  // Sleep for 100ms
        return;
    }
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
        return false;     // Failed to queue
    }

    return true;  // Successfully queued
}

// Start the logging process
void CANLogger::startLogging() {
    // Don't start if SD card not initialized or already logging
    if (!sdInitialized || logging) {
        return;
    }

    // Create the first log file
    createNewLogFile();

    // Enable logging
    logging = true;
}

// Stop logging and flush all remaining data
void CANLogger::stopLogging() {
    // Already stopped
    if (!logging) {
        return;
    }

    // Disable logging flag
    logging = false;

    // Wait for queue to empty (with timeout to prevent infinite loop)
    int timeout = 0;
    while (uxQueueMessagesWaiting(logQueue) > 0 && timeout < 100) {
        vTaskDelay(pdMS_TO_TICKS(50));  // Wait 50ms
        timeout++;
    }

    // Acquire mutex to safely access SD card
    if (xSemaphoreTake(sdMutex, portMAX_DELAY)) {
        // Close log file if open
        if (logFile) {
            logFile.flush();  // Write any buffered data
            logFile.close();  // Close the file
        }
        xSemaphoreGive(sdMutex);  // Release mutex
    }
}

// Create a new log file with timestamp-based name
void CANLogger::createNewLogFile() {
    // Try to acquire mutex (wait forever if necessary)
    if (!xSemaphoreTake(sdMutex, portMAX_DELAY)) {
        return;
    }

    // Close previous log file if open
    if (logFile) {
        logFile.flush();
        logFile.close();
    }

    // Record when this file was created (for rotation)
    fileStartTime = millis();

    // Generate filename based on timestamp: /canlog_12345678.csv
    snprintf(currentFilename, sizeof(currentFilename),
             "/canlog_%lu.csv", fileStartTime);

    // Open file for writing (creates if doesn't exist)
    logFile = SD_MMC.open(currentFilename, FILE_WRITE);

    // Write CSV header if file opened successfully
    if (logFile) {
        writeHeader();
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
        return;
    }

    // Force write buffered data to SD card
    logFile.flush();

    // Check if file rotation is needed (file age > 7 minutes)
    if (millis() - fileStartTime > FILE_ROTATION_MS) {
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
            xQueueSendToFront(logQueue, &entry, 0);
            break;  // Stop processing this batch
        }
    }

    // Check if it's time to flush (every FLUSH_INTERVAL_MS = 2 seconds)
    if (millis() - lastFlushMs > FLUSH_INTERVAL_MS) {
        // Try to acquire mutex with 10ms timeout
        if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(10))) {
            flushAndRotate();  // Flush data and rotate file if needed
            xSemaphoreGive(sdMutex);
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
