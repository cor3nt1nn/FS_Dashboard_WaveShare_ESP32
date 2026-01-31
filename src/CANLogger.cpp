#include "CANLogger.h"

CANLogger::CANLogger()
    : logging(false),
      sdInitialized(false),
      framesLogged(0),
      framesDropped(0),
      fileStartTime(0),
      lastFlushMs(0) {
    logQueue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(CANLogEntry));
    sdMutex = xSemaphoreCreateMutex(); // Mutex to protect SD card access
}

CANLogger::~CANLogger() {
    if (logging) {
        stopLogging();
    }

    if (sdInitialized) {
        SD.end();
    }

    if (logQueue) vQueueDelete(logQueue);
    if (sdMutex) vSemaphoreDelete(sdMutex);
}

bool CANLogger::begin() {
    // Initialize SPI for SD card
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

    // Initialize the SD card
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SD Card initialization failed!");
        return false;
    }

    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("No SD card attached");
        return false;
    }

    Serial.print("SD Card Type: ");
    if (cardType == CARD_MMC) Serial.println("MMC");
    else if (cardType == CARD_SD) Serial.println("SDSC");
    else if (cardType == CARD_SDHC) Serial.println("SDHC");
    else Serial.println("UNKNOWN");

    uint64_t cardSize = SD.cardSize() / (1024 * 1024); // Convert to MB
    Serial.printf("SD Card Size: %lluMB\n", cardSize);

    sdInitialized = true;
    Serial.println("CAN Logger initialized");
    return true;
}

// Main loop to process the log queue and write to SD card
void CANLogger::loop() {
    if (!logging) {
        vTaskDelay(pdMS_TO_TICKS(100));
        return;
    }

    processQueue();
}

// Add a CAN frame to the log queue (non-blocking)
bool CANLogger::logFrame(const twai_message_t &msg) {
    if (!logging) return false;

    CANLogEntry entry;
    entry.timestamp = millis();
    entry.identifier = msg.identifier;
    entry.dlc = msg.data_length_code;
    entry.extd = msg.extd;

    for (int i = 0; i < msg.data_length_code && i < 8; i++) {
        entry.data[i] = msg.data[i];
    }

    if (xQueueSend(logQueue, &entry, 0) != pdTRUE) {
        framesDropped++;
        return false;
    }

    return true;
}

// Start logging CAN frames to SD card
void CANLogger::startLogging() {
    if (!sdInitialized) {
        Serial.println("SD not initialized!");
        return;
    }

    if (logging) return;

    createNewLogFile();
    logging = true;
    Serial.println("Logging started");
}

// Stop logging CAN frames to SD card
void CANLogger::stopLogging() {
    if (!logging) return;

    logging = false;

    // Wait for the queue to empty
    vTaskDelay(pdMS_TO_TICKS(100));

    if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE) {
        if (logFile) {
            logFile.close();
        }
        xSemaphoreGive(sdMutex);
    }

    Serial.printf("Logging stopped. Frames: %u, Dropped: %u\n",
                  framesLogged.load(), framesDropped.load());
}

// Create a new log file on the SD card
void CANLogger::createNewLogFile() {
    if (xSemaphoreTake(sdMutex, portMAX_DELAY) != pdTRUE) return;

    // Close the previous file if open
    if (logFile) {
        logFile.close();
    }

    // Generate a filename with timestamp
    fileStartTime = millis();
    snprintf(currentFilename, sizeof(currentFilename),
             "/canlog_%lu.csv", fileStartTime);

    logFile = SD.open(currentFilename, FILE_WRITE);

    if (logFile) {
        writeHeader();
        Serial.printf("Created log file: %s\n", currentFilename);
    } else {
        Serial.println("Failed to create log file!");
    }

    xSemaphoreGive(sdMutex);
}

// Write the header to the current log file
void CANLogger::writeHeader() {
    if (!logFile) return;

    logFile.println("Timestamp(ms),ID,Extended,DLC,Data");
}

// Write a CAN log entry to the current log file
void CANLogger::writeEntry(const CANLogEntry &entry) {
    if (!logFile) return;

    // Format: timestamp,ID,ext,dlc,data0,data1,...
    logFile.printf("%lu,0x%08X,%d,%d",
                   entry.timestamp,
                   entry.identifier,
                   entry.extd ? 1 : 0,
                   entry.dlc);

    for (int i = 0; i < entry.dlc && i < 8; i++) {
        logFile.printf(",%02X", entry.data[i]);
    }

    logFile.println();
    framesLogged++;
}

// Flush the current log file and rotate to a new one if needed
void CANLogger::flushAndRotate() {
    if (!logFile) return;

    logFile.flush();

    uint32_t currentTime = millis();
    if (currentTime - fileStartTime > 600000) {  // 10 min
        Serial.println("Rotating log file...");
        createNewLogFile();
    }
}

// Process the log queue and write entries to the SD card
void CANLogger::processQueue() {
    CANLogEntry entries[20];  // Local buffer for batch writing

    // Batch read: read up to 20 messages at once
    int count = 0;
    while (count < 20 && xQueueReceive(logQueue, &entries[count], 0) == pdTRUE) {
        count++;
    }

    if (count > 0) {
        if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            for (int i = 0; i < count; i++) {
                writeEntry(entries[i]);
            }
            xSemaphoreGive(sdMutex);
        }
    }

    uint32_t now = millis();
    if (now - lastFlushMs > 2000) {
        if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            flushAndRotate();
            xSemaphoreGive(sdMutex);
        }
        lastFlushMs = now;
    }

    if (count > 0) {
        taskYIELD();
    } else {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
