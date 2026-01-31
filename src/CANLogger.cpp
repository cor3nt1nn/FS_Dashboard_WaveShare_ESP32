#include "CANLogger.hpp"

CANLogger::CANLogger()
    : logging(false),
      sdInitialized(false),
      framesLogged(0),
      framesDropped(0),
      fileStartTime(0),
      lastFlushMs(0) {

    logQueue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(CANLogEntry));
    sdMutex = xSemaphoreCreateMutex();
}

CANLogger::~CANLogger() {
    stopLogging();

    if (sdInitialized) {
        SD_MMC.end();
    }
    if (logQueue) {
        vQueueDelete(logQueue);
    }
    if (sdMutex) {
        vSemaphoreDelete(sdMutex);
    }
}

bool CANLogger::begin() {
    SD_MMC.setPins(12, 11, 13);
    delay(500);

    if(!SD_MMC.begin("/sdcard", true)){
        SD_MMC.end();
        sdInitialized=false;
        return false;
    }
    sdInitialized=true;
    return true;
}

void CANLogger::loop() {
    if (!logging) {
        vTaskDelay(pdMS_TO_TICKS(100));
        return;
    }
    processQueue();
}

bool CANLogger::logFrame(const twai_message_t &msg) {
    if (!logging) {
        return false;
    }

    CANLogEntry entry;
    entry.timestamp = millis();
    entry.identifier = msg.identifier;
    entry.dlc = msg.data_length_code;
    entry.extd = msg.extd;

    for (int i = 0; i < entry.dlc && i < 8; i++) {
        entry.data[i] = msg.data[i];
    }

    if (xQueueSend(logQueue, &entry, 0) != pdTRUE) {
        framesDropped++;
        return false;
    }
    return true;
}

void CANLogger::startLogging() {
    if (!sdInitialized || logging) {
        return;
    }

    createNewLogFile();
    logging = true;
}

void CANLogger::stopLogging() {
    if (!logging) {
        return;
    }

    logging = false;
    int timeout = 0;
    while (uxQueueMessagesWaiting(logQueue) > 0 && timeout < 100) {
        vTaskDelay(pdMS_TO_TICKS(50));
        timeout++;
    }

    if (xSemaphoreTake(sdMutex, portMAX_DELAY)) {
        if (logFile) {
            logFile.flush();
            logFile.close();
        }
        xSemaphoreGive(sdMutex);
    }
}

void CANLogger::createNewLogFile() {
    if (!xSemaphoreTake(sdMutex, portMAX_DELAY)) {
        return;
    }

    if (logFile) {
        logFile.flush();
        logFile.close();
    }

    fileStartTime = millis();
    snprintf(currentFilename, sizeof(currentFilename),
             "/canlog_%lu.csv", fileStartTime);

    logFile = SD_MMC.open(currentFilename, FILE_WRITE);

    if (logFile) {
        writeHeader();
    }

    xSemaphoreGive(sdMutex);
}

void CANLogger::writeHeader() {
    logFile.println("Timestamp,ID,Ext,DLC,Data");
}

void CANLogger::writeEntry(const CANLogEntry &entry) {
    if (!logFile) {
        return;
    }
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

void CANLogger::flushAndRotate() {
    if (!logFile) {
        return;
    }
    logFile.flush();
    if (millis() - fileStartTime > FILE_ROTATION_MS) {
        createNewLogFile();
    }
}

void CANLogger::processQueue() {
    CANLogEntry entry;
    int processed = 0;

    while (processed < MAX_BATCH_SIZE &&
           xQueueReceive(logQueue, &entry, 0) == pdTRUE) {

        if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(10))) {
            writeEntry(entry);
            xSemaphoreGive(sdMutex);
            processed++;
        } else {
            xQueueSendToFront(logQueue, &entry, 0);
            break;
        }
    }

    if (millis() - lastFlushMs > FLUSH_INTERVAL_MS) {
        if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(10))) {
            flushAndRotate();
            xSemaphoreGive(sdMutex);
        }
        lastFlushMs = millis();
    }

    if (processed > 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
    } else {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
