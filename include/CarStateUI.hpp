#ifndef CAR_STATE_UI_H
#define CAR_STATE_UI_H

#include "CarState.hpp"
#include "lvgl.h"
#include <Arduino.h>

struct DisplayField {
    float lastValue; // last value displayed on the UI
    uint32_t lastUpdateMs; // last update time in milliseconds
    uint32_t refreshIntervalMs; // refresh interval in milliseconds
    float valueThreshold; // minimum change in value to trigger a UI update

    DisplayField(float lv = 0, uint32_t lu = 0, uint32_t ri = 100, float vt = 1.0f)
        : lastValue(lv), lastUpdateMs(lu), refreshIntervalMs(ri), valueThreshold(vt) {}
};

class CarStateUI {
private:
    CarState &carState;

    // Display fields for various car state parameters
    DisplayField speedField;
    DisplayField tripMeterField;
    DisplayField deltaField;
    DisplayField consumField;
    DisplayField batteryLevelField;
    DisplayField batteryTempField;
    DisplayField voltageField;
    DisplayField amperageField;
    DisplayField throttleField;
    DisplayField brakeField;
    DisplayField engineTempField;
    DisplayField inverterTempField;

    // Last known colors for various UI elements to avoid unnecessary updates
    uint32_t lastBatteryLevelColor = 0xFFFFFF;
    uint32_t lastDeltaColor = 0xFFFFFF;
    uint32_t lastEngineTempColor = 0x00FF00;
    uint32_t lastInverterTempColor = 0x00FF00;
    bool lastBatteryTempOverheated = false;

    void updateBatteryLevelColor(int level);
    void updateBatteryTempColor(int temp);
    void updateEngineTempColor(int temp);
    void updateInverterTempColor(int temp);
    void updateDeltaColor(float delta);

public:
    CarStateUI(CarState &state);

    void updateVoltageUI();
    void updateAmperageUI();
    void updateBatteryLevelUI();
    void updateBatteryTempUI();
    void updateThrottleUI();
    void updateBrakeUI();
    void updateSpeedUI();
    void updateDistanceUI();
    void updateDeltaUI();
    void updateConsumptionUI();
    void updateEngineTempUI();
    void updateInverterTempUI();
};

#endif
