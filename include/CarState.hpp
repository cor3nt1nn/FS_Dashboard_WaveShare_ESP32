#ifndef CARSTATE_HPP
#define CARSTATE_HPP

#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Arduino.h>

// struct to represent the state of the car
struct CarState {
    // atomic fields because they may be accessed from multiple threads/tasks
    std::atomic<uint16_t> throttlePosition_permil{0};
    std::atomic<uint16_t> brakePosition_permil{0};
    std::atomic<uint16_t> carSpeed_dkmh{0};

    std::atomic<int> throttlePosition{0};
    std::atomic<int> brakePosition{0};
    std::atomic<int> carSpeed{0};

    std::atomic<uint32_t> lastUpdateMs{0};
    std::atomic<uint32_t> lastCalcMs{0};

    // Mutex-protected nested structs for battery electricity data
    struct BatteryElectrical {
        uint16_t voltage_cV{0};
        int16_t amperage_cA{0};
        int32_t powerOutput_mW{0};

        float voltage{0.0f};
        float amperage{0.0f};

        SemaphoreHandle_t mutex{nullptr};

        void init() { mutex = xSemaphoreCreateMutex(); }
        bool tryLock(TickType_t timeout = 0) { return xSemaphoreTake(mutex, timeout) == pdTRUE; }
        void unlock() { xSemaphoreGive(mutex); }
    } batteryElec;

    // Mutex-protected nested struct for battery state
    struct BatteryState {
        uint16_t soc_permil{0};
        int16_t temperature_dC{0};
        int64_t energyAvailable_mWh{0};

        int soc{0};
        int temperature{0};

        SemaphoreHandle_t mutex{nullptr};

        void init() { mutex = xSemaphoreCreateMutex(); }
        bool tryLock(TickType_t timeout = 0) { return xSemaphoreTake(mutex, timeout) == pdTRUE; }
        void unlock() { xSemaphoreGive(mutex); }
    } batteryState;

    // Mutex-protected nested struct for temperature data
    struct TemperatureData {
        uint16_t engine_dC{0};
        uint16_t inverter_dC{0};

        int engine{0};
        int inverter{0};

        SemaphoreHandle_t mutex{nullptr};

        void init() { mutex = xSemaphoreCreateMutex(); }
        bool tryLock(TickType_t timeout = 0) { return xSemaphoreTake(mutex, timeout) == pdTRUE; }
        void unlock() { xSemaphoreGive(mutex); }
    } temps;

    // Mutex-protected nested struct for energy data
    struct EnergyData {
        int32_t tripDistance_m{0};
        int64_t energyConsumed_mWh{0};
        int64_t energyRegenerated_mWh{0};
        int32_t energyConsumption_cWhKm{0};
        int32_t rangeRemaining_m{0};
        int32_t energyDelta_cm{0};

        float consumption{0.0f};
        float delta{0.0f};

        SemaphoreHandle_t mutex{nullptr};

        void init() { mutex = xSemaphoreCreateMutex(); }
        bool tryLock(TickType_t timeout = 0) { return xSemaphoreTake(mutex, timeout) == pdTRUE; }
        void unlock() { xSemaphoreGive(mutex); }
    } energy;

    // default values for race distance and battery capacity
    int64_t batteryCapacity_mWh{11300000};
    int32_t raceDistance_m{20000};

    CarState(float raceDistance_km, float batteryCapacity_kWh) {
        raceDistance_m = (int32_t)(raceDistance_km * 1000);
        batteryCapacity_mWh = (int64_t)(batteryCapacity_kWh * 1000000);
    }

    void init() {
        batteryElec.init();
        batteryState.init();
        temps.init();
        energy.init();
    }

    // ========== Atomic getters/setters for speed, throttle, brake ==========
    int getSpeed() const { return carSpeed.load(std::memory_order_relaxed); }
    int getThrottle() const { return throttlePosition.load(std::memory_order_relaxed); }
    int getBrake() const { return brakePosition.load(std::memory_order_relaxed); }

    uint16_t getSpeedRaw() const {
        return carSpeed_dkmh.load(std::memory_order_relaxed);
    }

    void setSpeed(uint16_t speed_dkmh) {
        carSpeed_dkmh.store(speed_dkmh, std::memory_order_relaxed);
        carSpeed.store(speed_dkmh / 10, std::memory_order_relaxed);
    }

    void setThrottle(uint16_t throttle_permil) {
        throttlePosition_permil.store(throttle_permil, std::memory_order_relaxed);
        throttlePosition.store(throttle_permil / 10, std::memory_order_relaxed);
    }

    void setBrake(uint16_t brake_permil) {
        brakePosition_permil.store(brake_permil, std::memory_order_relaxed);
        brakePosition.store(brake_permil / 10, std::memory_order_relaxed);
    }

    void updateTimestamp() {
        lastUpdateMs.store(millis(), std::memory_order_relaxed);
    }

    uint32_t getLastCalcMs() const {
        return lastCalcMs.load(std::memory_order_relaxed);
    }

    void setLastCalcMs(uint32_t ms) {
        lastCalcMs.store(ms, std::memory_order_relaxed);
    }

    // ========== BatteryElectrical - Setters/Getters individuels ==========
    void setBatteryVoltage(uint16_t voltage_cV);
    uint16_t getBatteryVoltage_cV();
    float getBatteryVoltage();

    void setBatteryAmperage(int16_t amperage_cA);
    int16_t getBatteryAmperage_cA();
    float getBatteryAmperage();

    void setBatteryPowerOutput(int32_t powerOutput_mW);
    int32_t getBatteryPowerOutput();

    // ========== BatteryElectrical - Setter/Getter de module complet ==========
    void setBatteryElectrical(uint16_t voltage_cV, int16_t amperage_cA, int32_t powerOutput_mW);
    bool getBatteryElectrical(uint16_t &voltage_cV, int16_t &amperage_cA, int32_t &powerOutput_mW, float &voltage, float &amperage);

    // ========== BatteryState - Setters/Getters individuels ==========
    void setBatterySOC(uint16_t soc_permil);
    uint16_t getBatterySOC_permil();
    int getBatterySOC();

    void setBatteryTemperature(int16_t temperature_dC);
    int16_t getBatteryTemperature_dC();
    int getBatteryTemperature();

    void setBatteryEnergyAvailable(int64_t energyAvailable_mWh);
    int64_t getBatteryEnergyAvailable();

    // ========== BatteryState - Setter/Getter de module complet ==========
    void setBatteryState(uint16_t soc_permil, int16_t temperature_dC, int64_t energyAvailable_mWh);
    bool getBatteryState(uint16_t &soc_permil, int16_t &temperature_dC, int64_t &energyAvailable_mWh, int &soc, int &temperature);

    // ========== TemperatureData - Setters/Getters individuels ==========
    void setEngineTemperature(uint16_t engine_dC);
    uint16_t getEngineTemperature_dC();
    int getEngineTemperature();

    void setInverterTemperature(uint16_t inverter_dC);
    uint16_t getInverterTemperature_dC();
    int getInverterTemperature();

    // ========== TemperatureData - Setter/Getter de module complet ==========
    void setTemperatures(uint16_t engine_dC, uint16_t inverter_dC);
    bool getTemperatures(uint16_t &engine_dC, uint16_t &inverter_dC, int &engine, int &inverter);

    // ========== EnergyData - Setters/Getters individuels ==========
    void setTripDistance(int32_t tripDistance_m);
    int32_t getTripDistance();

    void setEnergyConsumed(int64_t energyConsumed_mWh);
    int64_t getEnergyConsumed();

    void setEnergyRegenerated(int64_t energyRegenerated_mWh);
    int64_t getEnergyRegenerated();

    void setEnergyConsumption(int32_t energyConsumption_cWhKm);
    int32_t getEnergyConsumption_cWhKm();
    float getEnergyConsumption();

    void setRangeRemaining(int32_t rangeRemaining_m);
    int32_t getRangeRemaining();

    void setEnergyDelta(int32_t energyDelta_cm);
    int32_t getEnergyDelta_cm();
    float getEnergyDelta();

    // ========== EnergyData - Setter/Getter de module complet ==========
    void setEnergyData(int32_t tripDistance_m, int64_t energyConsumed_mWh,
                       int64_t energyRegenerated_mWh, int32_t energyConsumption_cWhKm,
                       int32_t rangeRemaining_m, int32_t energyDelta_cm);
    bool getEnergyData(int32_t &tripDistance_m, int64_t &energyConsumed_mWh,
                       int64_t &energyRegenerated_mWh, int32_t &energyConsumption_cWhKm,
                       int32_t &rangeRemaining_m, int32_t &energyDelta_cm,
                       float &consumption, float &delta);
};

#endif
