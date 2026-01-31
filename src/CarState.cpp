#include "CarState.hpp"

// ========== BatteryElectrical - Setters/Getters individuels ==========
void CarState::setBatteryVoltage(uint16_t voltage_cV) {
    if (batteryElec.tryLock(pdMS_TO_TICKS(10))) {
        batteryElec.voltage_cV = voltage_cV;
        batteryElec.voltage = voltage_cV / 100.0f;
        batteryElec.unlock();
    }
}

uint16_t CarState::getBatteryVoltage_cV() {
    uint16_t value = 0;
    if (batteryElec.tryLock(pdMS_TO_TICKS(10))) {
        value = batteryElec.voltage_cV;
        batteryElec.unlock();
    }
    return value;
}

float CarState::getBatteryVoltage() {
    float value = 0.0f;
    if (batteryElec.tryLock(pdMS_TO_TICKS(10))) {
        value = batteryElec.voltage;
        batteryElec.unlock();
    }
    return value;
}

void CarState::setBatteryAmperage(int16_t amperage_cA) {
    if (batteryElec.tryLock(pdMS_TO_TICKS(10))) {
        batteryElec.amperage_cA = amperage_cA;
        batteryElec.amperage = amperage_cA / 100.0f;
        batteryElec.unlock();
    }
}

int16_t CarState::getBatteryAmperage_cA() {
    int16_t value = 0;
    if (batteryElec.tryLock(pdMS_TO_TICKS(10))) {
        value = batteryElec.amperage_cA;
        batteryElec.unlock();
    }
    return value;
}

float CarState::getBatteryAmperage() {
    float value = 0.0f;
    if (batteryElec.tryLock(pdMS_TO_TICKS(10))) {
        value = batteryElec.amperage;
        batteryElec.unlock();
    }
    return value;
}

void CarState::setBatteryPowerOutput(int32_t powerOutput_mW) {
    if (batteryElec.tryLock(pdMS_TO_TICKS(10))) {
        batteryElec.powerOutput_mW = powerOutput_mW;
        batteryElec.unlock();
    }
}

int32_t CarState::getBatteryPowerOutput() {
    int32_t value = 0;
    if (batteryElec.tryLock(pdMS_TO_TICKS(10))) {
        value = batteryElec.powerOutput_mW;
        batteryElec.unlock();
    }
    return value;
}

// ========== BatteryElectrical - Setter/Getter de module complet ==========
void CarState::setBatteryElectrical(uint16_t voltage_cV, int16_t amperage_cA, int32_t powerOutput_mW) {
    if (batteryElec.tryLock(pdMS_TO_TICKS(10))) {
        batteryElec.voltage_cV = voltage_cV;
        batteryElec.voltage = voltage_cV / 100.0f;
        batteryElec.amperage_cA = amperage_cA;
        batteryElec.amperage = amperage_cA / 100.0f;
        batteryElec.powerOutput_mW = powerOutput_mW;
        batteryElec.unlock();
    }
}

bool CarState::getBatteryElectrical(uint16_t &voltage_cV, int16_t &amperage_cA, int32_t &powerOutput_mW, float &voltage, float &amperage) {
    if (batteryElec.tryLock(pdMS_TO_TICKS(10))) {
        voltage_cV = batteryElec.voltage_cV;
        amperage_cA = batteryElec.amperage_cA;
        powerOutput_mW = batteryElec.powerOutput_mW;
        voltage = batteryElec.voltage;
        amperage = batteryElec.amperage;
        batteryElec.unlock();
        return true;
    }
    return false;
}

// ========== BatteryState - Setters/Getters individuels ==========
void CarState::setBatterySOC(uint16_t soc_permil) {
    if (batteryState.tryLock(pdMS_TO_TICKS(10))) {
        batteryState.soc_permil = soc_permil;
        batteryState.soc = soc_permil / 10;
        batteryState.unlock();
    }
}

uint16_t CarState::getBatterySOC_permil() {
    uint16_t value = 0;
    if (batteryState.tryLock(pdMS_TO_TICKS(10))) {
        value = batteryState.soc_permil;
        batteryState.unlock();
    }
    return value;
}

int CarState::getBatterySOC() {
    int value = 0;
    if (batteryState.tryLock(pdMS_TO_TICKS(10))) {
        value = batteryState.soc;
        batteryState.unlock();
    }
    return value;
}

void CarState::setBatteryTemperature(int16_t temperature_dC) {
    if (batteryState.tryLock(pdMS_TO_TICKS(10))) {
        batteryState.temperature_dC = temperature_dC;
        batteryState.temperature = temperature_dC / 10;
        batteryState.unlock();
    }
}

int16_t CarState::getBatteryTemperature_dC() {
    int16_t value = 0;
    if (batteryState.tryLock(pdMS_TO_TICKS(10))) {
        value = batteryState.temperature_dC;
        batteryState.unlock();
    }
    return value;
}

int CarState::getBatteryTemperature() {
    int value = 0;
    if (batteryState.tryLock(pdMS_TO_TICKS(10))) {
        value = batteryState.temperature;
        batteryState.unlock();
    }
    return value;
}

void CarState::setBatteryEnergyAvailable(int64_t energyAvailable_mWh) {
    if (batteryState.tryLock(pdMS_TO_TICKS(10))) {
        batteryState.energyAvailable_mWh = energyAvailable_mWh;
        batteryState.unlock();
    }
}

int64_t CarState::getBatteryEnergyAvailable() {
    int64_t value = 0;
    if (batteryState.tryLock(pdMS_TO_TICKS(10))) {
        value = batteryState.energyAvailable_mWh;
        batteryState.unlock();
    }
    return value;
}

// ========== BatteryState - Setter/Getter de module complet ==========
void CarState::setBatteryState(uint16_t soc_permil, int16_t temperature_dC, int64_t energyAvailable_mWh) {
    if (batteryState.tryLock(pdMS_TO_TICKS(10))) {
        batteryState.soc_permil = soc_permil;
        batteryState.soc = soc_permil / 10;
        batteryState.temperature_dC = temperature_dC;
        batteryState.temperature = temperature_dC / 10;
        batteryState.energyAvailable_mWh = energyAvailable_mWh;
        batteryState.unlock();
    }
}

bool CarState::getBatteryState(uint16_t &soc_permil, int16_t &temperature_dC, int64_t &energyAvailable_mWh, int &soc, int &temperature) {
    if (batteryState.tryLock(pdMS_TO_TICKS(10))) {
        soc_permil = batteryState.soc_permil;
        temperature_dC = batteryState.temperature_dC;
        energyAvailable_mWh = batteryState.energyAvailable_mWh;
        soc = batteryState.soc;
        temperature = batteryState.temperature;
        batteryState.unlock();
        return true;
    }
    return false;
}

// ========== TemperatureData - Setters/Getters individuels ==========
void CarState::setEngineTemperature(uint16_t engine_dC) {
    if (temps.tryLock(pdMS_TO_TICKS(10))) {
        temps.engine_dC = engine_dC;
        temps.engine = engine_dC / 10;
        temps.unlock();
    }
}

uint16_t CarState::getEngineTemperature_dC() {
    uint16_t value = 0;
    if (temps.tryLock(pdMS_TO_TICKS(10))) {
        value = temps.engine_dC;
        temps.unlock();
    }
    return value;
}

int CarState::getEngineTemperature() {
    int value = 0;
    if (temps.tryLock(pdMS_TO_TICKS(10))) {
        value = temps.engine;
        temps.unlock();
    }
    return value;
}

void CarState::setInverterTemperature(uint16_t inverter_dC) {
    if (temps.tryLock(pdMS_TO_TICKS(10))) {
        temps.inverter_dC = inverter_dC;
        temps.inverter = inverter_dC / 10;
        temps.unlock();
    }
}

uint16_t CarState::getInverterTemperature_dC() {
    uint16_t value = 0;
    if (temps.tryLock(pdMS_TO_TICKS(10))) {
        value = temps.inverter_dC;
        temps.unlock();
    }
    return value;
}

int CarState::getInverterTemperature() {
    int value = 0;
    if (temps.tryLock(pdMS_TO_TICKS(10))) {
        value = temps.inverter;
        temps.unlock();
    }
    return value;
}

// ========== TemperatureData - Setter/Getter de module complet ==========
void CarState::setTemperatures(uint16_t engine_dC, uint16_t inverter_dC) {
    if (temps.tryLock(pdMS_TO_TICKS(10))) {
        temps.engine_dC = engine_dC;
        temps.engine = engine_dC / 10;
        temps.inverter_dC = inverter_dC;
        temps.inverter = inverter_dC / 10;
        temps.unlock();
    }
}

bool CarState::getTemperatures(uint16_t &engine_dC, uint16_t &inverter_dC, int &engine, int &inverter) {
    if (temps.tryLock(pdMS_TO_TICKS(10))) {
        engine_dC = temps.engine_dC;
        inverter_dC = temps.inverter_dC;
        engine = temps.engine;
        inverter = temps.inverter;
        temps.unlock();
        return true;
    }
    return false;
}

// ========== EnergyData - Setters/Getters individuels ==========
void CarState::setTripDistance(int32_t tripDistance_m) {
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        energy.tripDistance_m = tripDistance_m;
        energy.unlock();
    }
}

int32_t CarState::getTripDistance() {
    int32_t value = 0;
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        value = energy.tripDistance_m;
        energy.unlock();
    }
    return value;
}

void CarState::setEnergyConsumed(int64_t energyConsumed_mWh) {
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        energy.energyConsumed_mWh = energyConsumed_mWh;
        energy.unlock();
    }
}

int64_t CarState::getEnergyConsumed() {
    int64_t value = 0;
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        value = energy.energyConsumed_mWh;
        energy.unlock();
    }
    return value;
}

void CarState::setEnergyRegenerated(int64_t energyRegenerated_mWh) {
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        energy.energyRegenerated_mWh = energyRegenerated_mWh;
        energy.unlock();
    }
}

int64_t CarState::getEnergyRegenerated() {
    int64_t value = 0;
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        value = energy.energyRegenerated_mWh;
        energy.unlock();
    }
    return value;
}

void CarState::setEnergyConsumption(int32_t energyConsumption_cWhKm) {
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        energy.energyConsumption_cWhKm = energyConsumption_cWhKm;
        energy.consumption = energyConsumption_cWhKm / 100.0f;
        energy.unlock();
    }
}

int32_t CarState::getEnergyConsumption_cWhKm() {
    int32_t value = 0;
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        value = energy.energyConsumption_cWhKm;
        energy.unlock();
    }
    return value;
}

float CarState::getEnergyConsumption() {
    float value = 0.0f;
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        value = energy.consumption;
        energy.unlock();
    }
    return value;
}

void CarState::setRangeRemaining(int32_t rangeRemaining_m) {
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        energy.rangeRemaining_m = rangeRemaining_m;
        energy.unlock();
    }
}

int32_t CarState::getRangeRemaining() {
    int32_t value = 0;
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        value = energy.rangeRemaining_m;
        energy.unlock();
    }
    return value;
}

void CarState::setEnergyDelta(int32_t energyDelta_cm) {
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        energy.energyDelta_cm = energyDelta_cm;
        energy.delta = energyDelta_cm / 100.0f;
        energy.unlock();
    }
}

int32_t CarState::getEnergyDelta_cm() {
    int32_t value = 0;
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        value = energy.energyDelta_cm;
        energy.unlock();
    }
    return value;
}

float CarState::getEnergyDelta() {
    float value = 0.0f;
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        value = energy.delta;
        energy.unlock();
    }
    return value;
}

// ========== EnergyData - Setter/Getter de module complet ==========
void CarState::setEnergyData(int32_t tripDistance_m, int64_t energyConsumed_mWh,
                   int64_t energyRegenerated_mWh, int32_t energyConsumption_cWhKm,
                   int32_t rangeRemaining_m, int32_t energyDelta_cm) {
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        energy.tripDistance_m = tripDistance_m;
        energy.energyConsumed_mWh = energyConsumed_mWh;
        energy.energyRegenerated_mWh = energyRegenerated_mWh;
        energy.energyConsumption_cWhKm = energyConsumption_cWhKm;
        energy.consumption = energyConsumption_cWhKm / 100.0f;
        energy.rangeRemaining_m = rangeRemaining_m;
        energy.energyDelta_cm = energyDelta_cm;
        energy.delta = energyDelta_cm / 100.0f;
        energy.unlock();
    }
}

bool CarState::getEnergyData(int32_t &tripDistance_m, int64_t &energyConsumed_mWh,
                   int64_t &energyRegenerated_mWh, int32_t &energyConsumption_cWhKm,
                   int32_t &rangeRemaining_m, int32_t &energyDelta_cm,
                   float &consumption, float &delta) {
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        tripDistance_m = energy.tripDistance_m;
        energyConsumed_mWh = energy.energyConsumed_mWh;
        energyRegenerated_mWh = energy.energyRegenerated_mWh;
        energyConsumption_cWhKm = energy.energyConsumption_cWhKm;
        rangeRemaining_m = energy.rangeRemaining_m;
        energyDelta_cm = energy.energyDelta_cm;
        consumption = energy.consumption;
        delta = energy.delta;
        energy.unlock();
        return true;
    }
    return false;
}
