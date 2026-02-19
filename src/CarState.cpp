#include "CarState.hpp"
#include "Debug.hpp"

// ========== BatteryElectrical - Setters/Getters ==========
void CarState::setBatteryVoltage(uint16_t voltage_cV) {
    if (batteryElec.tryLock(pdMS_TO_TICKS(10))) {
        batteryElec.voltage_cV = voltage_cV;
        batteryElec.voltage = voltage_cV / 100.0f;
        batteryElec.unlock();
    } else {
        LOG_W(TAG_STATE, "setBatteryVoltage() mutex timeout");
    }
}

uint16_t CarState::getBatteryVoltage_cV() {
    uint16_t value = 0;
    if (batteryElec.tryLock(pdMS_TO_TICKS(10))) {
        value = batteryElec.voltage_cV;
        batteryElec.unlock();
    } else {
        LOG_W(TAG_STATE, "getBatteryVoltage_cV() mutex timeout");
    }
    return value;
}

float CarState::getBatteryVoltage() {
    float value = 0.0f;
    if (batteryElec.tryLock(pdMS_TO_TICKS(10))) {
        value = batteryElec.voltage;
        batteryElec.unlock();
    } else {
        LOG_W(TAG_STATE, "getBatteryVoltage() mutex timeout");
    }
    return value;
}

void CarState::setBatteryAmperage(int16_t amperage_cA) {
    if (batteryElec.tryLock(pdMS_TO_TICKS(10))) {
        batteryElec.amperage_cA = amperage_cA;
        batteryElec.amperage = amperage_cA / 100.0f;
        batteryElec.unlock();
    } else {
        LOG_W(TAG_STATE, "setBatteryAmperage() mutex timeout");
    }
}

int16_t CarState::getBatteryAmperage_cA() {
    int16_t value = 0;
    if (batteryElec.tryLock(pdMS_TO_TICKS(10))) {
        value = batteryElec.amperage_cA;
        batteryElec.unlock();
    } else {
        LOG_W(TAG_STATE, "getBatteryAmperage_cA() mutex timeout");
    }
    return value;
}

float CarState::getBatteryAmperage() {
    float value = 0.0f;
    if (batteryElec.tryLock(pdMS_TO_TICKS(10))) {
        value = batteryElec.amperage;
        batteryElec.unlock();
    } else {
        LOG_W(TAG_STATE, "getBatteryAmperage() mutex timeout");
    }
    return value;
}

void CarState::setBatteryPowerOutput(int32_t powerOutput_mW) {
    if (batteryElec.tryLock(pdMS_TO_TICKS(10))) {
        batteryElec.powerOutput_mW = powerOutput_mW;
        batteryElec.unlock();
    } else {
        LOG_W(TAG_STATE, "setBatteryPowerOutput() mutex timeout");
    }
}

int32_t CarState::getBatteryPowerOutput() {
    int32_t value = 0;
    if (batteryElec.tryLock(pdMS_TO_TICKS(10))) {
        value = batteryElec.powerOutput_mW;
        batteryElec.unlock();
    } else {
        LOG_W(TAG_STATE, "getBatteryPowerOutput() mutex timeout");
    }
    return value;
}

// ========== BatteryElectrical - Setter/Getter ==========
void CarState::setBatteryElectrical(uint16_t voltage_cV, int16_t amperage_cA, int32_t powerOutput_mW) {
    if (batteryElec.tryLock(pdMS_TO_TICKS(10))) {
        batteryElec.voltage_cV = voltage_cV;
        batteryElec.voltage = voltage_cV / 100.0f;
        batteryElec.amperage_cA = amperage_cA;
        batteryElec.amperage = amperage_cA / 100.0f;
        batteryElec.powerOutput_mW = powerOutput_mW;
        batteryElec.unlock();
    } else {
        LOG_W(TAG_STATE, "setBatteryElectrical() mutex timeout");
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
    LOG_W(TAG_STATE, "getBatteryElectrical() mutex timeout");
    return false;
}

// ========== BatteryState - Setters/Getters  ==========
void CarState::setBatterySOC(uint16_t soc_permil) {
    if (batteryState.tryLock(pdMS_TO_TICKS(10))) {
        batteryState.soc_permil = soc_permil;
        batteryState.soc = soc_permil / 10;
        batteryState.unlock();
    } else {
        LOG_W(TAG_STATE, "setBatterySOC() mutex timeout");
    }
}

uint16_t CarState::getBatterySOC_permil() {
    uint16_t value = 0;
    if (batteryState.tryLock(pdMS_TO_TICKS(10))) {
        value = batteryState.soc_permil;
        batteryState.unlock();
    } else {
        LOG_W(TAG_STATE, "getBatterySOC_permil() mutex timeout");
    }
    return value;
}

uint8_t CarState::getBatterySOC() {
    uint8_t value = 0;
    if (batteryState.tryLock(pdMS_TO_TICKS(10))) {
        value = batteryState.soc;
        batteryState.unlock();
    } else {
        LOG_W(TAG_STATE, "getBatterySOC() mutex timeout");
    }
    return value;
}

void CarState::setBatteryTemperature(int16_t temperature_dC) {
    if (batteryState.tryLock(pdMS_TO_TICKS(10))) {
        batteryState.temperature_dC = temperature_dC;
        batteryState.temperature = temperature_dC / 10;
        batteryState.unlock();
    } else {
        LOG_W(TAG_STATE, "setBatteryTemperature() mutex timeout");
    }
}

int16_t CarState::getBatteryTemperature_dC() {
    int16_t value = 0;
    if (batteryState.tryLock(pdMS_TO_TICKS(10))) {
        value = batteryState.temperature_dC;
        batteryState.unlock();
    } else {
        LOG_W(TAG_STATE, "getBatteryTemperature_dC() mutex timeout");
    }
    return value;
}

int16_t CarState::getBatteryTemperature() {
    int16_t value = 0;
    if (batteryState.tryLock(pdMS_TO_TICKS(10))) {
        value = batteryState.temperature;
        batteryState.unlock();
    } else {
        LOG_W(TAG_STATE, "getBatteryTemperature() mutex timeout");
    }
    return value;
}

void CarState::setBatteryEnergyAvailable(int64_t energyAvailable_mWh) {
    if (batteryState.tryLock(pdMS_TO_TICKS(10))) {
        batteryState.energyAvailable_mWh = energyAvailable_mWh;
        batteryState.unlock();
    } else {
        LOG_W(TAG_STATE, "setBatteryEnergyAvailable() mutex timeout");
    }
}

int64_t CarState::getBatteryEnergyAvailable() {
    int64_t value = 0;
    if (batteryState.tryLock(pdMS_TO_TICKS(10))) {
        value = batteryState.energyAvailable_mWh;
        batteryState.unlock();
    } else {
        LOG_W(TAG_STATE, "getBatteryEnergyAvailable() mutex timeout");
    }
    return value;
}

// ========== BatteryState - Setter/Getter ==========
void CarState::setBatteryState(uint16_t soc_permil, int16_t temperature_dC, int64_t energyAvailable_mWh) {
    if (batteryState.tryLock(pdMS_TO_TICKS(10))) {
        batteryState.soc_permil = soc_permil;
        batteryState.soc = soc_permil / 10;
        batteryState.temperature_dC = temperature_dC;
        batteryState.temperature = temperature_dC / 10;
        batteryState.energyAvailable_mWh = energyAvailable_mWh;
        batteryState.unlock();
    } else {
        LOG_W(TAG_STATE, "setBatteryState() mutex timeout");
    }
}

bool CarState::getBatteryState(uint16_t &soc_permil, int16_t &temperature_dC, int64_t &energyAvailable_mWh, uint8_t &soc, int16_t &temperature) {
    if (batteryState.tryLock(pdMS_TO_TICKS(10))) {
        soc_permil = batteryState.soc_permil;
        temperature_dC = batteryState.temperature_dC;
        energyAvailable_mWh = batteryState.energyAvailable_mWh;
        soc = batteryState.soc;
        temperature = batteryState.temperature;
        batteryState.unlock();
        return true;
    }
    LOG_W(TAG_STATE, "getBatteryState() mutex timeout");
    return false;
}

// ========== TemperatureData - Setters/Getters  ==========
void CarState::setEngineTemperature(uint16_t engine_dC) {
    if (temps.tryLock(pdMS_TO_TICKS(10))) {
        temps.engine_dC = engine_dC;
        temps.engine = engine_dC / 10;
        temps.unlock();
    } else {
        LOG_W(TAG_STATE, "setEngineTemperature() mutex timeout");
    }
}

uint16_t CarState::getEngineTemperature_dC() {
    uint16_t value = 0;
    if (temps.tryLock(pdMS_TO_TICKS(10))) {
        value = temps.engine_dC;
        temps.unlock();
    } else {
        LOG_W(TAG_STATE, "getEngineTemperature_dC() mutex timeout");
    }
    return value;
}

uint8_t CarState::getEngineTemperature() {
    uint8_t value = 0;
    if (temps.tryLock(pdMS_TO_TICKS(10))) {
        value = temps.engine;
        temps.unlock();
    } else {
        LOG_W(TAG_STATE, "getEngineTemperature() mutex timeout");
    }
    return value;
}

void CarState::setInverterTemperature(uint16_t inverter_dC) {
    if (temps.tryLock(pdMS_TO_TICKS(10))) {
        temps.inverter_dC = inverter_dC;
        temps.inverter = inverter_dC / 10;
        temps.unlock();
    } else {
        LOG_W(TAG_STATE, "setInverterTemperature() mutex timeout");
    }
}

uint16_t CarState::getInverterTemperature_dC() {
    uint16_t value = 0;
    if (temps.tryLock(pdMS_TO_TICKS(10))) {
        value = temps.inverter_dC;
        temps.unlock();
    } else {
        LOG_W(TAG_STATE, "getInverterTemperature_dC() mutex timeout");
    }
    return value;
}

uint8_t CarState::getInverterTemperature() {
    uint8_t value = 0;
    if (temps.tryLock(pdMS_TO_TICKS(10))) {
        value = temps.inverter;
        temps.unlock();
    } else {
        LOG_W(TAG_STATE, "getInverterTemperature() mutex timeout");
    }
    return value;
}

// ========== TemperatureData - Setter/Getter ==========
void CarState::setTemperatures(uint16_t engine_dC, uint16_t inverter_dC) {
    if (temps.tryLock(pdMS_TO_TICKS(10))) {
        temps.engine_dC = engine_dC;
        temps.engine = engine_dC / 10;
        temps.inverter_dC = inverter_dC;
        temps.inverter = inverter_dC / 10;
        temps.unlock();
    } else {
        LOG_W(TAG_STATE, "setTemperatures() mutex timeout");
    }
}

bool CarState::getTemperatures(uint16_t &engine_dC, uint16_t &inverter_dC, uint8_t &engine, uint8_t &inverter) {
    if (temps.tryLock(pdMS_TO_TICKS(10))) {
        engine_dC = temps.engine_dC;
        inverter_dC = temps.inverter_dC;
        engine = temps.engine;
        inverter = temps.inverter;
        temps.unlock();
        return true;
    }
    LOG_W(TAG_STATE, "getTemperatures() mutex timeout");
    return false;
}

// ========== EnergyData - Setters/Getters ==========
void CarState::setTripDistance(int32_t tripDistance_m) {
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        energy.tripDistance_m = tripDistance_m;
        energy.unlock();
    } else {
        LOG_W(TAG_STATE, "setTripDistance() mutex timeout");
    }
}

int32_t CarState::getTripDistance() {
    int32_t value = 0;
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        value = energy.tripDistance_m;
        energy.unlock();
    } else {
        LOG_W(TAG_STATE, "getTripDistance() mutex timeout");
    }
    return value;
}

void CarState::setEnergyConsumed(int64_t energyConsumed_mWh) {
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        energy.energyConsumed_mWh = energyConsumed_mWh;
        energy.unlock();
    } else {
        LOG_W(TAG_STATE, "setEnergyConsumed() mutex timeout");
    }
}

int64_t CarState::getEnergyConsumed() {
    int64_t value = 0;
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        value = energy.energyConsumed_mWh;
        energy.unlock();
    } else {
        LOG_W(TAG_STATE, "getEnergyConsumed() mutex timeout");
    }
    return value;
}

void CarState::setEnergyRegenerated(int64_t energyRegenerated_mWh) {
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        energy.energyRegenerated_mWh = energyRegenerated_mWh;
        energy.unlock();
    } else {
        LOG_W(TAG_STATE, "setEnergyRegenerated() mutex timeout");
    }
}

int64_t CarState::getEnergyRegenerated() {
    int64_t value = 0;
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        value = energy.energyRegenerated_mWh;
        energy.unlock();
    } else {
        LOG_W(TAG_STATE, "getEnergyRegenerated() mutex timeout");
    }
    return value;
}

void CarState::setEnergyConsumption(int32_t energyConsumption_cWhKm) {
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        energy.energyConsumption_cWhKm = energyConsumption_cWhKm;
        energy.consumption = energyConsumption_cWhKm / 100.0f;
        energy.unlock();
    } else {
        LOG_W(TAG_STATE, "setEnergyConsumption() mutex timeout");
    }
}

int32_t CarState::getEnergyConsumption_cWhKm() {
    int32_t value = 0;
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        value = energy.energyConsumption_cWhKm;
        energy.unlock();
    } else {
        LOG_W(TAG_STATE, "getEnergyConsumption_cWhKm() mutex timeout");
    }
    return value;
}

float CarState::getEnergyConsumption() {
    float value = 0.0f;
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        value = energy.consumption;
        energy.unlock();
    } else {
        LOG_W(TAG_STATE, "getEnergyConsumption() mutex timeout");
    }
    return value;
}

void CarState::setRangeRemaining(int32_t rangeRemaining_m) {
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        energy.rangeRemaining_m = rangeRemaining_m;
        energy.unlock();
    } else {
        LOG_W(TAG_STATE, "setRangeRemaining() mutex timeout");
    }
}

int32_t CarState::getRangeRemaining() {
    int32_t value = 0;
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        value = energy.rangeRemaining_m;
        energy.unlock();
    } else {
        LOG_W(TAG_STATE, "getRangeRemaining() mutex timeout");
    }
    return value;
}

void CarState::setEnergyDelta(int32_t energyDelta_cm) {
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        energy.energyDelta_cm = energyDelta_cm;
        energy.delta = energyDelta_cm / 100.0f;
        energy.unlock();
    } else {
        LOG_W(TAG_STATE, "setEnergyDelta() mutex timeout");
    }
}

int32_t CarState::getEnergyDelta_cm() {
    int32_t value = 0;
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        value = energy.energyDelta_cm;
        energy.unlock();
    } else {
        LOG_W(TAG_STATE, "getEnergyDelta_cm() mutex timeout");
    }
    return value;
}

float CarState::getEnergyDelta() {
    float value = 0.0f;
    if (energy.tryLock(pdMS_TO_TICKS(10))) {
        value = energy.delta;
        energy.unlock();
    } else {
        LOG_W(TAG_STATE, "getEnergyDelta() mutex timeout");
    }
    return value;
}

// ========== EnergyData - Setter/Getter  ==========
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
    } else {
        LOG_W(TAG_STATE, "setEnergyData() mutex timeout");
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
    LOG_W(TAG_STATE, "getEnergyData() mutex timeout");
    return false;
}
