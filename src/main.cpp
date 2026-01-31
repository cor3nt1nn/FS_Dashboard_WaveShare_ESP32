/**
 * ================================================================================
 * FORMULA STUDENT DASHBOARD FOR SIGMA RACING USE.
 * ================================================================================
 *
 * Real-time vehicle telemetry display system for Sigma Formula Student car.
 *
 * Main features:
 * - CAN bus communication for vehicle data
 * - Real-time energy consumption calculations
 * - Range estimation and race strategy display
 * - Storage of data to be recovered later.
 * - Multi-threaded architecture for responsive UI
 *
 * Hardware: ESP32-S3 + 800x480 RGB display
 *
 * Author: BESQUEUT Corentin for Sigma Racing Team
 * Version: 1.0
 * Date: January 31th, 2026
 */

#include <Arduino.h>
#include <lvgl.h>
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <esp_heap_caps.h>

#include "ui/ui.h"                   // UI definitions
#include "CANSocket.h"               // CAN communication handler
#include "CarState.hpp"              // Vehicle state data structure
#include "CarStateUI.hpp"            // UI update functions
#include "CANLogger.hpp"         // CAN logging (SD Card via SD_MMC)

// ================================================================================
// CONFIGURATION CONSTANTS
// ================================================================================

#define TFT_BL 2                     // Display backlight pin

// Energy calculation constants
constexpr int32_t ALPHA_FIXED = 154;           // Smoothing filter weight (15%)
constexpr int32_t INV_ALPHA_FIXED = 870;       // Inverse filter weight (85%)
constexpr int32_t MIN_DISTANCE_M = 800;        // Min distance before calculating consumption
constexpr int32_t DEFAULT_CONSUMPTION = 41000; // Default consumption (410 Wh/km)
constexpr int32_t SPEED_THRESHOLD = 10;        // Speed threshold to consider car moving (1 km/h)
constexpr int32_t MIN_CONSUMPTION = 10;        // Min consumption to avoid division by zero
constexpr int32_t DELTA_CLAMP_M = 99999;       // Max delta value to prevent overflow

// ================================================================================
// DISPLAY HARDWARE CONFIGURATION (MUST NOT TO BE MODIFIED)
// ================================================================================

/**
 * LGFX Class - Configures the RGB parallel display interface
 * Sets up 16 data pins + control signals for 800x480 display
 */
class LGFX : public lgfx::LGFX_Device {
public:
  lgfx::Bus_RGB _bus_instance;
  lgfx::Panel_RGB _panel_instance;

  LGFX(void) {
    auto cfg = _bus_instance.config();
    cfg.panel = &_panel_instance;

    // Data pins (16-bit RGB565)
    cfg.pin_d0 = GPIO_NUM_14;
    cfg.pin_d1 = GPIO_NUM_38;
    cfg.pin_d2 = GPIO_NUM_18;
    cfg.pin_d3 = GPIO_NUM_17;
    cfg.pin_d4 = GPIO_NUM_10;
    cfg.pin_d5 = GPIO_NUM_39;
    cfg.pin_d6 = GPIO_NUM_0;
    cfg.pin_d7 = GPIO_NUM_45;
    cfg.pin_d8 = GPIO_NUM_48;
    cfg.pin_d9 = GPIO_NUM_47;
    cfg.pin_d10 = GPIO_NUM_21;
    cfg.pin_d11 = GPIO_NUM_1;
    cfg.pin_d12 = GPIO_NUM_2;
    cfg.pin_d13 = GPIO_NUM_42;
    cfg.pin_d14 = GPIO_NUM_41;
    cfg.pin_d15 = GPIO_NUM_40;

    // Control signals
    cfg.pin_henable = GPIO_NUM_5;
    cfg.pin_vsync = GPIO_NUM_3;
    cfg.pin_hsync = GPIO_NUM_46;
    cfg.pin_pclk = GPIO_NUM_7;

    // Timing parameters
    cfg.freq_write = 15000000;
    cfg.hsync_polarity = 0;
    cfg.hsync_front_porch = 8;
    cfg.hsync_pulse_width = 4;
    cfg.hsync_back_porch = 43;
    cfg.vsync_polarity = 0;
    cfg.vsync_front_porch = 8;
    cfg.vsync_pulse_width = 4;
    cfg.vsync_back_porch = 12;
    cfg.pclk_active_neg = 1;
    cfg.de_idle_high = 0;
    cfg.pclk_idle_high = 0;

    _bus_instance.config(cfg);

    // Panel configuration
    auto pcfg = _panel_instance.config();
    pcfg.memory_width = 800;
    pcfg.memory_height = 480;
    pcfg.panel_width = 800;
    pcfg.panel_height = 480;
    pcfg.offset_x = 0;
    pcfg.offset_y = 0;
    _panel_instance.config(pcfg);

    _panel_instance.setBus(&_bus_instance);
    setPanel(&_panel_instance);
  }
};

LGFX lcd;

// LVGL display buffers
static lv_disp_draw_buf_t draw_buf;
static lv_color_t disp_draw_buf[800*120];
static lv_disp_drv_t disp_drv;

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  lcd.pushImageDMA(area->x1, area->y1, w, h, (lgfx::rgb565_t *)&color_p->full);
  lv_disp_flush_ready(disp);
}

CANSocket can;
CarStateUI car_ui(can.state());
CANLogger canLogger;
static int32_t filteredConsumption_cWhKm = 0; // Smoothed consumption value

/**
 * Processes incoming CAN messages and updates vehicle state
 *
 * CAN IDs:
 * 0x18010001 - Throttle position
 * 0x18010002 - Brake position
 * 0x18010003 - Vehicle speed
 * 0x18020001 - Battery voltage
 * 0x18020002 - Battery current
 * 0x18030001 - Battery SOC (State of Charge)
 * 0x18030002 - Battery temperature
 * 0x18030003 - Motor temperature
 * 0x18030004 - Inverter temperature
 */
void handleCANFrame(const twai_message_t &msg, CarState &carState) {

    canLogger.logFrame(msg);
    uint16_t value_u16;
    int16_t value_i16;

    switch (msg.identifier) {
        case 0x18010001:  // Throttle
            value_u16 = (msg.data[1] << 8) | msg.data[0];
            carState.setThrottle(value_u16);
            break;

        case 0x18010002:  // Brake
            value_u16 = (msg.data[1] << 8) | msg.data[0];
            carState.setBrake(value_u16);
            break;

        case 0x18010003:  // Speed
            value_u16 = (msg.data[1] << 8) | msg.data[0];
            carState.setSpeed(value_u16);
            break;

        case 0x18020001:  // Battery voltage
            value_u16 = (msg.data[1] << 8) | msg.data[0];
            carState.setBatteryVoltage(value_u16);
            break;

        case 0x18020002:  // Battery current
            value_i16 = (int16_t)((msg.data[1] << 8) | msg.data[0]);
            carState.setBatteryAmperage(value_i16);
            // Calculate power output (P = V * I)
            {
                uint16_t voltage_cV = carState.getBatteryVoltage_cV();
                int32_t powerOutput_mW = ((int64_t)voltage_cV * value_i16) * 0.1;
                carState.setBatteryPowerOutput(powerOutput_mW);
            }
            break;

        case 0x18030001:  // Battery SOC
            value_u16 = (msg.data[1] << 8) | msg.data[0];
            carState.setBatterySOC(value_u16);
            // Calculate available energy
            {
                int64_t energyAvailable_mWh = (carState.batteryCapacity_mWh * value_u16) * 0.001f;
                carState.setBatteryEnergyAvailable(energyAvailable_mWh);
            }
            break;

        case 0x18030002:  // Battery temperature
            value_i16 = (int16_t)((msg.data[1] << 8) | msg.data[0]);
            carState.setBatteryTemperature(value_i16);
            break;

        case 0x18030003:  // Motor temperature
            value_u16 = (msg.data[1] << 8) | msg.data[0];
            carState.setEngineTemperature(value_u16);
            break;

        case 0x18030004:  // Inverter temperature
            value_u16 = (msg.data[1] << 8) | msg.data[0];
            carState.setInverterTemperature(value_u16);
            break;
        default:
            break;
    }
    carState.updateTimestamp();
}

/**
 * Core energy management function that runs every 100ms
 *
 * Calculates:
 * - Distance traveled based on speed and time
 * - Energy consumed and regenerated
 * - Average consumption using exponential smoothing filter
 * - Remaining range based on current consumption
 * - Delta (range vs race distance remaining)
 */
void updateEnergyAndDelta() {
    CarState& carState = can.state();
    uint32_t now = millis();
    uint32_t lastCalc = carState.getLastCalcMs();

    // First run initialization
    if (lastCalc == 0) {
        carState.setLastCalcMs(now);
        return;
    }

    // Get current state
    uint32_t deltaTime_ms = now - lastCalc;
    uint16_t speed_dkmh = carState.getSpeedRaw();
    int32_t powerOutput_mW = carState.getBatteryPowerOutput();
    int64_t energyAvailable_mWh = carState.getBatteryEnergyAvailable();

    // Calculate distance traveled (only if moving)
    int32_t distance_m = 0;
    if (speed_dkmh > SPEED_THRESHOLD) {
        distance_m = ((int64_t)speed_dkmh * deltaTime_ms) / 36000;
    }

    // Calculate energy increment (positive = consumed, negative = regenerated)
    int64_t energyIncrement_mWh = ((int64_t)powerOutput_mW * deltaTime_ms) / 3600000;
    int64_t energyConsumed_mWh = carState.getEnergyConsumed();
    int64_t energyRegenerated_mWh = carState.getEnergyRegenerated();
    int32_t tripDistance_m = carState.getTripDistance();

    // Update energy counters
    if (energyIncrement_mWh > 0) {
        energyConsumed_mWh += energyIncrement_mWh;
        carState.setEnergyConsumed(energyConsumed_mWh);
    } else if (energyIncrement_mWh < 0) {
        energyRegenerated_mWh += (-energyIncrement_mWh);
        carState.setEnergyRegenerated(energyRegenerated_mWh);
    }

    // Update trip distance
    tripDistance_m += distance_m;
    carState.setTripDistance(tripDistance_m);

    // Calculate current consumption (Wh/km)
    int32_t currentConsumption_cWhKm;
    if (tripDistance_m >= MIN_DISTANCE_M) {
        int64_t energyNet_mWh = energyConsumed_mWh - energyRegenerated_mWh;
        currentConsumption_cWhKm = (int32_t)((energyNet_mWh * 100) / tripDistance_m);
    } else {
        currentConsumption_cWhKm = DEFAULT_CONSUMPTION;
    }

    // Apply exponential smoothing filter to consumption
    // This prevents jumpy values and provides a more stable reading
    if (filteredConsumption_cWhKm == 0) {
        filteredConsumption_cWhKm = currentConsumption_cWhKm;
    } else {
        filteredConsumption_cWhKm = (ALPHA_FIXED * currentConsumption_cWhKm + INV_ALPHA_FIXED * filteredConsumption_cWhKm) >> 10;
    }
    carState.setEnergyConsumption(filteredConsumption_cWhKm);

    // Calculate remaining range
    int32_t rangeRemaining_m;
    if (filteredConsumption_cWhKm > MIN_CONSUMPTION) {
        rangeRemaining_m = (int32_t)((energyAvailable_mWh * 100) / filteredConsumption_cWhKm);
    } else {
        rangeRemaining_m = 999999;
    }
    carState.setRangeRemaining(rangeRemaining_m);

    // Calculate delta (how much further we can go than race distance)
    int32_t distanceRemaining_m = carState.raceDistance_m - tripDistance_m;
    int32_t delta_m = rangeRemaining_m - distanceRemaining_m;

    // Clamp delta to prevent overflow
    if (delta_m > DELTA_CLAMP_M) delta_m = DELTA_CLAMP_M;
    if (delta_m < -DELTA_CLAMP_M) delta_m = -DELTA_CLAMP_M;

    carState.setEnergyDelta(delta_m * 100);
    carState.setLastCalcMs(now);
}

// ##############################################################################################
// ##############  TO DELETE WHEN THE FIRST TEST WILL HAVE RECORDED DATAS   #####################
// ##############################################################################################

/**
 * Fake CAN Task that reproduce realistic vehicle behavior for debugging.
 *
 * Simulates:
 * - Aerodynamic drag
 * - Rolling resistance
 * - Motor performance
 * - Regenerative braking
 * - Battery voltage drop under load
 *
 * Cycle: Accelerate to 150 km/h → Brake to 25 km/h → Repeat
 */
void fakeCANTask(void *pvParameters) {
    twai_message_t msg;
    msg.extd = 1;
    msg.data_length_code = 2;
    CarState& carState = can.state();

    // Vehicle physics parameters
    const float mass = 350.0f;                      // Vehicle mass (kg)
    const float Cx = 0.18f;                         // Drag coefficient
    const float S = 0.9f;                           // Frontal area (m²)
    const float rho = 1.225f;                       // Air density (kg/m³)
    const float Crr = 0.008f;                       // Rolling resistance coefficient
    const float g = 9.81f;                          // Gravity (m/s²)
    const float motorEff = 0.92f;                   // Motor efficiency (92%)
    const float batteryVoltageNominal = 340.0f;     // Nominal voltage (V)
    const float batteryVoltageMax = 349.0f;         // Max voltage (V)
    const float batteryInternalResistance = 0.08f;  // Internal resistance (Ω)
    const float maxMotorPower = 80000.0f;           // Max power (W)

    // State variables
    float speed_kmh = 0;
    float throttle = 0;
    float brake = 0;
    float batteryVoltage = batteryVoltageMax;
    float temp_motor = 55.0f;
    float temp_inverter = 74.0f;
    float temp_battery = 82.0f;

    // Driving cycle state machine
    enum Phase { ACCEL, HOLD_ACCEL, DECEL_FULL, DECEL_RELEASE };
    Phase phase = ACCEL;
    Phase previous_phase = ACCEL;
    uint32_t phase_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Simulation constants
    const float dt = 0.01f;                         // Time step (20ms)
    const float throttle_ramp = 1000.0f / 2.0f * dt; // Throttle ramp rate
    const float brake_ramp = 1000.0f / 3.0f * dt;    // Brake ramp rate
    const float temp_rate = 0.005f;                  // Temperature rise rate

    // Battery simulation
    int64_t batteryCapacity_mWh = carState.batteryCapacity_mWh;
    int64_t batteryEnergy_mWh = batteryCapacity_mWh;
    uint16_t soc_permil = 1000;

    for (;;) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        uint32_t elapsed = now - phase_start_time;

        // State machine for driving cycle
        switch (phase) {
            case ACCEL:
                throttle += throttle_ramp;
                if (throttle >= 1000) {
                    throttle = 1000;
                    phase = HOLD_ACCEL;
                }
                brake = 0;
                break;

            case HOLD_ACCEL:
                throttle = 1000;
                brake = 0;
                if (speed_kmh >= 150) {
                    phase = DECEL_FULL;
                    phase_start_time = now;
                    throttle = 0;
                    brake = 1000;
                }
                break;

            case DECEL_FULL:
                throttle = 0;
                brake = 1000;
                if (speed_kmh <= 25.0f) {
                    brake = 0;
                    phase = ACCEL;
                    phase_start_time = now;
                }
                break;

            case DECEL_RELEASE:
                throttle = 0;
                brake -= brake_ramp;
                if (brake <= 0) {
                    brake = 0;
                    phase = ACCEL;
                    phase_start_time = now;
                }
                break;
        }

        // Temperature simulation (slowly rising)
        temp_motor = fmin(temp_motor + temp_rate, 88.0f);
        temp_inverter = fmin(temp_inverter + temp_rate, 110.0f);
        temp_battery = fmin(temp_battery + temp_rate, 70.0f);

        // Physics simulation
        float speed_ms = speed_kmh / 3.6f;

        // Force calculations
        float F_aero = 0.5f * rho * Cx * S * speed_ms * speed_ms;  // Aerodynamic drag
        float F_roll = mass * g * Crr;                             // Rolling resistance
        float F_resist = F_aero + F_roll;                          // Total resistance

        float P_available = maxMotorPower * (throttle / 1000.0f);
        const float F_motor_max = 6000.0f;
        float F_motor = P_available / fmax(speed_ms, 1.0f);
        F_motor = fmin(F_motor, F_motor_max);
        float F_brake = (brake / 1000.0f) * 6000.0f;

        // Acceleration calculation (F = ma)
        float accel_ms2 = (F_motor - F_resist - F_brake) / mass;
        speed_ms += accel_ms2 * dt;
        speed_ms = fmax(speed_ms, 0.0f);
        speed_kmh = speed_ms * 3.6f;

        // Power and energy calculations
        float P_elec = 0.0f;
        float P_regen = 0.0f;

        if (brake > 0 && speed_ms > 1.0f) {
            // Regenerative braking
            P_regen = F_brake * speed_ms * 0.7f;  // 70% efficiency
            P_regen = fmin(P_regen, maxMotorPower * 0.5f);
        } else if (throttle > 0) {
            // Acceleration
            float P_mech = F_resist * speed_ms + fmax(accel_ms2 * mass * speed_ms, 0.0f);
            P_elec = P_mech / motorEff;
            P_elec = fmin(P_elec, maxMotorPower);
        } else {
            // Coasting
            float P_mech = F_resist * speed_ms;
            P_elec = P_mech / motorEff;
        }

        // Battery model with voltage drop
        float P_net = P_elec - P_regen;
        float I = P_net / batteryVoltage;
        float voltage_drop = I * batteryInternalResistance;
        batteryVoltage = batteryVoltageNominal - voltage_drop;
        batteryVoltage = fmin(fmax(batteryVoltage, 300.0f), 349.9f);
        int16_t I_cA = (int16_t)(I * 100.0f);

        // Energy tracking
        float energyUsed_mWh = (P_net * dt) / 3.6f;
        batteryEnergy_mWh -= (int64_t)energyUsed_mWh;
        batteryEnergy_mWh = fmax(batteryEnergy_mWh, 0LL);
        batteryEnergy_mWh = fmin(batteryEnergy_mWh, batteryCapacity_mWh);

        soc_permil = (uint16_t)((batteryEnergy_mWh * 1000LL) / batteryCapacity_mWh);
        soc_permil = fmin(soc_permil, 1000);

        // Send simulated CAN messages
        msg.identifier = 0x18010001;  // Throttle
        msg.data[0] = (uint8_t)((int) throttle & 0xFF);
        msg.data[1] = (uint8_t)(((int) throttle >> 8) & 0xFF);
        handleCANFrame(msg, carState);

        msg.identifier = 0x18010002;  // Brake
        msg.data[0] = (uint8_t)((int)brake & 0xFF);
        msg.data[1] = (uint8_t)(((int)brake >> 8) & 0xFF);
        handleCANFrame(msg, carState);

        msg.identifier = 0x18010003;  // Speed
        uint16_t speed_dkmh = (uint16_t)(speed_kmh * 10);
        msg.data[0] = speed_dkmh & 0xFF;
        msg.data[1] = (speed_dkmh >> 8) & 0xFF;
        handleCANFrame(msg, carState);

        msg.identifier = 0x18020001;  // Voltage
        uint16_t v_cV = (uint16_t)(batteryVoltage * 100);
        msg.data[0] = v_cV & 0xFF;
        msg.data[1] = (v_cV >> 8) & 0xFF;
        handleCANFrame(msg, carState);

        msg.identifier = 0x18020002;  // Current
        msg.data[0] = I_cA & 0xFF;
        msg.data[1] = (I_cA >> 8) & 0xFF;
        handleCANFrame(msg, carState);

        msg.identifier = 0x18030001;  // SOC
        msg.data[0] = soc_permil & 0xFF;
        msg.data[1] = (soc_permil >> 8) & 0xFF;
        handleCANFrame(msg, carState);

        msg.identifier = 0x18030002;  // Battery temp
        int16_t temp_battery_dC = (int16_t)(temp_battery * 10);
        msg.data[0] = temp_battery_dC & 0xFF;
        msg.data[1] = (temp_battery_dC >> 8) & 0xFF;
        handleCANFrame(msg, carState);

        msg.identifier = 0x18030003;  // Motor temp
        uint16_t temp_motor_dC = (uint16_t)(temp_motor * 10);
        msg.data[0] = temp_motor_dC & 0xFF;
        msg.data[1] = (temp_motor_dC >> 8) & 0xFF;
        handleCANFrame(msg, carState);

        msg.identifier = 0x18030004;  // Inverter temp
        uint16_t temp_inverter_dC = (uint16_t)(temp_inverter * 10);
        msg.data[0] = temp_inverter_dC & 0xFF;
        msg.data[1] = (temp_inverter_dC >> 8) & 0xFF;
        handleCANFrame(msg, carState);

        vTaskDelay(pdMS_TO_TICKS(10));  // 50 Hz update rate
    }
}


// ================================================================================
// RTOS TASKS (Parallel execution threads)
// ================================================================================

/**
 * CAN Task that handles CAN bus reception and storage in vehicle state
 * Priority: 2 | Core: 1 | Stack: 4KB
 */
void canTask(void *pvParameters) {
  for (;;) {
    can.loop();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

/**
 * Calculation Task that updates energy calculations every 100ms
 * Priority: 1 | Core: 1 | Stack: 4KB
 */
void calculationTask(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t frequency = pdMS_TO_TICKS(100);

  for (;;) {
    updateEnergyAndDelta();
    vTaskDelayUntil(&lastWakeTime, frequency);
  }
}

/**
 * Logger Task that stores all the CAN Frames received onto a SD Card
 * Priority: 0 | Core: 0 | Stack: 4KB
 */
void loggerTask(void *pvParameters) {
  for (;;) {
    canLogger.loop();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

/**
 * UI Task that updates all display elements every 100ms
 * Priority: 1 | Core: 0 | Stack: 8KB
 */
void uiTask(void *pvParameters) {
  for (;;) {
    car_ui.updateSpeedUI();
    car_ui.updateThrottleUI();
    car_ui.updateBrakeUI();
    car_ui.updateVoltageUI();
    car_ui.updateAmperageUI();
    car_ui.updateBatteryLevelUI();
    car_ui.updateBatteryTempUI();
    car_ui.updateEngineTempUI();
    car_ui.updateInverterTempUI();
    car_ui.updateDeltaUI();
    car_ui.updateConsumptionUI();
    car_ui.updateDistanceUI();
    lv_timer_handler();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/**
 * Setup function that runs once at startup
 * Initializes display, LVGL, CAN, and creates all parallel tasks (RTOS)
 */
void setup()
{
    Serial.begin(9600);

    // Initialize display hardware
    lcd.begin();
    lcd.fillScreen(TFT_BLACK);

    // Initialize LVGL
    lv_init();
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, 120 * 800);
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = 800;
    disp_drv.ver_res = 480;
    disp_drv.sw_rotate = 1;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    // Put On the Backlight of the screen.
    #ifdef TFT_BL
      pinMode(TFT_BL, OUTPUT);
      digitalWrite(TFT_BL, HIGH);
    #endif

    // Initialize UI and Vehicle State
    ui_init();
    can.state().init();

    // Initialize CAN Logger (SD Card)
    if (canLogger.begin()) {
        canLogger.startLogging();
    }

    //   if (!can.begin()) {
    //     Serial.println("CAN init failed");
    //     while (1) {
    //       delay(1000);
    //     }
    //   }

    // CAN message Handler. Communicates with Vehicule State
    can.onFrame(handleCANFrame);

    // Assigns Parallel Tasks
    // xTaskCreatePinnedToCore(canTask, "CAN Task", 4096, NULL, 2, NULL, 1); // Handle reception of CAN Frames and update the state of the car
    xTaskCreatePinnedToCore(fakeCANTask, "CAN Task", 4096, NULL, 2, NULL, 1); // Fake CAN Frames to simulate reception from the car (DEBUG USE ONLY)
    xTaskCreatePinnedToCore(calculationTask, "Calc Task", 4096, NULL, 1, NULL, 1); // All the calculations to retrieve datas that are not provided by CAN Bus
    xTaskCreatePinnedToCore(uiTask, "UI Task", 8192, NULL, 1, NULL, 0); // Update the display with new Vehicule State
    xTaskCreatePinnedToCore(loggerTask, "Logger Task", 4096, NULL, 0, NULL, 0); // Store all the CAN Frames received in CSV
}

/**
 * Main Loop (Mandatory). It is empty because all work is done in RTOS tasks.
 */
void loop(){}
