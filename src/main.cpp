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
#include "Debug.hpp"             // Debug logging macros

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
    cfg.pin_d0 = GPIO_NUM_14; //B3
    cfg.pin_d1 = GPIO_NUM_38; //B4
    cfg.pin_d2 = GPIO_NUM_18; //B5
    cfg.pin_d3 = GPIO_NUM_17; //B6
    cfg.pin_d4 = GPIO_NUM_10; //B7
    cfg.pin_d5 = GPIO_NUM_39; //G2
    cfg.pin_d6 = GPIO_NUM_0;  //G3
    cfg.pin_d7 = GPIO_NUM_45; //G4
    cfg.pin_d8 = GPIO_NUM_48; //G5
    cfg.pin_d9 = GPIO_NUM_47; //G6
    cfg.pin_d10 = GPIO_NUM_21; //G7
    cfg.pin_d11 = GPIO_NUM_1; //R3
    cfg.pin_d12 = GPIO_NUM_2; //R4
    cfg.pin_d13 = GPIO_NUM_42; //R5
    cfg.pin_d14 = GPIO_NUM_41; //R6
    cfg.pin_d15 = GPIO_NUM_40; //R7

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
 * 0x18020002 - Battery amperage
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
            LOG_EVERY_MS(500, LOG_I(TAG_CAN, "Throttle raw=%u  display=%u%%", value_u16, value_u16 / 10));
            break;

        case 0x18010002:  // Brake
            value_u16 = (msg.data[1] << 8) | msg.data[0];
            carState.setBrake(value_u16);
            LOG_EVERY_MS(500, LOG_I(TAG_CAN, "Brake    raw=%u  display=%u%%", value_u16, value_u16 / 10));
            break;

        case 0x18010003:  // Speed
            value_u16 = (msg.data[1] << 8) | msg.data[0];
            carState.setSpeed(value_u16);
            LOG_EVERY_MS(500, LOG_I(TAG_CAN, "Speed    raw=%u dkm/h  display=%u km/h", value_u16, value_u16 / 10));
            break;

        case 0x18020001:  // Battery voltage
            value_u16 = (msg.data[1] << 8) | msg.data[0];
            carState.setBatteryVoltage(value_u16);
            LOG_EVERY_MS(1000, LOG_I(TAG_CAN, "Voltage  raw=%u cV  display=%.2f V", value_u16, value_u16 / 100.0f));
            break;

        case 0x18020002:  // Battery amperage
            value_i16 = (int16_t)((msg.data[1] << 8) | msg.data[0]);
            carState.setBatteryAmperage(value_i16);
            // Calculated power output (P = V * I)
            {
                uint16_t voltage_cV = carState.getBatteryVoltage_cV();
                int32_t powerOutput_mW = ((int64_t)voltage_cV * value_i16) * 0.1;
                carState.setBatteryPowerOutput(powerOutput_mW);
                LOG_EVERY_MS(1000, LOG_I(TAG_CAN, "Amperage raw=%d cA  display=%.2f A  power=%ld mW",
                    value_i16, value_i16 / 100.0f, powerOutput_mW));
            }
            break;

        case 0x18030001:  // Battery SOC
            value_u16 = (msg.data[1] << 8) | msg.data[0];
            carState.setBatterySOC(value_u16);
            // Calculated available energy
            {
                int64_t energyAvailable_mWh = (carState.batteryCapacity_mWh * value_u16) * 0.001f;
                carState.setBatteryEnergyAvailable(energyAvailable_mWh);
                LOG_EVERY_MS(2000, LOG_I(TAG_CAN, "SOC      raw=%u permil  display=%u%%  energy=%lld mWh",
                    value_u16, value_u16 / 10, energyAvailable_mWh));
            }
            break;

        case 0x18030002:  // Battery temperature
            value_i16 = (int16_t)((msg.data[1] << 8) | msg.data[0]);
            carState.setBatteryTemperature(value_i16);
            LOG_EVERY_MS(2000, LOG_I(TAG_CAN, "BattTemp raw=%d dC  display=%d degC", value_i16, value_i16 / 10));
            break;

        case 0x18030003:  // Motor temperature
            value_u16 = (msg.data[1] << 8) | msg.data[0];
            carState.setEngineTemperature(value_u16);
            LOG_EVERY_MS(2000, LOG_I(TAG_CAN, "EngTemp  raw=%u dC  display=%u degC", value_u16, value_u16 / 10));
            break;

        case 0x18030004:  // Inverter temperature
            value_u16 = (msg.data[1] << 8) | msg.data[0];
            carState.setInverterTemperature(value_u16);
            LOG_EVERY_MS(2000, LOG_I(TAG_CAN, "InvTemp  raw=%u dC  display=%u degC", value_u16, value_u16 / 10));
            break;

        default:
            LOG_W(TAG_CAN, "Unknown CAN ID: 0x%08X  DLC=%u", msg.identifier, msg.data_length_code);
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
        LOG_I(TAG_CALC, "First run - initializing lastCalcMs to %lu", now);
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
    } else {
        LOG_EVERY_MS(5000, LOG_I(TAG_CALC, "Car stopped (speed_dkmh=%u <= threshold=%d), no distance added", speed_dkmh, SPEED_THRESHOLD));
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
        LOG_W(TAG_CALC, "filteredConsumption_cWhKm=%ld <= MIN_CONSUMPTION=%d, range set to max", filteredConsumption_cWhKm, MIN_CONSUMPTION);
        rangeRemaining_m = 999999;
    }
    carState.setRangeRemaining(rangeRemaining_m);

    // Calculate delta (how much further we can go than race distance)
    int32_t distanceRemaining_m = carState.raceDistance_m - tripDistance_m;
    int32_t delta_m = rangeRemaining_m - distanceRemaining_m;

    // Clamp delta to prevent overflow
    if (delta_m > DELTA_CLAMP_M) {
        LOG_W(TAG_CALC, "delta_m clamped to +MAX (was %ld)", delta_m);
        delta_m = DELTA_CLAMP_M;
    }
    if (delta_m < -DELTA_CLAMP_M) {
        LOG_W(TAG_CALC, "delta_m clamped to -MAX (was %ld)", delta_m);
        delta_m = -DELTA_CLAMP_M;
    }

    LOG_EVERY_MS(5000, LOG_I(TAG_CALC,
        "dt=%lums  dist=%ldm  trip=%ldm  consumed=%lldmWh  regen=%lldmWh  "
        "consump=%ldcWh/km  filtered=%ldcWh/km  range=%ldm  distLeft=%ldm  delta=%ldm",
        (unsigned long)deltaTime_ms, distance_m, tripDistance_m,
        energyConsumed_mWh, energyRegenerated_mWh,
        currentConsumption_cWhKm, filteredConsumption_cWhKm,
        rangeRemaining_m, distanceRemaining_m, delta_m));

    carState.setEnergyDelta(delta_m * 100);
    carState.setLastCalcMs(now);
}

// ================================================================================
// RTOS TASKS (Parallel execution threads)
// ================================================================================

/**
 * CAN Task that handles CAN bus reception and storage in vehicle state
 * Priority: 2 | Core: 1 | Stack: 4KB
 */
void canTask(void *pvParameters) {
  LOG_I(TAG_CAN, "CAN task started on core %d", xPortGetCoreID());
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
  LOG_I(TAG_CALC, "Calculation task started on core %d", xPortGetCoreID());
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
  LOG_I(TAG_LOGGER, "Logger task started on core %d", xPortGetCoreID());
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
  LOG_I(TAG_UI, "UI task started on core %d", xPortGetCoreID());
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
    LOG_EVERY_MS(10000, LOG_I(TAG_UI, "Heap free: %u bytes", esp_get_free_heap_size()));
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
    delay(200); // Give serial time to come up before first logs

    LOG_I(TAG_MAIN, "========================================");
    LOG_I(TAG_MAIN, " Sigma Racing Dashboard - Booting...");
    LOG_I(TAG_MAIN, "========================================");
    LOG_I(TAG_MAIN, "Free heap at start: %u bytes", esp_get_free_heap_size());

    // Initialize display hardware
    LOG_I(TAG_MAIN, "Initializing display...");
    lcd.begin();
    lcd.fillScreen(TFT_BLACK);
    LOG_I(TAG_MAIN, "Display initialized OK");

    // Initialize LVGL
    LOG_I(TAG_MAIN, "Initializing LVGL...");
    lv_init();
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, 120 * 800);
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = 800;
    disp_drv.ver_res = 480;
    disp_drv.sw_rotate = 1;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);
    LOG_I(TAG_MAIN, "LVGL initialized OK (800x480)");

    // Put On the Backlight of the screen.
    #ifdef TFT_BL
      pinMode(TFT_BL, OUTPUT);
      digitalWrite(TFT_BL, HIGH);
      LOG_I(TAG_MAIN, "Backlight ON (pin %d)", TFT_BL);
    #endif

    // Initialize UI and Vehicle State
    LOG_I(TAG_MAIN, "Initializing UI...");
    ui_init();
    LOG_I(TAG_MAIN, "UI initialized OK");

    LOG_I(TAG_MAIN, "Initializing CarState...");
    can.state().init();
    LOG_I(TAG_MAIN, "CarState initialized OK");

    // Initialize CAN Logger (SD Card)
    LOG_I(TAG_LOGGER, "Initializing SD card logger...");
    if (canLogger.begin()) {
        LOG_I(TAG_LOGGER, "SD card initialized OK - starting logging");
        canLogger.startLogging();
        LOG_I(TAG_LOGGER, "Logging started");
    } else {
        LOG_E(TAG_LOGGER, "SD card initialization FAILED - logging disabled");
    }

    LOG_I(TAG_CAN, "Initializing CAN bus (TX=GPIO20, RX=GPIO19, 500kbps)...");
    if (!can.begin()) {
        LOG_E(TAG_CAN, "CAN bus initialization FAILED - halting");
        Serial.println("CAN init failed");
        while (1) {
            delay(1000);
        }
    }
    LOG_I(TAG_CAN, "CAN bus initialized OK");

    // CAN message Handler. Communicates with Vehicule State
    can.onFrame(handleCANFrame);
    LOG_I(TAG_CAN, "CAN frame handler registered");

    // Assigns Parallel Tasks
    LOG_I(TAG_MAIN, "Creating RTOS tasks...");
    xTaskCreatePinnedToCore(canTask,         "CAN Task",    4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(calculationTask, "Calc Task",   4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(uiTask,          "UI Task",     8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(loggerTask,      "Logger Task", 4096, NULL, 0, NULL, 0);
    LOG_I(TAG_MAIN, "All tasks created - setup complete");
    LOG_I(TAG_MAIN, "Free heap after setup: %u bytes", esp_get_free_heap_size());
}

/**
 * Main Loop (Mandatory). It is empty because all work is done in RTOS tasks.
 */
void loop(){}
