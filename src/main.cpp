#include <Arduino.h>
#include <lvgl.h>
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <esp_heap_caps.h>

#include "ui/ui.h"
#include "CANSocket.h"
#include "CarState.hpp"
#include "CarStateUI.hpp"
#include "CANLogger.h"

#define TFT_BL 2

constexpr int32_t ALPHA_FIXED = 154;
constexpr int32_t INV_ALPHA_FIXED = 870;
constexpr int32_t MIN_DISTANCE_M = 500;
constexpr int32_t DEFAULT_CONSUMPTION = 41000;
constexpr int32_t SPEED_THRESHOLD = 10;
constexpr int32_t MIN_CONSUMPTION = 10;
constexpr int32_t DELTA_CLAMP_M = 99999;

// ---------------------- LVGL Mutex ----------------------
SemaphoreHandle_t lvgl_mutex;

#define LVGL_LOCK() xSemaphoreTake(lvgl_mutex, portMAX_DELAY)
#define LVGL_UNLOCK() xSemaphoreGive(lvgl_mutex)

// ---------------------- Setup LGFX ----------------------
class LGFX : public lgfx::LGFX_Device {
public:
  lgfx::Bus_RGB _bus_instance;
  lgfx::Panel_RGB _panel_instance;

  LGFX(void) {
    auto cfg = _bus_instance.config();
    cfg.panel = &_panel_instance;

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

    cfg.pin_henable = GPIO_NUM_5;
    cfg.pin_vsync = GPIO_NUM_3;
    cfg.pin_hsync = GPIO_NUM_46;
    cfg.pin_pclk = GPIO_NUM_7;

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

static lv_disp_draw_buf_t draw_buf;
static lv_color_t disp_draw_buf[800*80];
static lv_disp_drv_t disp_drv;

// SEULE MODIFICATION : Flush amélioré pour réduire le tearing
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  lcd.pushImageDMA(area->x1, area->y1, w, h, (lgfx::rgb565_t *)&color_p->full);
  lv_disp_flush_ready(disp);
}

CANSocket can;
CarStateUI car_ui(can.state());
static int32_t filteredConsumption_cWhKm = 0;

void handleCANFrame(const twai_message_t &msg, CarState &carState) {
    uint16_t value_u16;
    int16_t value_i16;

    switch (msg.identifier) {
        case 0x18010001:
            value_u16 = (msg.data[1] << 8) | msg.data[0];
            carState.setThrottle(value_u16);
            break;
        case 0x18010002:
            value_u16 = (msg.data[1] << 8) | msg.data[0];
            carState.setBrake(value_u16);
            break;
        case 0x18010003:
            value_u16 = (msg.data[1] << 8) | msg.data[0];
            carState.setSpeed(value_u16);
            break;
        case 0x18020001:
            value_u16 = (msg.data[1] << 8) | msg.data[0];
            carState.setBatteryVoltage(value_u16);
            break;
        case 0x18020002:
            value_i16 = (int16_t)((msg.data[1] << 8) | msg.data[0]);
            carState.setBatteryAmperage(value_i16);
            {
                uint16_t voltage_cV = carState.getBatteryVoltage_cV();
                int32_t powerOutput_mW = ((int64_t)voltage_cV * value_i16) * 0.1;
                carState.setBatteryPowerOutput(powerOutput_mW);
            }
            break;
        case 0x18030001:
            value_u16 = (msg.data[1] << 8) | msg.data[0];
            carState.setBatterySOC(value_u16);
            {
                int64_t energyAvailable_mWh = (carState.batteryCapacity_mWh * value_u16) * 0.001f;
                carState.setBatteryEnergyAvailable(energyAvailable_mWh);
            }
            break;
        case 0x18030002:
            value_i16 = (int16_t)((msg.data[1] << 8) | msg.data[0]);
            carState.setBatteryTemperature(value_i16);
            break;
        case 0x18030003:
            value_u16 = (msg.data[1] << 8) | msg.data[0];
            carState.setEngineTemperature(value_u16);
            break;
        case 0x18030004:
            value_u16 = (msg.data[1] << 8) | msg.data[0];
            carState.setInverterTemperature(value_u16);
            break;
    }
    carState.updateTimestamp();
}

void updateEnergyAndDelta() {
    CarState& carState = can.state();
    uint32_t now = millis();
    uint32_t lastCalc = carState.getLastCalcMs();

    if (lastCalc == 0) {
        carState.setLastCalcMs(now);
        return;
    }

    uint32_t deltaTime_ms = now - lastCalc;
    uint16_t speed_dkmh = carState.getSpeedRaw();
    int32_t powerOutput_mW = carState.getBatteryPowerOutput();
    int64_t energyAvailable_mWh = carState.getBatteryEnergyAvailable();

    int32_t distance_m = 0;
    if (speed_dkmh > SPEED_THRESHOLD) {
        distance_m = ((int64_t)speed_dkmh * deltaTime_ms) / 36000;
    }

    int64_t energyIncrement_mWh = ((int64_t)powerOutput_mW * deltaTime_ms) / 3600000;
    int64_t energyConsumed_mWh = carState.getEnergyConsumed();
    int64_t energyRegenerated_mWh = carState.getEnergyRegenerated();
    int32_t tripDistance_m = carState.getTripDistance();

    if (energyIncrement_mWh > 0) {
        energyConsumed_mWh += energyIncrement_mWh;
        carState.setEnergyConsumed(energyConsumed_mWh);
    } else if (energyIncrement_mWh < 0) {
        energyRegenerated_mWh += (-energyIncrement_mWh);
        carState.setEnergyRegenerated(energyRegenerated_mWh);
    }

    tripDistance_m += distance_m;
    carState.setTripDistance(tripDistance_m);

    int32_t currentConsumption_cWhKm;
    if (tripDistance_m >= MIN_DISTANCE_M) {
        int64_t energyNet_mWh = energyConsumed_mWh - energyRegenerated_mWh;
        currentConsumption_cWhKm = (int32_t)((energyNet_mWh * 100) / tripDistance_m);
    } else {
        currentConsumption_cWhKm = DEFAULT_CONSUMPTION;
    }

    if (filteredConsumption_cWhKm == 0) {
        filteredConsumption_cWhKm = currentConsumption_cWhKm;
    } else {
        filteredConsumption_cWhKm = (ALPHA_FIXED * currentConsumption_cWhKm + INV_ALPHA_FIXED * filteredConsumption_cWhKm) >> 10;
    }

    carState.setEnergyConsumption(filteredConsumption_cWhKm);

    int32_t rangeRemaining_m;
    if (filteredConsumption_cWhKm > MIN_CONSUMPTION) {
        rangeRemaining_m = (int32_t)((energyAvailable_mWh * 100) / filteredConsumption_cWhKm);
    } else {
        rangeRemaining_m = 999999;
    }
    carState.setRangeRemaining(rangeRemaining_m);

    int32_t distanceRemaining_m = carState.raceDistance_m - tripDistance_m;
    int32_t delta_m = rangeRemaining_m - distanceRemaining_m;

    if (delta_m > DELTA_CLAMP_M) delta_m = DELTA_CLAMP_M;
    if (delta_m < -DELTA_CLAMP_M) delta_m = -DELTA_CLAMP_M;

    carState.setEnergyDelta(delta_m * 100);
    carState.setLastCalcMs(now);
}

void canTask(void *pvParameters) {
  for (;;) {
    can.loop();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void calculationTask(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t frequency = pdMS_TO_TICKS(100);

  for (;;) {
    updateEnergyAndDelta();
    vTaskDelayUntil(&lastWakeTime, frequency);
  }
}

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

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void fakeCANTask(void *pvParameters) {
    twai_message_t msg;
    msg.extd = 1;
    msg.data_length_code = 2;
    CarState& carState = can.state();

    const float mass = 350.0f;
    const float Cx = 0.18f;
    const float S = 0.9f;
    const float rho = 1.225f;
    const float Crr = 0.008f;
    const float g = 9.81f;
    const float motorEff = 0.92f;
    const float batteryVoltageNominal = 340.0f;
    const float batteryVoltageMax = 349.0f;
    const float batteryInternalResistance = 0.08f;
    const float maxMotorPower = 80000.0f;

    float speed_kmh = 0;
    float throttle = 0;
    float brake = 0;
    float batteryVoltage = batteryVoltageMax;
    float temp_motor = 55.0f;
    float temp_inverter = 74.0f;
    float temp_battery = 82.0f;

    enum Phase { ACCEL, HOLD_ACCEL, DECEL_FULL, DECEL_RELEASE };
    Phase phase = ACCEL;
    Phase previous_phase = ACCEL;
    uint32_t phase_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    const float dt = 0.02f;
    const float throttle_ramp = 1000.0f / 2.0f * dt;
    const float brake_ramp = 1000.0f / 3.0f * dt;
    const float temp_rate = 0.005f;

    int64_t batteryCapacity_mWh = carState.batteryCapacity_mWh;
    int64_t batteryEnergy_mWh = batteryCapacity_mWh;
    uint16_t soc_permil = 1000;

    for (;;) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        uint32_t elapsed = now - phase_start_time;

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

        temp_motor = fmin(temp_motor + temp_rate, 115.0f);
        temp_inverter = fmin(temp_inverter + temp_rate, 115.0f);
        temp_battery = fmin(temp_battery + temp_rate, 75.0f);

        float speed_ms = speed_kmh / 3.6f;
        float F_aero = 0.5f * rho * Cx * S * speed_ms * speed_ms;
        float F_roll = mass * g * Crr;
        float F_resist = F_aero + F_roll;

        float P_available = maxMotorPower * (throttle / 1000.0f);
        const float F_motor_max = 6000.0f;
        float F_motor = P_available / fmax(speed_ms, 1.0f);
        F_motor = fmin(F_motor, F_motor_max);
        float F_brake = (brake / 1000.0f) * 6000.0f;

        float accel_ms2 = (F_motor - F_resist - F_brake) / mass;
        speed_ms += accel_ms2 * dt;
        speed_ms = fmax(speed_ms, 0.0f);
        speed_kmh = speed_ms * 3.6f;

        float P_elec = 0.0f;
        float P_regen = 0.0f;

        if (brake > 0 && speed_ms > 1.0f) {
            P_regen = F_brake * speed_ms * 0.7f;
            P_regen = fmin(P_regen, maxMotorPower * 0.5f);
        } else if (throttle > 0) {
            float P_mech = F_resist * speed_ms + fmax(accel_ms2 * mass * speed_ms, 0.0f);
            P_elec = P_mech / motorEff;
            P_elec = fmin(P_elec, maxMotorPower);
        } else {
            float P_mech = F_resist * speed_ms;
            P_elec = P_mech / motorEff;
        }

        float P_net = P_elec - P_regen;
        float I = P_net / batteryVoltage;
        float voltage_drop = I * batteryInternalResistance;
        batteryVoltage = batteryVoltageNominal - voltage_drop;
        batteryVoltage = fmin(fmax(batteryVoltage, 300.0f), 349.9f);
        int16_t I_cA = (int16_t)(I * 100.0f);

        float energyUsed_mWh = (P_net * dt) / 3.6f;
        batteryEnergy_mWh -= (int64_t)energyUsed_mWh;
        batteryEnergy_mWh = fmax(batteryEnergy_mWh, 0LL);
        batteryEnergy_mWh = fmin(batteryEnergy_mWh, batteryCapacity_mWh);

        soc_permil = (uint16_t)((batteryEnergy_mWh * 1000LL) / batteryCapacity_mWh);
        soc_permil = fmin(soc_permil, 1000);

        msg.identifier = 0x18010001;
        msg.data[0] = (uint8_t)((int) throttle & 0xFF);
        msg.data[1] = (uint8_t)(((int) throttle >> 8) & 0xFF);
        handleCANFrame(msg, carState);

        msg.identifier = 0x18010002;
        msg.data[0] = (uint8_t)((int)brake & 0xFF);
        msg.data[1] = (uint8_t)(((int)brake >> 8) & 0xFF);
        handleCANFrame(msg, carState);

        msg.identifier = 0x18010003;
        uint16_t speed_dkmh = (uint16_t)(speed_kmh * 10);
        msg.data[0] = speed_dkmh & 0xFF;
        msg.data[1] = (speed_dkmh >> 8) & 0xFF;
        handleCANFrame(msg, carState);

        msg.identifier = 0x18020001;
        uint16_t v_cV = (uint16_t)(batteryVoltage * 100);
        msg.data[0] = v_cV & 0xFF;
        msg.data[1] = (v_cV >> 8) & 0xFF;
        handleCANFrame(msg, carState);

        msg.identifier = 0x18020002;
        msg.data[0] = I_cA & 0xFF;
        msg.data[1] = (I_cA >> 8) & 0xFF;
        handleCANFrame(msg, carState);

        msg.identifier = 0x18030001;
        msg.data[0] = soc_permil & 0xFF;
        msg.data[1] = (soc_permil >> 8) & 0xFF;
        handleCANFrame(msg, carState);

        msg.identifier = 0x18030002;
        int16_t temp_battery_dC = (int16_t)(temp_battery * 10);
        msg.data[0] = temp_battery_dC & 0xFF;
        msg.data[1] = (temp_battery_dC >> 8) & 0xFF;
        handleCANFrame(msg, carState);

        msg.identifier = 0x18030003;
        uint16_t temp_motor_dC = (uint16_t)(temp_motor * 10);
        msg.data[0] = temp_motor_dC & 0xFF;
        msg.data[1] = (temp_motor_dC >> 8) & 0xFF;
        handleCANFrame(msg, carState);

        msg.identifier = 0x18030004;
        uint16_t temp_inverter_dC = (uint16_t)(temp_inverter * 10);
        msg.data[0] = temp_inverter_dC & 0xFF;
        msg.data[1] = (temp_inverter_dC >> 8) & 0xFF;
        handleCANFrame(msg, carState);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void setup()
{
    Serial.begin(9600);

    // lcd.init();
    lcd.begin();
    // lcd.setBrightness(255);
    // lcd.setColorDepth(16);
    lcd.fillScreen(TFT_BLACK);
    lv_init();
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, 80 * 800);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = 800;
    disp_drv.ver_res = 480;
    disp_drv.sw_rotate = 1;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    #ifdef TFT_BL
      pinMode(TFT_BL, OUTPUT);
      digitalWrite(TFT_BL, HIGH);
    #endif

    ui_init();
    can.state().init();

    can.onFrame(handleCANFrame);

    xTaskCreatePinnedToCore(fakeCANTask, "CAN Task", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(calculationTask, "Calc Task", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(uiTask, "UI Task", 8192, NULL, 1, NULL, 0);
}

void loop(){}
