#include "CarStateUI.hpp"
#include "lvgl.h"
#include "ui/screens/ui_Main_Screen.h"
#include <cmath>

// UI refresh rates (value thresholds and time intervals)
CarStateUI::CarStateUI(CarState &state) :
    carState(state),
    voltageField{-1, 0, 200, 1.0f}, // refresh voltage label every 200 ms or 1V.
    amperageField{-1, 0, 200, 1.0f}, // refresh amperage label every 200 ms or 1A.
    batteryLevelField{-1, 0, 2000, 1.0f}, // refresh battery SOC label every 2s or 1%.
    batteryTempField{-1, 0, 5000, 1.0f}, // refresh battery temperature label every 5s or 1°C.
    throttleField{-1, 0, 200, 1.0f}, // refresh throttle potentiometer label every 200ms or 1%.
    brakeField{-1, 0, 200, 1.0f}, // refresh brake potentiometer label every 200ms or 1%.
    speedField{-1, 0, 200, 1.0f}, // refresh speed label every 200ms or 1kmh.
    tripMeterField{-1, 0, 1000, 0.095f}, // refresh tripmeter label every second or 0.095km.
    deltaField{-1, 0, 500, 0.01f}, // refresh trip delta every 500ms or +-0.01km .
    consumField{-1, 0, 1000, 0.1f}, // refresh consumption label every second or 0.1Wh/km.
    engineTempField{-1, 0, 2000, 1.0f}, // refresh engine temperature label every 2s or 1°C.
    inverterTempField{-1, 0, 2000, 1.0f} // refresh inverter temperature label every 2s or 1°C.
{}

// update speed label on the screen
void CarStateUI::updateSpeedUI() {
  uint32_t now = millis();
  if (now - speedField.lastUpdateMs < speedField.refreshIntervalMs) {
    return;
  }

  int currentSpeed = carState.getSpeed();

  if (abs(currentSpeed - speedField.lastValue) < speedField.valueThreshold) {
    return;
  }
  lv_label_set_text_fmt(ui_speedlabel, "%d", currentSpeed);
  speedField.lastValue = currentSpeed;
  speedField.lastUpdateMs = now;
}

// update throttle potentiometer on the screen
void CarStateUI::updateThrottleUI() {
  uint32_t now = millis();
  if (now - throttleField.lastUpdateMs < throttleField.refreshIntervalMs) {
    return;
  }
  int throttlePosition = carState.getThrottle();

  if (abs(throttlePosition - throttleField.lastValue) < throttleField.valueThreshold) {
    return;
  }

  lv_bar_set_value(ui_throttlevisu, throttlePosition, LV_ANIM_ON);
  throttleField.lastValue = throttlePosition;
  throttleField.lastUpdateMs = now;
}

// update brake potentiometer on the screen
void CarStateUI::updateBrakeUI() {
  uint32_t now = millis();
  if (now - brakeField.lastUpdateMs < brakeField.refreshIntervalMs) {
    return;
  }

  int brakePosition = carState.getBrake();

  if (abs(brakePosition - brakeField.lastValue) < brakeField.valueThreshold) {
    return;
  }

  lv_bar_set_value(ui_brakevisu, brakePosition, LV_ANIM_ON);
  brakeField.lastValue = brakePosition;
  brakeField.lastUpdateMs = now;
}

// update voltage field on the screen
void CarStateUI::updateVoltageUI() {
  uint32_t now = millis();
  if (now - voltageField.lastUpdateMs < voltageField.refreshIntervalMs) {
    return;
  }

  float voltage = carState.getBatteryVoltage();

  int voltage_int = (int)voltage;
  if (abs(voltage_int - voltageField.lastValue) < voltageField.valueThreshold) {
    return;
  }

  lv_bar_set_value(ui_voltage, voltage_int, LV_ANIM_ON);
  lv_label_set_text_fmt(ui_voltagelabel, "%d", voltage_int);
  voltageField.lastValue = voltage_int;
  voltageField.lastUpdateMs = now;
}

// update amperage field on the screen
void CarStateUI::updateAmperageUI() {
  uint32_t now = millis();
  if (now - amperageField.lastUpdateMs < amperageField.refreshIntervalMs) {
    return;
  }

  float amperage = carState.getBatteryAmperage();

  int amperage_int = (int)amperage;
  if (abs(amperage_int - amperageField.lastValue) < amperageField.valueThreshold) {
    return;
  }

  if(amperageField.lastValue*amperage_int < 0 || amperageField.lastUpdateMs==0){
      if(amperage_int < 0){
          lv_bar_set_value(ui_amperage, 0, LV_ANIM_ON);
          lv_bar_set_value(ui_amperage1, 300 + amperage_int, LV_ANIM_ON);
      }else{
          lv_bar_set_value(ui_amperage, amperage_int, LV_ANIM_ON);
          lv_bar_set_value(ui_amperage1, 300, LV_ANIM_ON);
      }
  }else{
      if(amperage_int < 0){
          lv_bar_set_value(ui_amperage1, 300 + amperage_int, LV_ANIM_ON);
      }else{
          lv_bar_set_value(ui_amperage, amperage_int, LV_ANIM_ON);
      }
  }

  lv_label_set_text_fmt(ui_amperagelabel, "%d", amperage_int);
  amperageField.lastValue = amperage_int;
  amperageField.lastUpdateMs = now;
}

// update battery SOC on the screen
void CarStateUI::updateBatteryLevelUI() {
  uint32_t now = millis();
  if (now - batteryLevelField.lastUpdateMs < batteryLevelField.refreshIntervalMs) {
    return;
  }

  int batteryLevel = carState.getBatterySOC();

  if (abs(batteryLevel - batteryLevelField.lastValue) < batteryLevelField.valueThreshold) {
    return;
  }

  lv_bar_set_value(ui_batterylevel, batteryLevel, LV_ANIM_ON);
  lv_label_set_text_fmt(ui_batterylevellabel, "%d", batteryLevel);
  updateBatteryLevelColor(batteryLevel);
  batteryLevelField.lastValue = batteryLevel;
  batteryLevelField.lastUpdateMs = now;
}

// update battery temperature on the screen
void CarStateUI::updateBatteryTempUI() {
  uint32_t now = millis();
  if (now - batteryTempField.lastUpdateMs < batteryTempField.refreshIntervalMs) {
    return;
  }

  int batteryTemperature = carState.getBatteryTemperature();

  if (abs(batteryTemperature - batteryTempField.lastValue) < batteryTempField.valueThreshold) {
    return;
  }

  lv_arc_set_value(ui_batterytemp, batteryTemperature);
  lv_label_set_text_fmt(ui_batterytemplabel, "%d", batteryTemperature);
  updateBatteryTempColor(batteryTemperature);
  batteryTempField.lastValue = batteryTemperature;
  batteryTempField.lastUpdateMs = now;
}

// update engine temperature on the screen
void CarStateUI::updateEngineTempUI() {
  uint32_t now = millis();
  if (now - engineTempField.lastUpdateMs < engineTempField.refreshIntervalMs) {
    return;
  }

  int engineTemp = carState.getEngineTemperature();

  if (abs(engineTemp - engineTempField.lastValue) < engineTempField.valueThreshold) {
    return;
  }

  lv_label_set_text_fmt(ui_enginetemplabel, "%d", engineTemp);
  updateEngineTempColor(engineTemp);
  engineTempField.lastValue = engineTemp;
  engineTempField.lastUpdateMs = now;
}

// update inverter temperature on the screen
void CarStateUI::updateInverterTempUI() {
  uint32_t now = millis();
  if (now - inverterTempField.lastUpdateMs < inverterTempField.refreshIntervalMs) {
    return;
  }

  int inverterTemp = carState.getInverterTemperature();

  if (abs(inverterTemp - inverterTempField.lastValue) < inverterTempField.valueThreshold) {
    return;
  }

  lv_label_set_text_fmt(ui_invertertemplabel, "%d", inverterTemp);
  updateInverterTempColor(inverterTemp);
  inverterTempField.lastValue = inverterTemp;
  inverterTempField.lastUpdateMs = now;
}

// update trip meter label on the screen
void CarStateUI::updateDistanceUI() {
  uint32_t now = millis();
  if (now - tripMeterField.lastUpdateMs < tripMeterField.refreshIntervalMs) {
    return;
  }

  int32_t distance_m = carState.getTripDistance();
  float distance_km = distance_m / 1000.0f;

  if (fabs(distance_km - tripMeterField.lastValue) < tripMeterField.valueThreshold) {
    return;
  }

  lv_label_set_text_fmt(ui_tripmeter, "%d.%d",
                        (int)distance_km,
                        (int)((distance_km - (int)distance_km) * 10 + 0.5f) % 10);
  tripMeterField.lastValue = distance_km;
  tripMeterField.lastUpdateMs = now;
}

// update delta label on the screen
void CarStateUI::updateDeltaUI() {
  uint32_t now = millis();
  if (now - deltaField.lastUpdateMs < deltaField.refreshIntervalMs) {
    return;
  }

  float delta = carState.getEnergyDelta();

  if (fabs(delta - deltaField.lastValue) < deltaField.valueThreshold) {
    return;
  }

  if (delta * deltaField.lastValue <= 0) {
    lv_label_set_text_fmt(ui_deltasignlabel, (delta < 0) ? "-" : "+");
  }


  // delta est en mètres, convertir en km avec 2 décimales
  float delta_km = delta / 1000.0f;
  int delta_int = abs((int)delta_km);
  int delta_dec = (int)(fabs(delta_km - delta_int) * 100.0f + 0.5f) % 100;


  lv_label_set_text_fmt(ui_deltalabel, "%d.%02d", delta_int, delta_dec);
  lv_arc_set_value(ui_deltaarc, (delta/100) + 200);
  updateDeltaColor(delta);

  deltaField.lastValue = delta;
  deltaField.lastUpdateMs = now;
}

// update consumption label on the screen
void CarStateUI::updateConsumptionUI() {
  uint32_t now = millis();
  if (now - consumField.lastUpdateMs < consumField.refreshIntervalMs) {
    return;
  }

  float consumption = carState.getEnergyConsumption();

  if (fabs(consumption - consumField.lastValue) < consumField.valueThreshold) {
    return;
  }

  int consumption_int = (int)consumption;
  int consumption_dec = (int)(fabs(consumption - consumption_int) * 10 + 0.5f) % 10;

  lv_label_set_text_fmt(ui_consolabel, "%d.%d", consumption_int, consumption_dec);

  consumField.lastValue = consumption;
  consumField.lastUpdateMs = now;
}

// update battery SOC color on the screen
void CarStateUI::updateBatteryLevelColor(int level) {
  uint32_t color = (level <= 15) ? 0xFF0000 : (level >= 75) ? 0x00FF00 : 0xFFFFFF; // Red if <=15%, Green if >=75%, White otherwise
  if (color == lastBatteryLevelColor)
    return;
  lastBatteryLevelColor = color;
  lv_obj_set_style_text_color(ui_batterylevellabel, lv_color_hex(color), LV_PART_MAIN);
  lv_obj_set_style_text_color(ui_percent, lv_color_hex(color), LV_PART_MAIN);
  lv_obj_set_style_bg_color(ui_batterylevel, lv_color_hex(color), LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(ui_batterylevel, lv_color_hex(color), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(ui_batterylevel, 100, LV_PART_MAIN);
}

// update battery temperature color on the screen
void CarStateUI::updateBatteryTempColor(int temp) {
  bool isOverheated = (temp > 70); // Overheated if temperature is above 70 degrees Celsius
  if (isOverheated == lastBatteryTempOverheated)
    return;
  lastBatteryTempOverheated = isOverheated;
  uint32_t color = isOverheated ? 0xFF0000 : 0xFFFFFF; // Red if overheated, White otherwise

  lv_obj_set_style_text_color(ui_batterytemplabel, lv_color_hex(color), LV_PART_MAIN);
  lv_obj_set_style_text_color(ui_celsius2, lv_color_hex(color), LV_PART_MAIN);
  lv_obj_set_style_arc_color(ui_batterytemp, lv_color_hex(color), LV_PART_INDICATOR | LV_STATE_DEFAULT);
}

// update engine temperature color on the screen
void CarStateUI::updateEngineTempColor(int temp) {
  uint32_t color = (temp < 90) ? 0x00FF00 : (temp < 110) ? 0xFFFF00 : 0xFF0000; // Green if <90, Yellow if <110, Red otherwise
  if (color == lastEngineTempColor)
    return;
  lastEngineTempColor = color;
  lv_obj_set_style_bg_color(ui_enginetemp, lv_color_hex(color), LV_PART_MAIN);
}

// update inverter temperature color on the screen
void CarStateUI::updateInverterTempColor(int temp) {
  uint32_t color = (temp < 90) ? 0x00FF00 : (temp < 110) ? 0xFFFF00 : 0xFF0000; // Green if <90, Yellow if <110, Red otherwise
  if (color == lastInverterTempColor)
    return;
  lastInverterTempColor = color;
  lv_obj_set_style_bg_color(ui_invertertemp, lv_color_hex(color), LV_PART_MAIN);
}

// update delta color on the screen
void CarStateUI::updateDeltaColor(float delta) {
  uint32_t color = (delta < 0) ? 0xFF0000 : (delta == 0) ? 0xFFFFFF : 0x00FF00; // Red if negative, White if zero, Green if positive
  if (color == lastDeltaColor)
    return;
  lastDeltaColor = color;
  lv_obj_set_style_text_color(ui_deltalabel, lv_color_hex(color), LV_PART_MAIN);
  lv_obj_set_style_text_color(ui_deltasignlabel, lv_color_hex(color), LV_PART_MAIN);
  lv_obj_set_style_arc_color(ui_deltaarc, lv_color_hex(color), LV_PART_INDICATOR | LV_STATE_DEFAULT);
}
