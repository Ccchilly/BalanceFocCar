#ifndef BALANCE_CAR_BUTTON_AND_BATTERY_H
#define BALANCE_CAR_BUTTON_AND_BATTERY_H

// Compatibility include for the previous file name.
#include "button.h"
#include "../Config/config.h"

// Old code may still use these names while the new controller is assembled.
constexpr int BUTTON_PIN = button_config::BUTTON_PIN;
constexpr int BUZZER_PIN = pins::BUZZER;
constexpr int POWEREN_PIN = button_config::POWER_ENABLE_PIN;
constexpr int BAT_ADC_PIN = button_config::BATTERY_ADC_PIN;

constexpr int BATTERY_ADC_SAMPLE_COUNT = button_config::BATTERY_SAMPLE_COUNT;
constexpr int BATTERY_ADC_PRESENT_RAW = button_config::BATTERY_PRESENT_RAW;
constexpr int BATTERY_ADC_LOW_RAW = button_config::BATTERY_LOW_RAW;
constexpr int BATTERY_ADC_RECOVER_RAW = button_config::BATTERY_RECOVER_RAW;

#endif
