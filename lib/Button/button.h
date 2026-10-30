#ifndef BALANCE_CAR_BUTTON_H
#define BALANCE_CAR_BUTTON_H

#include <Arduino.h>

// Pins and thresholds for the power/button circuit.
namespace button_config
{
constexpr int BUTTON_PIN = 6;
constexpr int POWER_ENABLE_PIN = 7;
constexpr int BATTERY_ADC_PIN = 1;

constexpr uint32_t DEBOUNCE_MS = 30;
constexpr uint32_t LONG_PRESS_MS = 1200;
constexpr uint32_t BATTERY_CHECK_INTERVAL_MS = 1000;

constexpr uint8_t BATTERY_SAMPLE_COUNT = 8;
constexpr uint16_t BATTERY_PRESENT_RAW = 100;
constexpr uint16_t BATTERY_LOW_RAW = 3000;
constexpr uint16_t BATTERY_RECOVER_RAW = 3100;
}

class button_control
{
public:
    // Enable the power latch and prepare the button and battery ADC.
    bool begin();

    // Poll once from the main control loop.
    void update();

    // A short press requests arm/disarm. The total controller consumes it.
    bool consumeArmToggleRequest();

    // A long press requests a safe motor stop followed by power-off.
    bool consumePowerOffRequest();

    // A low battery requests a safe motor stop.
    bool consumeLowBatteryRequest();

    // Execute the physical power-off after the motor controller is disabled.
    void powerOff();

    // Notify the user that sensor/FOC self-check has completed.
    void notifyReady();

    uint16_t batteryAdcRaw() const;
    bool isPowerEnabled() const;
    bool isPressed() const;

    // Compatibility name for the previous ButtonAndBattery entry point.
    void startTask();

    void setDebug(bool enabled);

private:
    void updateButton(uint32_t now);
    void updateBattery(uint32_t now);
    void handleReleased(uint32_t now);

    bool initialized_ = false;
    bool debug_ = true;
    bool powerEnabled_ = false;

    bool rawPressed_ = false;
    bool stablePressed_ = false;
    bool ignoreStartupPress_ = false;
    bool longPressHandled_ = false;
    uint32_t rawChangedAt_ = 0;
    uint32_t pressedAt_ = 0;
    uint32_t lastBatteryCheckAt_ = 0;

    bool armToggleRequested_ = false;
    bool powerOffRequested_ = false;
    bool lowBatteryRequested_ = false;
    bool lowBatteryReported_ = false;
    uint16_t batteryAdcRaw_ = 0;
};

extern button_control button;

// Legacy name retained for old code during the migration.
using BNT_AND_PWR = button_control;
extern button_control &ButtonAndPower;
extern volatile uint16_t batteryAdcRaw;

// Legacy callbacks retained without a dependency on SuperCar.
void power_off();
void focSelfCheckCompleted();
void turnOffRGB();

#endif
