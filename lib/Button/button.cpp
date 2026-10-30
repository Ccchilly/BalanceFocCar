#include "button.h"

#include "../Buzzer/buzzer.h"
#include "../Config/config.h"
#include "../RGB/RGBLED.h"

namespace
{
bool readButtonPressed()
{
    return digitalRead(button_config::BUTTON_PIN) == LOW;
}
}

button_control button;
button_control &ButtonAndPower = button;
volatile uint16_t batteryAdcRaw = 0;

bool button_control::begin()
{
    pinMode(button_config::POWER_ENABLE_PIN, OUTPUT);
    digitalWrite(button_config::POWER_ENABLE_PIN, HIGH);
    powerEnabled_ = true;

    pinMode(button_config::BUTTON_PIN, INPUT_PULLUP);
    pinMode(button_config::BATTERY_ADC_PIN, INPUT);

    analogReadResolution(12);
#if defined(ESP32)
    analogSetPinAttenuation(button_config::BATTERY_ADC_PIN, ADC_11db);
#endif

    buzzer.begin(pins::BUZZER);
    rgb.begin();
    rgb.setWhite();

    const uint32_t now = millis();
    rawPressed_ = readButtonPressed();
    stablePressed_ = rawPressed_;
    ignoreStartupPress_ = stablePressed_;
    rawChangedAt_ = now;
    lastBatteryCheckAt_ = now;
    initialized_ = true;

    if (debug_)
    {
        Serial.println(F("Button and power control ready"));
        Serial.println(ignoreStartupPress_
                           ? F("Startup button press will be ignored")
                           : F("Waiting for button events"));
    }

    return true;
}

void button_control::startTask()
{
    begin();
}

void button_control::update()
{
    if (!initialized_)
    {
        return;
    }

    const uint32_t now = millis();
    updateButton(now);
    updateBattery(now);
}

void button_control::updateButton(uint32_t now)
{
    const bool currentPressed = readButtonPressed();

    if (currentPressed != rawPressed_)
    {
        rawPressed_ = currentPressed;
        rawChangedAt_ = now;
    }

    if (now - rawChangedAt_ < button_config::DEBOUNCE_MS ||
        rawPressed_ == stablePressed_)
    {
        return;
    }

    stablePressed_ = rawPressed_;

    if (stablePressed_)
    {
        pressedAt_ = now;
        longPressHandled_ = false;
        return;
    }

    handleReleased(now);
}

void button_control::handleReleased(uint32_t now)
{
    const uint32_t pressDuration = now - pressedAt_;

    if (ignoreStartupPress_)
    {
        ignoreStartupPress_ = false;
        return;
    }

    if (longPressHandled_ || pressDuration >= button_config::LONG_PRESS_MS)
    {
        powerOffRequested_ = true;
        return;
    }

    armToggleRequested_ = true;
}

void button_control::updateBattery(uint32_t now)
{
    if (now - lastBatteryCheckAt_ < button_config::BATTERY_CHECK_INTERVAL_MS)
    {
        return;
    }

    lastBatteryCheckAt_ = now;

    uint32_t total = 0;
    for (uint8_t sample = 0; sample < button_config::BATTERY_SAMPLE_COUNT; ++sample)
    {
        total += analogRead(button_config::BATTERY_ADC_PIN);
    }

    batteryAdcRaw_ = static_cast<uint16_t>(
        total / button_config::BATTERY_SAMPLE_COUNT);
    ::batteryAdcRaw = batteryAdcRaw_;

    // A zero divider is expected during USB-only testing.
    if (batteryAdcRaw_ < button_config::BATTERY_PRESENT_RAW)
    {
        lowBatteryReported_ = false;
        return;
    }

    if (batteryAdcRaw_ < button_config::BATTERY_LOW_RAW &&
        !lowBatteryReported_)
    {
        lowBatteryReported_ = true;
        lowBatteryRequested_ = true;

        if (debug_)
        {
            Serial.print(F("Low battery ADC="));
            Serial.println(batteryAdcRaw_);
        }
    }
    else if (batteryAdcRaw_ > button_config::BATTERY_RECOVER_RAW)
    {
        lowBatteryReported_ = false;
    }
}

bool button_control::consumeArmToggleRequest()
{
    const bool requested = armToggleRequested_;
    armToggleRequested_ = false;
    return requested;
}

bool button_control::consumePowerOffRequest()
{
    const bool requested = powerOffRequested_;
    powerOffRequested_ = false;
    return requested;
}

bool button_control::consumeLowBatteryRequest()
{
    const bool requested = lowBatteryRequested_;
    lowBatteryRequested_ = false;
    return requested;
}

void button_control::powerOff()
{
    if (!powerEnabled_)
    {
        return;
    }

    buzzer.powerOff();
    rgb.off();
    digitalWrite(button_config::POWER_ENABLE_PIN, LOW);
    powerEnabled_ = false;
}

void button_control::notifyReady()
{
    buzzer.beep();
}

uint16_t button_control::batteryAdcRaw() const
{
    return batteryAdcRaw_;
}

bool button_control::isPowerEnabled() const
{
    return powerEnabled_;
}

bool button_control::isPressed() const
{
    return stablePressed_;
}

void button_control::setDebug(bool enabled)
{
    debug_ = enabled;
}

void power_off()
{
    button.powerOff();
}

void focSelfCheckCompleted()
{
    button.notifyReady();
}

void turnOffRGB()
{
    rgb.off();
}
