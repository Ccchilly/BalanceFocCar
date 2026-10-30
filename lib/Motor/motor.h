#ifndef BALANCE_CAR_MOTOR_H
#define BALANCE_CAR_MOTOR_H

#include <Arduino.h>
#include <SimpleFOC.h>

#include "encoder.h"

struct motor_settings
{
    int polePairs = 7;
    float supplyVoltage = 8.4f;
    float voltageLimit = 8.0f;
    float velocityLimit = 8.0f;
    float alignmentVoltage = 0.6f;

    float velocityKp = 0.06f;
    float velocityKi = 0.0f;
    float velocityKd = 0.0f;
    float velocityFilter = 0.02f;
    float outputRamp = 1000.0f;
};

class motor_control
{
public:
    motor_control(encoder_control &encoder,
                  int pwmA, int pwmB, int pwmC,
                  const char *name = "Motor",
                  const motor_settings &settings = motor_settings{});

    // Configure the driver and velocity controller. The motor finishes disabled.
    bool begin();

    // Align motor phases with the encoder. The wheel moves briefly during this call.
    bool alignFOC();

    // Enable torque only after begin() and alignFOC() have succeeded.
    bool enable();
    void disable();

    // Set the requested mechanical shaft velocity in rad/s.
    void setTargetVelocity(float targetVelocity);

    // Run as frequently as possible. This also updates the linked encoder.
    void update();

    float getTargetVelocity() const;
    float getVelocity() const;
    float getAngle() const;

    bool isReady() const;
    bool isAligned() const;
    bool isEnabled() const;
    bool hasFault() const;
    bool clearFault();

    void setDebug(bool enabled);

private:
    void stopWithFault(const __FlashStringHelper *reason);
    void printStatus(unsigned long now);

    encoder_control &encoder_;
    const char *name_;
    motor_settings settings_;
    BLDCMotor motor_;
    BLDCDriver3PWM driver_;

    float targetVelocity_ = 0.0f;
    unsigned long lastStatusPrintMs_ = 0;
    bool ready_ = false;
    bool aligned_ = false;
    bool enabled_ = false;
    bool fault_ = false;
    bool debug_ = true; 
};

#endif
