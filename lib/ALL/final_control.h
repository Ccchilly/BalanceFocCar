#ifndef BALANCE_CAR_FINAL_CONTROL_H
#define BALANCE_CAR_FINAL_CONTROL_H

#include <Arduino.h>
#include <Wire.h>

#include "../Button/button.h"
#include "../Encoder/encoder.h"
#include "../IMU/imu.h"
#include "../Motor/motor.h"
#include "../PID/PID.h"
#include "../WIFI/wifi_control.h"

namespace final_control_hardware
{
constexpr int ENCODER_A_SDA = 37;
constexpr int ENCODER_A_SCL = 36;

constexpr int MOTOR_A_PWM_A = 35;
constexpr int MOTOR_A_PWM_B = 34;
constexpr int MOTOR_A_PWM_C = 33;

constexpr int MOTOR_B_PWM_A = 12;
constexpr int MOTOR_B_PWM_B = 11;
constexpr int MOTOR_B_PWM_C = 10;

constexpr uint32_t I2C_FREQUENCY_HZ = 400000;
}

enum class final_control_state : uint8_t
{
    STARTING,
    STANDBY,
    ARMED,
    FAULT
};

struct final_control_settings
{
    // Suspended-wheel test; false restores the balance controller.
    bool wheelTestMode = true;
    bool wheelTestUseBle = false; // false: existing PC gamepad -> WiFi UDP.
    float wheelTestVelocityLimit = 2.5f;
    // Wheel-speed loop: requested speed -> requested lean angle.
    float speedKp = 0.0f;
    float speedKi = 0.0f;
    float speedIntegralLimit = 3.0f;
    float targetAngleLimit = 3.0f;

    // Upright loop: lean angle -> motor shaft velocity.
    float uprightKp = 1.5f;
    float uprightKi = 0.0f;
    float uprightKd = 0.003f;
    float uprightOutputLimit = 8.0f;
    float uprightIntegralLimit = 6.0f;

    float armGyroLimit = 20.0f;
    float fallAngleLimit = 25.0f;
    float balanceDirection = -1.0f;

    uint32_t speedLoopIntervalUs = 10000;
    uint32_t telemetryIntervalMs = 1000;
};

class final_control
{
public:
    explicit final_control(
        const final_control_settings &settings = final_control_settings{});

    // Initialize all hardware. Wheels move briefly during FOC alignment,
    // then both motors return to the disabled standby state.
    bool begin();

    // Call continuously from Arduino loop(), without delay().
    void update();

    bool arm();
    void disarm(const char *reason = "manual stop");

    bool isReady() const;
    bool isArmed() const;
    final_control_state state() const;
    const char *stateName() const;

    float relativePitch() const;
    float targetAngle() const;
    float balanceOutput() const;

    void setDebug(bool enabled);

private:
    void processUserRequests();
    void updateControl(uint32_t nowUs);
    void updateSpeedLoop(const remote_data &command, float deltaTimeSeconds);
    void updateLinkFeedback(const remote_data &command);
    void enterFault(const char *reason);
    void disableMotors();
    void setState(final_control_state newState);
    void printTelemetry(uint32_t nowMs);
    bool sensorsValid() const;
    remote_data readCommand() const;

    final_control_settings settings_;

    TwoWire encoderBusA_;
    imu_control imu_;
    encoder_control encoderA_;
    encoder_control encoderB_;
    motor_control motorA_;
    motor_control motorB_;
    upright_pid uprightPid_;
    wifi_control wifi_;

    final_control_state state_ = final_control_state::STARTING;
    bool initialized_ = false;
    bool debug_ = true;
    bool lastRemoteBalanceHeld_ = true;
    bool lastLinkConnected_ = false;

    float uprightZero_ = 0.0f;
    float relativePitch_ = 0.0f;
    float targetAngle_ = 0.0f;
    float balanceOutput_ = 0.0f;
    float speedIntegral_ = 0.0f;

    uint32_t lastControlUs_ = 0;
    uint32_t lastSpeedLoopUs_ = 0;
    uint32_t lastTelemetryMs_ = 0;
};

extern final_control finalControl;

#endif
