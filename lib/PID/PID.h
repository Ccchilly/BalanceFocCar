#ifndef BALANCE_CAR_UPRIGHT_PID_H
#define BALANCE_CAR_UPRIGHT_PID_H

#include <Arduino.h>

#include "config.h"

struct upright_pid_settings
{
    float kp = 1.0f;
    float ki = 0.0f;
    float kd = 0.02f;
    float outputLimit = 12.0f;
    float integralLimit = 6.0f;
};

class upright_pid
{
public:
    explicit upright_pid(const upright_pid_settings &settings = upright_pid_settings{});

    void setGains(float kp, float ki, float kd);
    void setLimits(float outputLimit, float integralLimit);

    // Clear the integral term and the previous calculation result.
    void reset();

    // Calculate the motor correction. Angles and angular rate use degrees
    // and degrees per second; the output is the requested motor velocity.
    float update(float targetAngle, float actualAngle,
                 float gyroRate, float deltaTimeSeconds);

    // Convenience overload for the shared IMU data structure.
    float update(const imu_data &imu, float targetAngle,
                 float deltaTimeSeconds);

    float getOutput() const;
    float getError() const;
    float getProportionalOutput() const;
    float getIntegralOutput() const;
    float getDerivativeOutput() const;

private:
    upright_pid_settings settings_;
    float error_ = 0.0f;
    float proportionalOutput_ = 0.0f;
    float integralOutput_ = 0.0f;
    float derivativeOutput_ = 0.0f;
    float output_ = 0.0f;
};

#endif
