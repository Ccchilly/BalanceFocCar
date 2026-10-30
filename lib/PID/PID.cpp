#include "PID.h"

#include <cmath>

namespace
{
constexpr float MIN_DELTA_TIME_SECONDS = 0.0001f;
constexpr float MAX_DELTA_TIME_SECONDS = 0.05f;
}

upright_pid::upright_pid(const upright_pid_settings &settings)
    : settings_(settings)
{
    setLimits(settings.outputLimit, settings.integralLimit);
    reset();
}

void upright_pid::setGains(float kp, float ki, float kd)
{
    settings_.kp = kp;
    settings_.ki = ki;
    settings_.kd = kd;
}

void upright_pid::setLimits(float outputLimit, float integralLimit)
{
    settings_.outputLimit = fabsf(outputLimit);
    settings_.integralLimit = fabsf(integralLimit);

    integralOutput_ = constrain(integralOutput_,
                                -settings_.integralLimit,
                                settings_.integralLimit);
    output_ = constrain(output_,
                        -settings_.outputLimit,
                        settings_.outputLimit);
}

void upright_pid::reset()
{
    error_ = 0.0f;
    proportionalOutput_ = 0.0f;
    integralOutput_ = 0.0f;
    derivativeOutput_ = 0.0f;
    output_ = 0.0f;
}

float upright_pid::update(float targetAngle, float actualAngle,
                          float gyroRate, float deltaTimeSeconds)
{
    if (!std::isfinite(targetAngle) ||
        !std::isfinite(actualAngle) ||
        !std::isfinite(gyroRate) ||
        !std::isfinite(deltaTimeSeconds) ||
        deltaTimeSeconds < MIN_DELTA_TIME_SECONDS ||
        deltaTimeSeconds > MAX_DELTA_TIME_SECONDS)
    {
        reset();
        return 0.0f;
    }

    // Positive error means the target is above the measured angle.
    error_ = targetAngle - actualAngle;
    proportionalOutput_ = settings_.kp * error_;

    // d(error)/dt is approximately -gyroRate because gyroRate is d(angle)/dt.
    derivativeOutput_ = -settings_.kd * gyroRate;

    const float candidateIntegral = constrain(
        integralOutput_ + settings_.ki * error_ * deltaTimeSeconds,
        -settings_.integralLimit,
        settings_.integralLimit);

    const float candidateOutput = proportionalOutput_ +
                                   candidateIntegral +
                                   derivativeOutput_;

    // Do not integrate farther into an already saturated output. When the
    // error changes sign, integration is allowed to unwind normally.
    const bool saturatingHigh = candidateOutput > settings_.outputLimit && error_ > 0.0f;
    const bool saturatingLow = candidateOutput < -settings_.outputLimit && error_ < 0.0f;
    if (!saturatingHigh && !saturatingLow)
    {
        integralOutput_ = candidateIntegral;
    }

    output_ = constrain(proportionalOutput_ +
                            integralOutput_ +
                            derivativeOutput_,
                        -settings_.outputLimit,
                        settings_.outputLimit);

    return output_;
}

float upright_pid::update(const imu_data &imu, float targetAngle,
                          float deltaTimeSeconds)
{
    if (!imu.valid)
    {
        reset();
        return 0.0f;
    }

    return update(targetAngle, imu.pitch, imu.gyro, deltaTimeSeconds);
}

float upright_pid::getOutput() const
{
    return output_;
}

float upright_pid::getError() const
{
    return error_;
}

float upright_pid::getProportionalOutput() const
{
    return proportionalOutput_;
}

float upright_pid::getIntegralOutput() const
{
    return integralOutput_;
}

float upright_pid::getDerivativeOutput() const
{
    return derivativeOutput_;
}
