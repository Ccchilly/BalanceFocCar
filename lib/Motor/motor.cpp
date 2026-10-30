#include "motor.h"

#include <cmath>

namespace
{
constexpr unsigned long STATUS_PRINT_INTERVAL_MS = 250;
}

motor_control::motor_control(encoder_control &encoder,
                             int pwmA, int pwmB, int pwmC,
                             const char *name,
                             const motor_settings &settings)
    : encoder_(encoder),
      name_(name),
      settings_(settings),
      motor_(settings.polePairs),
      driver_(pwmA, pwmB, pwmC)
{
}

bool motor_control::begin()
{
    ready_ = false;
    aligned_ = false;
    enabled_ = false;
    fault_ = false;
    targetVelocity_ = 0.0f;

    if (!encoder_.isReady() || !encoder_.getData().valid)
    {
        Serial.print(name_);
        Serial.println(F(" begin failed: encoder is not ready"));
        fault_ = true;
        return false;
    }

    driver_.voltage_power_supply = settings_.supplyVoltage;
    driver_.voltage_limit = settings_.supplyVoltage;

    if (!driver_.init())
    {
        Serial.print(name_);
        Serial.println(F(" driver initialization failed"));
        fault_ = true;
        return false;
    }

    motor_.linkSensor(&encoder_);
    motor_.linkDriver(&driver_);

    motor_.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor_.modulation_centered = 1;
    motor_.torque_controller = TorqueControlType::voltage;
    motor_.controller = MotionControlType::velocity;

    motor_.PID_velocity.P = settings_.velocityKp;
    motor_.PID_velocity.I = settings_.velocityKi;
    motor_.PID_velocity.D = settings_.velocityKd;
    motor_.PID_velocity.output_ramp = settings_.outputRamp;
    motor_.LPF_velocity.Tf = settings_.velocityFilter;

    motor_.voltage_sensor_align = settings_.alignmentVoltage;
    motor_.voltage_limit = settings_.voltageLimit;
    motor_.velocity_limit = settings_.velocityLimit;
    motor_.target = 0.0f;

    motor_.init();
    motor_.disable();

    if (motor_.motor_status == FOCMotorStatus::motor_init_failed)
    {
        Serial.print(name_);
        Serial.println(F(" motor initialization failed"));
        fault_ = true;
        return false;
    }

    ready_ = true;
    lastStatusPrintMs_ = millis();

    if (debug_)
    {
        Serial.print(name_);
        Serial.println(F(" initialized and disabled"));
    }

    return true;
}

bool motor_control::alignFOC()
{
    if (!ready_)
    {
        Serial.print(name_);
        Serial.println(F(" alignment rejected: motor is not initialized"));
        return false;
    }

    if (!encoder_.getData().valid)
    {
        Serial.print(name_);
        Serial.println(F(" alignment rejected: encoder is invalid"));
        fault_ = true;
        return false;
    }

    disable();

    if (debug_)
    {
        Serial.print(name_);
        Serial.println(F(" FOC alignment starting; keep the wheel suspended"));
    }

    motor_.enable();
    const bool success = motor_.initFOC() != 0;
    motor_.target = 0.0f;
    motor_.disable();

    enabled_ = false;
    targetVelocity_ = 0.0f;
    aligned_ = success;
    fault_ = !success;

    Serial.print(name_);
    Serial.println(success ? F(" FOC alignment complete")
                           : F(" FOC alignment failed"));

    return success;
}

bool motor_control::enable()
{
    if (!ready_ || !aligned_ || fault_)
    {
        Serial.print(name_);
        Serial.println(F(" enable rejected: motor is not ready"));
        return false;
    }

    if (!encoder_.getData().valid)
    {
        Serial.print(name_);
        Serial.println(F(" enable rejected: encoder is invalid"));
        fault_ = true;
        return false;
    }

    targetVelocity_ = 0.0f;
    motor_.target = 0.0f;
    motor_.enable();
    enabled_ = true;
    return true;
}

void motor_control::disable()
{
    targetVelocity_ = 0.0f;
    motor_.target = 0.0f;
    motor_.PID_velocity.reset();
    motor_.disable();
    enabled_ = false;
}

void motor_control::setTargetVelocity(float targetVelocity)
{
    if (!std::isfinite(targetVelocity))
    {
        stopWithFault(F("non-finite target"));
        return;
    }

    targetVelocity_ = constrain(targetVelocity,
                                -settings_.velocityLimit,
                                settings_.velocityLimit);
}

void motor_control::update()
{
    if (!ready_)
    {
        return;
    }

    // loopFOC() calls encoder_.update() through the SimpleFOC Sensor interface.
    motor_.loopFOC();

    if (!encoder_.getData().valid)
    {
        stopWithFault(F("encoder became invalid"));
        return;
    }

    motor_.move(enabled_ ? targetVelocity_ : 0.0f);

    if (enabled_ && motor_.motor_status != FOCMotorStatus::motor_ready)
    {
        stopWithFault(F("SimpleFOC motor status is not ready"));
        return;
    }

    if (debug_)
    {
        printStatus(millis());
    }
}

float motor_control::getTargetVelocity() const
{
    return targetVelocity_;
}

float motor_control::getVelocity() const
{
    return motor_.shaft_velocity;
}

float motor_control::getAngle() const
{
    return motor_.shaft_angle;
}

bool motor_control::isReady() const
{
    return ready_;
}

bool motor_control::isAligned() const
{
    return aligned_;
}

bool motor_control::isEnabled() const
{
    return enabled_;
}

bool motor_control::hasFault() const
{
    return fault_;
}

bool motor_control::clearFault()
{
    if (!ready_ || !encoder_.getData().valid)
    {
        return false;
    }

    disable();
    fault_ = false;
    return true;
}

void motor_control::setDebug(bool enabled)
{
    debug_ = enabled;
}

void motor_control::stopWithFault(const __FlashStringHelper *reason)
{
    const bool firstFault = !fault_;
    disable();
    fault_ = true;

    if (firstFault)
    {
        Serial.print(name_);
        Serial.print(F(" stopped: "));
        Serial.println(reason);
    }
}

void motor_control::printStatus(unsigned long now)
{
    if (now - lastStatusPrintMs_ < STATUS_PRINT_INTERVAL_MS)
    {
        return;
    }

    lastStatusPrintMs_ = now;

    Serial.print(name_);
    Serial.print(F(" target="));
    Serial.print(targetVelocity_, 3);
    Serial.print(F(" rad/s  velocity="));
    Serial.print(motor_.shaft_velocity, 3);
    Serial.print(F(" rad/s  state="));

    if (fault_)
    {
        Serial.println(F("FAULT"));
    }
    else if (enabled_)
    {
        Serial.println(F("ENABLED"));
    }
    else if (aligned_)
    {
        Serial.println(F("ALIGNED_DISABLED"));
    }
    else
    {
        Serial.println(F("NOT_ALIGNED"));
    }
}
