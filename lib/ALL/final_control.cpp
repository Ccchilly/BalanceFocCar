#include "final_control.h"

#include "../Buzzer/buzzer.h"
#include "../Config/config.h"
#include "../RGB/RGBLED.h"
#include "../APP/APP.h"
#include "../UserConfig/UserConfig.h"

#include <cmath>

namespace
{
constexpr float MIN_CONTROL_DT_SECONDS = 0.0001f;
constexpr float MAX_CONTROL_DT_SECONDS = 0.05f;

upright_pid_settings makeUprightSettings(
    const final_control_settings &settings)
{
    upright_pid_settings pidSettings;
    pidSettings.kp = settings.uprightKp;
    pidSettings.ki = settings.uprightKi;
    pidSettings.kd = settings.uprightKd;
    pidSettings.outputLimit = settings.uprightOutputLimit;
    pidSettings.integralLimit = settings.uprightIntegralLimit;
    return pidSettings;
}
}

final_control::final_control(const final_control_settings &settings)
    : settings_(settings),
      encoderBusA_(1),
      encoderA_(encoderBusA_, "Encoder A"),
      encoderB_(imu_.bus(), "Encoder B"),
      motorA_(encoderA_,
              final_control_hardware::MOTOR_A_PWM_A,
              final_control_hardware::MOTOR_A_PWM_B,
              final_control_hardware::MOTOR_A_PWM_C,
              "Motor A"),
      motorB_(encoderB_,
              final_control_hardware::MOTOR_B_PWM_A,
              final_control_hardware::MOTOR_B_PWM_B,
              final_control_hardware::MOTOR_B_PWM_C,
              "Motor B"),
      uprightPid_(makeUprightSettings(settings))
{
}

bool final_control::begin()
{
    initialized_ = false;
    setState(final_control_state::STARTING);

    Serial.println();
    Serial.println(F("=== Balance car final controller ==="));
    Serial.println(F("Keep both wheels suspended during startup."));
    Serial.println(F("Keep the chassis still during IMU calibration."));

    button.setDebug(debug_);
    if (!button.begin())
    {
        enterFault("button/power initialization failed");
        return false;
    }

    // button.begin() initializes RGB and leaves a white standby light.
    buzzer.powerOn();

    imu_.setDebug(debug_);
    encoderA_.setDebug(debug_);
    encoderB_.setDebug(debug_);
    motorA_.setDebug(debug_);
    motorB_.setDebug(debug_);
    wifi_.setDebug(debug_);

    if (!imu_.begin(true))
    {
        enterFault("IMU initialization failed");
        return false;
    }

    if (!encoderA_.begin(final_control_hardware::ENCODER_A_SDA,
                         final_control_hardware::ENCODER_A_SCL,
                         final_control_hardware::I2C_FREQUENCY_HZ))
    {
        enterFault("encoder A initialization failed");
        return false;
    }

    // IMU already initialized I2C0 on GPIO8/GPIO9. Encoder B shares it.
    if (!encoderB_.begin())
    {
        enterFault("encoder B initialization failed");
        return false;
    }

    if (!motorA_.begin() || !motorB_.begin())
    {
        enterFault("motor initialization failed");
        return false;
    }

    Serial.println(F("Aligning motor A..."));
    if (!motorA_.alignFOC())
    {
        enterFault("motor A FOC alignment failed");
        return false;
    }

    Serial.println(F("Aligning motor B..."));
    if (!motorB_.alignFOC())
    {
        enterFault("motor B FOC alignment failed");
        return false;
    }

    if (settings_.wheelTestMode && settings_.wheelTestUseBle)
    {
        APP.startTask();
        Serial.println(F("WHEEL TEST: BLE stick controls velocity/steering; balance is bypassed."));
    }
    else if (!wifi_.begin())
    {
        // WiFi is optional for standing still; the physical button remains usable.
        Serial.println(F("Warning: WiFi control is unavailable"));
    }

    // Keep only the consolidated controller telemetry during real-time FOC.
    imu_.setDebug(false);
    encoderA_.setDebug(false);
    encoderB_.setDebug(false);
    motorA_.setDebug(false);
    motorB_.setDebug(false);
    wifi_.setDebug(false);

    disableMotors();
    uprightPid_.reset();
    speedIntegral_ = 0.0f;
    targetAngle_ = 0.0f;
    balanceOutput_ = 0.0f;

    const uint32_t nowUs = micros();
    lastControlUs_ = nowUs;
    lastSpeedLoopUs_ = nowUs;
    lastTelemetryMs_ = millis();
    initialized_ = true;
    setState(final_control_state::STANDBY);

    button.notifyReady();
    Serial.println(settings_.wheelTestMode
        ? F("READY: suspend wheels, connect remote, center stick, then short-press START.")
        : F("READY: hold the car upright, then short-press START."));
    Serial.println(F("Long-press START to request a safe power-off."));
    return true;
}

void final_control::update()
{
    button.update();
    if (!settings_.wheelTestMode || !settings_.wheelTestUseBle)
    {
        wifi_.update();
    }

    if (!initialized_)
    {
        if (button.consumePowerOffRequest())
        {
            disableMotors();
            button.powerOff();
        }
        return;
    }

    imu_.update();

    // motor.update() performs both FOC and encoder updates.
    motorA_.update();
    motorB_.update();

    processUserRequests();

    if (state_ == final_control_state::ARMED)
    {
        updateControl(micros());
    }

    printTelemetry(millis());
}

void final_control::processUserRequests()
{
    if (button.consumePowerOffRequest())
    {
        disarm("power button");
        button.powerOff();
        return;
    }

    if (button.consumeLowBatteryRequest())
    {
        enterFault("low battery");
    }

    const remote_data command = readCommand();
    updateLinkFeedback(command);

    const bool remoteArmEdge = command.connected &&
                               command.balanceHeld &&
                               !lastRemoteBalanceHeld_;
    lastRemoteBalanceHeld_ = command.connected ? command.balanceHeld : true;

    if (button.consumeArmToggleRequest() || remoteArmEdge)
    {
        if (state_ == final_control_state::ARMED)
        {
            disarm("user request");
        }
        else if (state_ == final_control_state::STANDBY)
        {
            arm();
        }
    }
}

bool final_control::arm()
{
    if (!initialized_ || state_ != final_control_state::STANDBY)
    {
        Serial.println(F("ARM rejected: controller is not in standby"));
        return false;
    }

    const imu_data &imuData = imu_.getData();
    if (!sensorsValid())
    {
        Serial.println(F("ARM rejected: sensor data is invalid"));
        return false;
    }

    if (settings_.wheelTestMode)
    {
        const remote_data command = readCommand();
        if (!command.connected || fabsf(command.velocity) > 0.05f ||
            fabsf(command.steer) > 0.05f)
        {
            Serial.println(F("ARM rejected: connect gamepad and center stick"));
            return false;
        }
    }

    if (!settings_.wheelTestMode && fabsf(imuData.pitch) > car_config::MAX_SAFE_TILT_DEG)
    {
        Serial.print(F("ARM rejected: pitch="));
        Serial.println(imuData.pitch, 2);
        return false;
    }

    if (!settings_.wheelTestMode && fabsf(imuData.gyro) > settings_.armGyroLimit)
    {
        Serial.print(F("ARM rejected: gyro="));
        Serial.println(imuData.gyro, 2);
        return false;
    }

    uprightZero_ = imuData.pitch;
    relativePitch_ = 0.0f;
    targetAngle_ = 0.0f;
    balanceOutput_ = 0.0f;
    speedIntegral_ = 0.0f;
    uprightPid_.reset();

    // Finish all blocking user feedback before enabling motor torque.
    // buzzer.beep();

    if (!motorA_.enable())
    {
        enterFault("motor A enable failed");
        return false;
    }

    if (!motorB_.enable())
    {
        motorA_.disable();
        enterFault("motor B enable failed");
        return false;
    }

    const uint32_t nowUs = micros();
    lastControlUs_ = nowUs;
    lastSpeedLoopUs_ = nowUs;
    setState(final_control_state::ARMED);
    rgb.setGreen();

    Serial.print(F("ARMED: upright zero="));
    Serial.println(uprightZero_, 2);
    return true;
}

void final_control::disarm(const char *reason)
{
    const bool wasArmed = state_ == final_control_state::ARMED;
    disableMotors();
    uprightPid_.reset();
    speedIntegral_ = 0.0f;
    targetAngle_ = 0.0f;
    balanceOutput_ = 0.0f;

    if (state_ != final_control_state::FAULT)
    {
        setState(final_control_state::STANDBY);
        rgb.setWhite();
    }

    if (wasArmed && debug_)
    {
        Serial.print(F("DISARMED: "));
        Serial.println(reason);
    }
}

void final_control::updateControl(uint32_t nowUs)
{
    const uint32_t elapsedUs = nowUs - lastControlUs_;
    lastControlUs_ = nowUs;
    const float controlDt = static_cast<float>(elapsedUs) * 0.000001f;

    if (controlDt < MIN_CONTROL_DT_SECONDS ||
        controlDt > MAX_CONTROL_DT_SECONDS)
    {
        enterFault("invalid control interval");
        return;
    }

    if (!sensorsValid())
    {
        enterFault("sensor data became invalid");
        return;
    }

    if (motorA_.hasFault() || motorB_.hasFault())
    {
        enterFault("motor fault");
        return;
    }

    if (settings_.wheelTestMode)
    {
        const remote_data command = readCommand();
        if (!command.connected)
        {
            disarm("gamepad disconnected or command timed out");
            return;
        }
        const float limit = settings_.wheelTestVelocityLimit;
        motorA_.setTargetVelocity(constrain(-command.velocity + command.steer, -limit, limit));
        motorB_.setTargetVelocity(constrain(command.velocity + command.steer, -limit, limit));
        return;
    }

    const imu_data &imuData = imu_.getData();
    relativePitch_ = imuData.pitch - uprightZero_;

    if (fabsf(relativePitch_) > settings_.fallAngleLimit)
    {
        enterFault("fall angle exceeded");
        return;
    }

    const remote_data &command = wifi_.getCommand();
    const uint32_t speedElapsedUs = nowUs - lastSpeedLoopUs_;
    if (speedElapsedUs >= settings_.speedLoopIntervalUs)
    {
        lastSpeedLoopUs_ = nowUs;
        updateSpeedLoop(command,
                        static_cast<float>(speedElapsedUs) * 0.000001f);
    }

    balanceOutput_ = settings_.balanceDirection *
                     uprightPid_.update(targetAngle_,
                                        relativePitch_,
                                        imuData.gyro,
                                        controlDt);

    const float steer = command.connected ? command.steer : 0.0f;

    // The wheels are mirrored: forward motion uses opposite shaft signs.
    const float targetA = constrain(-balanceOutput_ + steer,
                                    -settings_.uprightOutputLimit,
                                    settings_.uprightOutputLimit);
    const float targetB = constrain(balanceOutput_ + steer,
                                    -settings_.uprightOutputLimit,
                                    settings_.uprightOutputLimit);

    motorA_.setTargetVelocity(targetA);
    motorB_.setTargetVelocity(targetB);
}

void final_control::updateSpeedLoop(const remote_data &command,
                                    float deltaTimeSeconds)
{
    const float requestedVelocity = command.connected ? command.velocity : 0.0f;
    const float measuredVelocity =
        (motorB_.getVelocity() - motorA_.getVelocity()) * 0.5f;
    const float speedError = requestedVelocity - measuredVelocity;

    speedIntegral_ = constrain(
        speedIntegral_ + settings_.speedKi * speedError * deltaTimeSeconds,
        -settings_.speedIntegralLimit,
        settings_.speedIntegralLimit);

    targetAngle_ = constrain(settings_.speedKp * speedError + speedIntegral_,
                             -settings_.targetAngleLimit,
                             settings_.targetAngleLimit);
}

void final_control::updateLinkFeedback(const remote_data &command)
{
    if (command.connected == lastLinkConnected_)
    {
        return;
    }

    lastLinkConnected_ = command.connected;
    if (state_ == final_control_state::ARMED)
    {
        return;
    }

    if (command.connected)
    {
        buzzer.connection();
    }
    else
    {
        buzzer.disconnection();
    }
}

void final_control::enterFault(const char *reason)
{
    disableMotors();
    uprightPid_.reset();
    speedIntegral_ = 0.0f;
    targetAngle_ = 0.0f;
    balanceOutput_ = 0.0f;

    const bool firstFault = state_ != final_control_state::FAULT;
    setState(final_control_state::FAULT);
    rgb.setRed();

    if (firstFault)
    {
        Serial.print(F("FAULT: "));
        Serial.println(reason);
        buzzer.siren();
    }
}

void final_control::disableMotors()
{
    if (motorA_.isReady())
    {
        motorA_.setTargetVelocity(0.0f);
        motorA_.disable();
    }

    if (motorB_.isReady())
    {
        motorB_.setTargetVelocity(0.0f);
        motorB_.disable();
    }
}

void final_control::setState(final_control_state newState)
{
    state_ = newState;
}

bool final_control::sensorsValid() const
{
    return imu_.isReady() && imu_.getData().valid &&
           encoderA_.isReady() && encoderA_.getData().valid &&
           encoderB_.isReady() && encoderB_.getData().valid;
}

remote_data final_control::readCommand() const
{
    if (!settings_.wheelTestMode || !settings_.wheelTestUseBle)
    {
        return wifi_.getCommand();
    }
    remote_data command;
    const unsigned long lastReport = appCTRL.LastReportMs;
    command.connected = appCTRL.Connected && lastReport != 0 &&
                        millis() - lastReport <= GAMEPAD_REPORT_TIMEOUT_MS;
    if (command.connected)
    {
        command.velocity = appCTRL.Velocity;
        command.steer = appCTRL.SteerVelocity;
    }
    return command;
}

bool final_control::isReady() const
{
    return initialized_ && state_ != final_control_state::FAULT;
}

bool final_control::isArmed() const
{
    return state_ == final_control_state::ARMED;
}

final_control_state final_control::state() const
{
    return state_;
}

const char *final_control::stateName() const
{
    switch (state_)
    {
    case final_control_state::STARTING:
        return "STARTING";
    case final_control_state::STANDBY:
        return "STANDBY";
    case final_control_state::ARMED:
        return "ARMED";
    case final_control_state::FAULT:
        return "FAULT";
    }

    return "UNKNOWN";
}

float final_control::relativePitch() const
{
    return relativePitch_;
}

float final_control::targetAngle() const
{
    return targetAngle_;
}

float final_control::balanceOutput() const
{
    return balanceOutput_;
}

void final_control::setDebug(bool enabled)
{
    debug_ = enabled;
    button.setDebug(enabled);

    // Per-module periodic logs are intentionally kept off: several long
    // Serial lines in one iteration can disturb the balance control period.
    imu_.setDebug(false);
    encoderA_.setDebug(false);
    encoderB_.setDebug(false);
    motorA_.setDebug(false);
    motorB_.setDebug(false);
    wifi_.setDebug(false);
}

void final_control::printTelemetry(uint32_t nowMs)
{
    if (!debug_ ||
        nowMs - lastTelemetryMs_ < settings_.telemetryIntervalMs)
    {
        return;
    }

    lastTelemetryMs_ = nowMs;
    const remote_data command = readCommand();

    Serial.print(F("state="));
    Serial.print(stateName());
    Serial.print(F(" pitch="));
    Serial.print(imu_.getData().pitch, 2);
    Serial.print(F(" relative="));
    Serial.print(relativePitch_, 2);
    Serial.print(F(" gyro="));
    Serial.print(imu_.getData().gyro, 2);
    Serial.print(F(" cmdV="));
    Serial.print(command.velocity, 2);
    Serial.print(F(" steer="));
    Serial.print(command.steer, 2);
    Serial.print(F(" targetAngle="));
    Serial.print(targetAngle_, 2);
    Serial.print(F(" balanceOut="));
    Serial.print(balanceOutput_, 2);
    Serial.print(F(" velA="));
    Serial.print(motorA_.getVelocity(), 2);
    Serial.print(F(" velB="));
    Serial.print(motorB_.getVelocity(), 2);
    Serial.print(F(" targetA="));
    Serial.print(motorA_.getTargetVelocity(), 2);
    Serial.print(F(" targetB="));
    Serial.println(motorB_.getTargetVelocity(), 2);
}

final_control finalControl;
