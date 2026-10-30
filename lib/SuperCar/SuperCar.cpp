#include "SuperCar.h"
#include "APP.h"
#include "ButtonAndBattery.h"
#include "UprightPID.h"
#include "UserConfig.h"
#include <Arduino.h>
#include <MPU6050.h>
#include <SimpleFOC.h>

MagneticSensorI2C sensor_A = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor_B = MagneticSensorI2C(AS5600_I2C);

TwoWire I2C_A = TwoWire(1);
TwoWire I2C_B = TwoWire(0);
MPU6050 mpu6050(I2C_B);

BLDCMotor motor_A = BLDCMotor(7);
BLDCMotor motor_B = BLDCMotor(7);
BLDCDriver3PWM driver_A = BLDCDriver3PWM(35, 34, 33);
BLDCDriver3PWM driver_B = BLDCDriver3PWM(12, 11, 10);

UprightPID_t upRightPIDConfig;
PIDController Velocity_PID(VELOCITY_Kp, VELOCITY_Ki, VELOCITY_Kd, 1000, VELOCITY_LIMIT);

CarControl_t carCTRL;
uint8_t velocityCount = 0;
uint8_t angleCount = 0;

void SuperCar::startTask()
{
    ready = false;
    armed = false;

    Serial.println(F("Starting sensors. Keep the chassis still."));

    I2C_A.begin(37, 36, 400000UL);
    I2C_B.begin(8, 9, 400000UL);
    sensor_A.init(&I2C_A);
    sensor_B.init(&I2C_B);

    if (mpu6050.readMPU6050(MPU6050_WHO_AM_I) != MPU6050_ADDR)
    {
        Serial.println(F("ERROR: MPU6050 not found at 0x68. Motors stay disabled."));
        return;
    }

    mpu6050.begin();
    mpu6050.calcGyroOffsets(true, 1000, 100);
    mpu6050.begin();

    motor_A.linkSensor(&sensor_A);
    motor_B.linkSensor(&sensor_B);

    driver_A.voltage_power_supply = DRIVER_SUPPLY_VOLTAGE;
    driver_B.voltage_power_supply = DRIVER_SUPPLY_VOLTAGE;
    driver_A.init();
    driver_B.init();
    motor_A.linkDriver(&driver_A);
    motor_B.linkDriver(&driver_B);

    motor_A.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor_B.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor_A.modulation_centered = 1.0f;
    motor_B.modulation_centered = 1.0f;
    motor_A.torque_controller = TorqueControlType::voltage;
    motor_B.torque_controller = TorqueControlType::voltage;
    motor_A.controller = MotionControlType::velocity;
    motor_B.controller = MotionControlType::velocity;

    motor_A.PID_velocity.P = VEL_Kp;
    motor_A.PID_velocity.I = VEL_Ki;
    motor_A.PID_velocity.D = 0.0f;
    motor_A.PID_velocity.output_ramp = 1000;
    motor_A.LPF_velocity.Tf = 0.02f;
    motor_A.voltage_sensor_align = 0.6f;
    motor_A.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    motor_A.velocity_limit = MOTOR_VELOCITY_LIMIT;

    motor_B.PID_velocity.P = VEL_Kp;
    motor_B.PID_velocity.I = VEL_Ki;
    motor_B.PID_velocity.D = 0.0f;
    motor_B.PID_velocity.output_ramp = 1000;
    motor_B.LPF_velocity.Tf = 0.02f;
    motor_B.voltage_sensor_align = 0.6f;
    motor_B.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    motor_B.velocity_limit = MOTOR_VELOCITY_LIMIT;

    motor_A.init();
    motor_B.init();

    Serial.println(F("Aligning motor A..."));
    const int focAOk = motor_A.initFOC();
    motor_A.disable();

    Serial.println(F("Aligning motor B..."));
    const int focBOk = motor_B.initFOC();
    motor_B.disable();

    UprightPID_Init(&upRightPIDConfig, UPRIGHT_Kp, UPRIGHT_Ki, UPRIGHT_Kd, UPRIGHT_LIMIT);
    Velocity_PID.reset();

    if (!focAOk || !focBOk)
    {
        Serial.print(F("ERROR: FOC alignment failed. A="));
        Serial.print(focAOk);
        Serial.print(F(" B="));
        Serial.println(focBOk);
        Serial.println(F("Motors stay disabled."));
        return;
    }

    ready = true;
    focSelfCheckCompleted();
    Serial.println(F("READY: hold upright, then short-press START to arm."));
    Serial.println(F("Short-press again to stop; long-press to power off."));
}

void SuperCar::requestArmToggle()
{
    armToggleRequested = true;
}

void SuperCar::requestDisarm()
{
    disarmRequested = true;
}

bool SuperCar::isArmed() const
{
    return armed;
}

void SuperCar::armIfSafe()
{
    if (!ready)
    {
        Serial.println(F("ARM rejected: initialization is not ready."));
        return;
    }

    if (fabsf(carCTRL.MPUangleX) > ARM_ANGLE_LIMIT)
    {
        Serial.print(F("ARM rejected: hold closer to upright. angle="));
        Serial.println(carCTRL.MPUangleX, 2);
        return;
    }

    if (fabsf(carCTRL.MPUgyroX) > ARM_GYRO_LIMIT)
    {
        Serial.print(F("ARM rejected: chassis is moving. gyro="));
        Serial.println(carCTRL.MPUgyroX, 2);
        return;
    }

    if (fabsf(appCTRL.Velocity) > 0.05f || fabsf(appCTRL.SteerVelocity) > 0.05f)
    {
        Serial.println(F("ARM rejected: release both gamepad sticks first."));
        return;
    }

    uprightZero = carCTRL.MPUangleX;
    carCTRL.TargetVelocity = 0.0f;
    carCTRL.TargetAngle = 0.0f;
    carCTRL.MotorVelocity = 0.0f;
    motor_A.target = 0.0f;
    motor_B.target = 0.0f;
    velocityCount = 0;
    angleCount = 0;

    Velocity_PID.reset();
    UprightPID_Init(&upRightPIDConfig, UPRIGHT_Kp, UPRIGHT_Ki, UPRIGHT_Kd, UPRIGHT_LIMIT);
    motor_A.enable();
    motor_B.enable();
    armed = true;

    Serial.print(F("ARMED: upright zero="));
    Serial.println(uprightZero, 2);
}

void SuperCar::disarm(const char *reason)
{
    motor_A.target = 0.0f;
    motor_B.target = 0.0f;
    motor_A.disable();
    motor_B.disable();
    Velocity_PID.reset();
    UprightPID_Init(&upRightPIDConfig, UPRIGHT_Kp, UPRIGHT_Ki, UPRIGHT_Kd, UPRIGHT_LIMIT);

    const bool wasArmed = armed;
    armed = false;
    if (wasArmed)
    {
        Serial.print(F("DISARMED: "));
        Serial.println(reason);
    }
}

void SuperCar::printTelemetry(float relativeAngle)
{
    const unsigned long now = millis();
    if (now - lastTelemetryMs < TELEMETRY_INTERVAL_MS)
    {
        return;
    }
    lastTelemetryMs = now;

    Serial.print(armed ? F("ARMED") : F("STANDBY"));
    Serial.print(F(" angle="));
    Serial.print(carCTRL.MPUangleX, 2);
    Serial.print(F(" relative="));
    Serial.print(relativeAngle, 2);
    Serial.print(F(" gyro="));
    Serial.print(carCTRL.MPUgyroX, 2);
    Serial.print(F(" cmdV="));
    Serial.print(appCTRL.Velocity, 2);
    Serial.print(F(" steer="));
    Serial.print(appCTRL.SteerVelocity, 2);
    Serial.print(F(" speed="));
    Serial.print((carCTRL.MB_Velocity - carCTRL.MA_Velocity) * 0.5f, 2);
    Serial.print(F(" targetA="));
    Serial.print(motor_A.target, 2);
    Serial.print(F(" targetB="));
    Serial.println(motor_B.target, 2);
}

void SuperCar::running()
{
    if (!ready)
    {
        disarmRequested = false;
        if (armToggleRequested)
        {
            armToggleRequested = false;
            Serial.println(F("ARM rejected: initialization is not ready."));
        }
        delay(1);
        return;
    }

    motor_A.loopFOC();
    motor_B.loopFOC();

    mpu6050.update();
    carCTRL.MPUangleX = mpu6050.getAngleX();
    carCTRL.MPUgyroX = mpu6050.getGyroXFV();
    carCTRL.MA_Velocity = motor_A.shaft_velocity;
    carCTRL.MB_Velocity = motor_B.shaft_velocity;

    if (disarmRequested)
    {
        disarmRequested = false;
        armToggleRequested = false;
        disarm("button/power request");
    }
    else if (armToggleRequested)
    {
        armToggleRequested = false;
        if (armed)
        {
            disarm("START button");
        }
        else
        {
            armIfSafe();
        }
    }

    const float relativeAngle = carCTRL.MPUangleX - uprightZero;
    if (armed && fabsf(relativeAngle) > FALL_ANGLE_LIMIT)
    {
        disarm("tilt protection");
    }

    if (armed)
    {
        if (++velocityCount >= VEL_PID_UPDATE)
        {
            const float forwardVelocity = (carCTRL.MB_Velocity - carCTRL.MA_Velocity) * 0.5f;
            carCTRL.TargetVelocity = constrain(appCTRL.Velocity, -GAMEPAD_FORWARD_LIMIT, GAMEPAD_FORWARD_LIMIT);
            carCTRL.VelocityError = carCTRL.TargetVelocity - forwardVelocity;
            carCTRL.TargetAngle = Velocity_PID(carCTRL.VelocityError);
            velocityCount = 0;
        }

        if (++angleCount >= UPRIGHT_PID_UPDATE)
        {
            const float uprightOutput = UprightPID(&upRightPIDConfig, carCTRL.TargetAngle, relativeAngle,
                                                    carCTRL.MPUgyroX);
            carCTRL.MotorVelocity = BALANCE_DIRECTION * uprightOutput;
            angleCount = 0;
        }

        carCTRL.SteerVelocity = constrain(appCTRL.SteerVelocity, -GAMEPAD_STEER_LIMIT, GAMEPAD_STEER_LIMIT);
        motor_A.target = constrain(-carCTRL.MotorVelocity + carCTRL.SteerVelocity,
                                   -MOTOR_VELOCITY_LIMIT, MOTOR_VELOCITY_LIMIT);
        motor_B.target = constrain(carCTRL.MotorVelocity + carCTRL.SteerVelocity,
                                   -MOTOR_VELOCITY_LIMIT, MOTOR_VELOCITY_LIMIT);
        motor_A.move(motor_A.target);
        motor_B.move(motor_B.target);
    }

    printTelemetry(relativeAngle);
}

SuperCar BalanceCar;
