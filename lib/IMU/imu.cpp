#include "imu.h"

#include <cmath>

namespace
{
constexpr uint8_t MPU_ADDRESS = 0x68;

constexpr uint8_t REG_SAMPLE_RATE_DIVIDER = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_DATA = 0x3B;
constexpr uint8_t REG_POWER_MANAGEMENT = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;

constexpr uint32_t I2C_FREQUENCY_HZ = 400000;
constexpr float ACCEL_SCALE = 16384.0f; // +/-2 g
constexpr float GYRO_SCALE = 65.5f;     // +/-500 deg/s
constexpr float COMPLEMENTARY_GYRO_WEIGHT = 0.975f;
constexpr float GYRO_LOW_PASS_WEIGHT = 0.85f;
constexpr float MAX_UPDATE_INTERVAL_S = 0.05f;
constexpr unsigned long STATUS_PRINT_INTERVAL_MS = 200;

int16_t combineBytes(uint8_t highByte, uint8_t lowByte)
{
    return static_cast<int16_t>((static_cast<uint16_t>(highByte) << 8) | lowByte);
}

float accelerationPitch(int16_t rawAccX, int16_t rawAccY, int16_t rawAccZ)
{
    const float accX = static_cast<float>(rawAccX) / ACCEL_SCALE;
    const float accY = static_cast<float>(rawAccY) / ACCEL_SCALE;
    const float accZ = static_cast<float>(rawAccZ) / ACCEL_SCALE;
    return std::atan2(accY, accZ + std::fabs(accX)) * 180.0f / PI;
}
}

imu_control::imu_control() : wire_(0)
{
}

bool imu_control::begin(bool calibrateAtStartup)
{
    ready_ = false;
    data_ = imu_data{};
    gyroOffsetX_ = 0.0f;
    gyroOffsetY_ = 0.0f;
    gyroOffsetZ_ = 0.0f;
    filteredGyroX_ = 0.0f;

    if (!wire_.begin(pins::MPU_SDA, pins::MPU_SCL, I2C_FREQUENCY_HZ))
    {
        Serial.println(F("IMU I2C start failed"));
        return false;
    }

    delay(50);

    uint8_t deviceId = 0;
    if (!readRegister(REG_WHO_AM_I, deviceId) || deviceId != MPU_ADDRESS)
    {
        Serial.print(F("MPU6050 not found, WHO_AM_I=0x"));
        Serial.println(deviceId, HEX);
        return false;
    }

    // Reset, select the X-axis gyro clock, use a 42 Hz digital filter,
    // sample at 200 Hz, and configure +/-500 deg/s and +/-2 g ranges.
    if (!writeRegister(REG_POWER_MANAGEMENT, 0x80))
    {
        return false;
    }
    delay(100);

    if (!writeRegister(REG_POWER_MANAGEMENT, 0x01) ||
        !writeRegister(REG_CONFIG, 0x03) ||
        !writeRegister(REG_SAMPLE_RATE_DIVIDER, 0x04) ||
        !writeRegister(REG_GYRO_CONFIG, 0x08) ||
        !writeRegister(REG_ACCEL_CONFIG, 0x00))
    {
        Serial.println(F("MPU6050 configuration failed"));
        return false;
    }

    delay(100);

    if (calibrateAtStartup && !calibrate())
    {
        Serial.println(F("MPU6050 calibration failed"));
        return false;
    }

    int16_t accX = 0;
    int16_t accY = 0;
    int16_t accZ = 0;
    int16_t gyroX = 0;
    int16_t gyroY = 0;
    int16_t gyroZ = 0;

    if (!readRaw(accX, accY, accZ, gyroX, gyroY, gyroZ))
    {
        Serial.println(F("MPU6050 first sample failed"));
        return false;
    }

    data_.pitch = accelerationPitch(accX, accY, accZ);
    data_.gyro = 0.0f;
    data_.valid = true;
    lastUpdateUs_ = micros();
    lastStatusPrintMs_ = millis();
    ready_ = true;

    if (debug_)
    {
        Serial.println(F("MPU6050 ready"));
        Serial.print(F("Gyro X offset: "));
        Serial.println(gyroOffsetX_, 3);
    }

    return true;
}

bool imu_control::calibrate(uint16_t sampleCount)
{
    if (sampleCount == 0)
    {
        return false;
    }

    if (debug_)
    {
        Serial.println(F("Calibrating MPU6050. Keep the car still..."));
    }

    double sumX = 0.0;
    double sumY = 0.0;
    double sumZ = 0.0;

    for (uint16_t sample = 0; sample < sampleCount; ++sample)
    {
        int16_t accX = 0;
        int16_t accY = 0;
        int16_t accZ = 0;
        int16_t gyroX = 0;
        int16_t gyroY = 0;
        int16_t gyroZ = 0;

        if (!readRaw(accX, accY, accZ, gyroX, gyroY, gyroZ))
        {
            data_.valid = false;
            return false;
        }

        sumX += static_cast<float>(gyroX) / GYRO_SCALE;
        sumY += static_cast<float>(gyroY) / GYRO_SCALE;
        sumZ += static_cast<float>(gyroZ) / GYRO_SCALE;
        delay(2);
    }

    gyroOffsetX_ = static_cast<float>(sumX / sampleCount);
    gyroOffsetY_ = static_cast<float>(sumY / sampleCount);
    gyroOffsetZ_ = static_cast<float>(sumZ / sampleCount);
    filteredGyroX_ = 0.0f;

    if (debug_)
    {
        Serial.println(F("MPU6050 calibration complete"));
    }

    return true;
}

void imu_control::update()
{
    if (!ready_)
    {
        return;
    }

    int16_t accX = 0;
    int16_t accY = 0;
    int16_t accZ = 0;
    int16_t gyroX = 0;
    int16_t gyroY = 0;
    int16_t gyroZ = 0;

    if (!readRaw(accX, accY, accZ, gyroX, gyroY, gyroZ))
    {
        data_.valid = false;
        return;
    }

    const uint32_t nowUs = micros();
    const float interval = static_cast<float>(nowUs - lastUpdateUs_) * 0.000001f;
    lastUpdateUs_ = nowUs;

    if (interval <= 0.0f || interval > MAX_UPDATE_INTERVAL_S)
    {
        data_.valid = false;
        return;
    }

    const float accelPitch = accelerationPitch(accX, accY, accZ);
    const float gyroRateX = static_cast<float>(gyroX) / GYRO_SCALE - gyroOffsetX_;

    filteredGyroX_ = GYRO_LOW_PASS_WEIGHT * filteredGyroX_ +
                     (1.0f - GYRO_LOW_PASS_WEIGHT) * gyroRateX;

    data_.pitch = COMPLEMENTARY_GYRO_WEIGHT *
                      (data_.pitch + gyroRateX * interval) +
                  (1.0f - COMPLEMENTARY_GYRO_WEIGHT) * accelPitch;
    data_.gyro = filteredGyroX_;
    data_.valid = std::isfinite(data_.pitch) && std::isfinite(data_.gyro);

    if (debug_)
    {
        printStatus(millis());
    }
}

const imu_data &imu_control::getData() const
{
    return data_;
}

TwoWire &imu_control::bus()
{
    return wire_;
}

bool imu_control::isReady() const
{
    return ready_;
}

void imu_control::setDebug(bool enabled)
{
    debug_ = enabled;
}

bool imu_control::writeRegister(uint8_t reg, uint8_t value)
{
    wire_.beginTransmission(MPU_ADDRESS);
    wire_.write(reg);
    wire_.write(value);
    return wire_.endTransmission(true) == 0;
}

bool imu_control::readRegister(uint8_t reg, uint8_t &value)
{
    wire_.beginTransmission(MPU_ADDRESS);
    wire_.write(reg);
    if (wire_.endTransmission(false) != 0)
    {
        return false;
    }

    if (wire_.requestFrom(MPU_ADDRESS, static_cast<size_t>(1), true) != 1)
    {
        return false;
    }

    value = wire_.read();
    return true;
}

bool imu_control::readRaw(int16_t &accX, int16_t &accY, int16_t &accZ,
                          int16_t &gyroX, int16_t &gyroY, int16_t &gyroZ)
{
    wire_.beginTransmission(MPU_ADDRESS);
    wire_.write(REG_ACCEL_DATA);
    if (wire_.endTransmission(false) != 0)
    {
        return false;
    }

    constexpr uint8_t BYTE_COUNT = 14;
    if (wire_.requestFrom(MPU_ADDRESS, static_cast<size_t>(BYTE_COUNT), true) != BYTE_COUNT)
    {
        return false;
    }

    uint8_t bytes[BYTE_COUNT] = {};
    for (uint8_t index = 0; index < BYTE_COUNT; ++index)
    {
        bytes[index] = wire_.read();
    }

    accX = combineBytes(bytes[0], bytes[1]);
    accY = combineBytes(bytes[2], bytes[3]);
    accZ = combineBytes(bytes[4], bytes[5]);
    gyroX = combineBytes(bytes[8], bytes[9]);
    gyroY = combineBytes(bytes[10], bytes[11]);
    gyroZ = combineBytes(bytes[12], bytes[13]);
    return true;
}

void imu_control::printStatus(unsigned long now)
{
    if (now - lastStatusPrintMs_ < STATUS_PRINT_INTERVAL_MS)
    {
        return;
    }

    lastStatusPrintMs_ = now;

    Serial.print(F("pitch="));
    Serial.print(data_.pitch, 2);
    Serial.print(F(" deg  gyro="));
    Serial.print(data_.gyro, 2);
    Serial.print(F(" deg/s  "));
    Serial.println(data_.valid ? F("IMU=OK") : F("IMU=INVALID"));
}
