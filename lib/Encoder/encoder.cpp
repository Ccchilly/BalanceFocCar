#include "encoder.h"

#include <cmath>

namespace
{
constexpr uint8_t AS5600_ADDRESS = 0x36;
constexpr uint8_t REG_STATUS = 0x0B;
constexpr uint8_t REG_RAW_ANGLE_HIGH = 0x0C;

constexpr uint8_t STATUS_MAGNET_DETECTED = 0x20;
constexpr uint8_t STATUS_MAGNET_TOO_WEAK = 0x10;
constexpr uint8_t STATUS_MAGNET_TOO_STRONG = 0x08;

constexpr float COUNTS_PER_REVOLUTION = 4096.0f;
constexpr unsigned long MAGNET_CHECK_INTERVAL_MS = 20;
constexpr unsigned long STATUS_PRINT_INTERVAL_MS = 200;
}

encoder_control::encoder_control(TwoWire &wire, const char *name)
    : wire_(wire), name_(name)
{
}

bool encoder_control::begin(int sdaPin, int sclPin, uint32_t frequencyHz)
{
    if (!wire_.begin(sdaPin, sclPin, frequencyHz))
    {
        Serial.print(name_);
        Serial.println(F(" I2C start failed"));
        return false;
    }

    return startSensor();
}

bool encoder_control::begin()
{
    return startSensor();
}

bool encoder_control::startSensor()
{
    ready_ = false;
    data_ = encoder_data{};
    magnetStatus_ = 0;
    angleReadOk_ = false;
    statusReadOk_ = false;

    uint16_t rawAngle = 0;
    if (!readRawAngle(rawAngle))
    {
        Serial.print(name_);
        Serial.println(F(" AS5600 not found at 0x36"));
        return false;
    }

    if (!refreshMagnetStatus())
    {
        Serial.print(name_);
        Serial.println(F(" AS5600 status read failed"));
        return false;
    }

    // Initialize SimpleFOC's turn counter and velocity timestamps.
    Sensor::init();
    if (!angleReadOk_)
    {
        Serial.print(name_);
        Serial.println(F(" AS5600 angle initialization failed"));
        return false;
    }

    ready_ = true;
    lastMagnetCheckMs_ = millis();
    lastStatusPrintMs_ = millis();
    update();

    if (debug_)
    {
        Serial.print(name_);
        Serial.print(F(" AS5600 ready, magnet="));
        Serial.println(data_.magnetOk ? F("OK") : F("NOT_OK"));
    }

    return true;
}

void encoder_control::update()
{
    if (!ready_)
    {
        return;
    }

    angleReadOk_ = false;
    Sensor::update();

    const unsigned long now = millis();
    if (now - lastMagnetCheckMs_ >= MAGNET_CHECK_INTERVAL_MS)
    {
        lastMagnetCheckMs_ = now;
        refreshMagnetStatus();
    }

    if (angleReadOk_)
    {
        data_.angle = Sensor::getAngle();
        data_.velocity = Sensor::getVelocity();
    }

    data_.valid = angleReadOk_ && statusReadOk_ && data_.magnetOk &&
                  std::isfinite(data_.angle) && std::isfinite(data_.velocity);

    if (debug_)
    {
        printStatus(now);
    }
}

const encoder_data &encoder_control::getData() const
{
    return data_;
}

bool encoder_control::isReady() const
{
    return ready_;
}

uint8_t encoder_control::getMagnetStatus() const
{
    return magnetStatus_;
}

void encoder_control::setDebug(bool enabled)
{
    debug_ = enabled;
}

float encoder_control::getSensorAngle()
{
    uint16_t rawAngle = 0;
    angleReadOk_ = readRawAngle(rawAngle);

    if (!angleReadOk_)
    {
        return -1.0f;
    }

    return static_cast<float>(rawAngle) * _2PI / COUNTS_PER_REVOLUTION;
}

bool encoder_control::readBytes(uint8_t startRegister, uint8_t *buffer, size_t length)
{
    wire_.beginTransmission(AS5600_ADDRESS);
    wire_.write(startRegister);
    if (wire_.endTransmission(false) != 0)
    {
        return false;
    }

    if (wire_.requestFrom(AS5600_ADDRESS, length, true) != length)
    {
        return false;
    }

    for (size_t index = 0; index < length; ++index)
    {
        buffer[index] = wire_.read();
    }

    return true;
}

bool encoder_control::readRawAngle(uint16_t &rawAngle)
{
    uint8_t bytes[2] = {};
    if (!readBytes(REG_RAW_ANGLE_HIGH, bytes, sizeof(bytes)))
    {
        return false;
    }

    rawAngle = (static_cast<uint16_t>(bytes[0] & 0x0F) << 8) | bytes[1];
    return true;
}

bool encoder_control::refreshMagnetStatus()
{
    uint8_t status = 0;
    statusReadOk_ = readBytes(REG_STATUS, &status, 1);

    if (!statusReadOk_)
    {
        data_.magnetOk = false;
        return false;
    }

    magnetStatus_ = status;

    const bool detected = (status & STATUS_MAGNET_DETECTED) != 0;
    const bool tooWeak = (status & STATUS_MAGNET_TOO_WEAK) != 0;
    const bool tooStrong = (status & STATUS_MAGNET_TOO_STRONG) != 0;
    data_.magnetOk = detected && !tooWeak && !tooStrong;
    return true;
}

void encoder_control::printStatus(unsigned long now)
{
    if (now - lastStatusPrintMs_ < STATUS_PRINT_INTERVAL_MS)
    {
        return;
    }

    lastStatusPrintMs_ = now;

    Serial.print(name_);
    Serial.print(F(" angle="));
    Serial.print(data_.angle, 3);
    Serial.print(F(" rad  velocity="));
    Serial.print(data_.velocity, 3);
    Serial.print(F(" rad/s  status=0x"));
    if (magnetStatus_ < 0x10)
    {
        Serial.print('0');
    }
    Serial.print(magnetStatus_, HEX);
    Serial.print(F("  magnet="));
    Serial.print(data_.magnetOk ? F("OK") : F("NOT_OK"));
    Serial.print(F("  "));
    Serial.println(data_.valid ? F("ENCODER=OK") : F("ENCODER=INVALID"));
}
