#ifndef BALANCE_CAR_IMU_H
#define BALANCE_CAR_IMU_H

#include <Arduino.h>
#include <Wire.h>

#include "config.h"

class imu_control
{
public:
    imu_control();

    // Start I2C, configure the MPU6050 and optionally calibrate its gyroscope.
    // Keep the car completely still while calibration is running.
    bool begin(bool calibrateAtStartup = true);

    // Recalculate the gyroscope zero offset while the car is stationary.
    bool calibrate(uint16_t sampleCount = 1000);

    // Read one sample and update the filtered pitch angle and pitch rate.
    void update();

    const imu_data &getData() const;
    TwoWire &bus();
    bool isReady() const;
    void setDebug(bool enabled);

private:
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t &value);
    bool readRaw(int16_t &accX, int16_t &accY, int16_t &accZ,
                 int16_t &gyroX, int16_t &gyroY, int16_t &gyroZ);
    void printStatus(unsigned long now);

    TwoWire wire_;
    imu_data data_{};

    float gyroOffsetX_ = 0.0f;
    float gyroOffsetY_ = 0.0f;
    float gyroOffsetZ_ = 0.0f;
    float filteredGyroX_ = 0.0f;

    uint32_t lastUpdateUs_ = 0;
    unsigned long lastStatusPrintMs_ = 0;
    bool ready_ = false;
    bool debug_ = true;
};

#endif
