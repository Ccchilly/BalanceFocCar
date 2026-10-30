#ifndef BALANCE_CAR_ENCODER_H
#define BALANCE_CAR_ENCODER_H

#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>

#include "config.h"

class encoder_control : public Sensor
{
public:
    explicit encoder_control(TwoWire &wire, const char *name = "Encoder");

    // Initialize this I2C bus, then start the AS5600.
    bool begin(int sdaPin, int sclPin, uint32_t frequencyHz = 400000);

    // Start the AS5600 on an I2C bus that another module already initialized.
    bool begin();

    // SimpleFOC calls this through Sensor. It may also be called manually.
    void update() override;

    const encoder_data &getData() const;
    bool isReady() const;
    uint8_t getMagnetStatus() const;
    void setDebug(bool enabled);

protected:
    float getSensorAngle() override;

private:
    bool startSensor();
    bool readBytes(uint8_t startRegister, uint8_t *buffer, size_t length);
    bool readRawAngle(uint16_t &rawAngle);
    bool refreshMagnetStatus();
    void printStatus(unsigned long now);

    TwoWire &wire_;
    const char *name_;
    encoder_data data_{};

    uint8_t magnetStatus_ = 0;
    unsigned long lastMagnetCheckMs_ = 0;
    unsigned long lastStatusPrintMs_ = 0;
    bool angleReadOk_ = false;
    bool statusReadOk_ = false;
    bool ready_ = false;
    bool debug_ = true;
};

#endif
