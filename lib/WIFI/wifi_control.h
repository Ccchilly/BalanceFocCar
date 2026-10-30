#ifndef WIFI_CONTROL_H
#define WIFI_CONTROL_H

#include <Arduino.h>
#include <WiFiUdp.h>
#include "config.h"

class wifi_control
{
public:
    bool begin();
    void update();

    const remote_data &getCommand() const;
    bool isReady() const;
    IPAddress ip() const;
    void setDebug(bool enabled);

private:
    void clearCommand();
    void printStatus(unsigned long now);

    WiFiUDP udp_;
    remote_data command_{};
    unsigned long lastPacketMs_ = 0;
    unsigned long lastStatusPrintMs_ = 0;
    bool ready_ = false;
    bool debug_ = true;
};

#endif
