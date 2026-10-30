#include "wifi_control.h"

#include <WiFi.h>
#include <cmath>
#include <cstdio>

namespace
{
constexpr unsigned long STATUS_PRINT_INTERVAL_MS = 200;
}

bool wifi_control::begin()
{
    command_ = remote_data{};
    lastPacketMs_ = 0;
    ready_ = false;
    lastStatusPrintMs_ = millis();

    WiFi.mode(WIFI_AP);

    if (!WiFi.softAP(wifi_config::SSID, wifi_config::PASSWORD))
    {
        Serial.println(F("WiFi AP start failed"));
        return false;
    }

    if (!udp_.begin(wifi_config::UDP_PORT))
    {
        Serial.println(F("UDP start failed"));
        WiFi.softAPdisconnect(true);
        return false;
    }

    ready_ = true;

    if (debug_)
    {
        Serial.println();
        Serial.println(F("=== WiFi UDP control started ==="));
        Serial.print(F("WiFi name: "));
        Serial.println(wifi_config::SSID);
        Serial.print(F("IP address: "));
        Serial.println(WiFi.softAPIP());
        Serial.print(F("UDP port: "));
        Serial.println(wifi_config::UDP_PORT);
        Serial.print(F("Command timeout: "));
        Serial.print(wifi_config::COMMAND_TIMEOUT_MS);
        Serial.println(F(" ms"));
    }

    return true;
}

void wifi_control::update()
{
    if (!ready_)
    {
        return;
    }

    const unsigned long now = millis();

    int packetSize = udp_.parsePacket();
    while (packetSize > 0)
    {
        char buffer[64] = {};
        const int length = udp_.read(buffer, sizeof(buffer) - 1);

        if (length > 0)
        {
            buffer[length] = '\0';

            float velocity = 0.0f;
            float steer = 0.0f;
            int balanceHeld = 0;

            // Preferred format: velocity,steer,balanceHeld
            // Two-field packets remain valid but cannot enable balancing.
            const int fields = std::sscanf(buffer, "%f,%f,%d", &velocity, &steer, &balanceHeld);
            if (fields >= 2 && std::isfinite(velocity) && std::isfinite(steer))
            {
                command_.velocity = constrain(velocity, -car_config::MAX_VELOCITY,
                                               car_config::MAX_VELOCITY);
                command_.steer = constrain(steer, -car_config::MAX_STEER,
                                            car_config::MAX_STEER);
                command_.balanceHeld = fields == 3 && balanceHeld != 0;
                command_.connected = true;
                lastPacketMs_ = now;
            }
        }

        packetSize = udp_.parsePacket();
    }

    if (lastPacketMs_ == 0 || now - lastPacketMs_ > wifi_config::COMMAND_TIMEOUT_MS)
    {
        clearCommand();
    }

    if (debug_)
    {
        printStatus(now);
    }
}

const remote_data &wifi_control::getCommand() const
{
    return command_;
}

bool wifi_control::isReady() const
{
    return ready_;
}

IPAddress wifi_control::ip() const
{
    return WiFi.softAPIP();
}

void wifi_control::setDebug(bool enabled)
{
    debug_ = enabled;
}

void wifi_control::clearCommand()
{
    command_.velocity = 0.0f;
    command_.steer = 0.0f;
    command_.balanceHeld = false;
    command_.connected = false;
}

void wifi_control::printStatus(unsigned long now)
{
    if (now - lastStatusPrintMs_ < STATUS_PRINT_INTERVAL_MS)
    {
        return;
    }

    lastStatusPrintMs_ = now;

    Serial.print(F("velocity="));
    Serial.print(command_.velocity, 3);
    Serial.print(F("  steer="));
    Serial.print(command_.steer, 3);
    Serial.print(F("  balance="));
    Serial.print(command_.balanceHeld ? F("ON") : F("OFF"));
    Serial.print(F("  "));
    Serial.println(command_.connected ? F("LINK=OK") : F("SAFE_STOP"));
}
