#include <Arduino.h>

#include "final_control.h"

void setup()
{
    Serial.begin(115200);

    // Do not delay here. finalControl.begin() first keeps the battery
    // power latch enabled, then initializes and checks every subsystem.
    finalControl.setDebug(true); // Consolidated wheel target/actual telemetry.
    finalControl.begin();
}

void loop()
{
    // The FOC and balance loops need to run as frequently as possible.
    finalControl.update();
}
