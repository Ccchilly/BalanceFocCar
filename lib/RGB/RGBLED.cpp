#include "RGBLED.h"

void RGBLED_Class::begin()
{
    rlStrip.begin();
    fbStrip.begin();

    rlStrip.setBrightness(RGB_LED_BRIGHTNESS);
    fbStrip.setBrightness(RGB_LED_BRIGHTNESS);

    off();
}

void RGBLED_Class::fillStrip(Adafruit_NeoPixel &strip,
                             uint16_t count,
                             uint8_t red,
                             uint8_t green,
                             uint8_t blue)
{
    for (uint16_t index = 0; index < count; ++index)
    {
        strip.setPixelColor(index, red, green, blue);
    }

    strip.show();
}

void RGBLED_Class::setColor(uint8_t red, uint8_t green, uint8_t blue)
{
    fillStrip(rlStrip, RGB_RL_COUNT, red, green, blue);
    fillStrip(fbStrip, RGB_FB_COUNT, red, green, blue);
}

void RGBLED_Class::setWhite()
{
    setColor(255, 255, 255);
}

void RGBLED_Class::setRed()
{
    setColor(255, 0, 0);
}

void RGBLED_Class::setGreen()
{
    setColor(0, 255, 0);
}

void RGBLED_Class::setYellow()
{
    setColor(255, 180, 0);
}

void RGBLED_Class::off()
{
    setColor(0, 0, 0);
}

void RGBLED_Class::RGB_Init()
{
    begin();
}

void RGBLED_Class::turn_off()
{
    off();
}

RGBLED_Class rgb;
