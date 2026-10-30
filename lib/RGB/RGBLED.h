#ifndef BALANCE_CAR_RGBLED_H
#define BALANCE_CAR_RGBLED_H

#include <Adafruit_NeoPixel.h>
#include <cstdint>

#include "../Config/config.h"

// 两路灯带的实际灯珠数量。
constexpr uint16_t RGB_RL_COUNT = 6;
constexpr uint16_t RGB_FB_COUNT = 8;
constexpr uint8_t RGB_LED_BRIGHTNESS = 50;

class RGBLED_Class
{
public:
    // 初始化灯带，并默认熄灭。
    void begin();

    // 设置两路灯带为同一种 RGB 颜色。
    void setColor(uint8_t red, uint8_t green, uint8_t blue);

    void setWhite();
    void setRed();
    void setGreen();
    void setYellow();
    void off();

    // 兼容现有 Button 模块的旧接口。
    void RGB_Init();
    void turn_off();

private:
    void fillStrip(Adafruit_NeoPixel &strip,
                   uint16_t count,
                   uint8_t red,
                   uint8_t green,
                   uint8_t blue);

    Adafruit_NeoPixel rlStrip{
        RGB_RL_COUNT,
        pins::RGB_RL,
        NEO_GRB + NEO_KHZ800};

    Adafruit_NeoPixel fbStrip{
        RGB_FB_COUNT,
        pins::RGB_FB,
        NEO_GRB + NEO_KHZ800};
};

extern RGBLED_Class rgb;

#endif
