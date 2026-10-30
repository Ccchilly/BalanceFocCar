#ifndef BALANCE_CAR_BUZZER_SOUND_H
#define BALANCE_CAR_BUZZER_SOUND_H

#include <Arduino.h>
#include <freertos/semphr.h>

// 只保留平衡车需要的提示音编号。
// 4、5、21、22 是旧 Button 模块正在使用的编号，暂时保持不变。
constexpr int S_CONNECTION = 0;
constexpr int S_DISCONNECTION = 1;
constexpr int S_POWER_ON = 4;
constexpr int S_POWER_OFF = 5;
constexpr int S_BEEP = 21;
constexpr int S_SIREN = 22;

class Buzzer_Class
{
public:
    // 推荐的新接口。
    void begin(int buzzerPin);
    void powerOn();
    void powerOff();
    void connection();
    void disconnection();
    void beep();
    void siren();

    // 兼容旧 Button 模块。
    void Buzzer_Init(int buzzerPin);
    void play(int soundId);

private:
    void toneFor(uint16_t frequency, uint16_t durationMs,
                 uint16_t silenceMs = 10);
    void playLocked(int soundId);
    bool lock();
    void unlock();

    int buzzerPin_ = -1;
    SemaphoreHandle_t playMutex_ = nullptr;
};

extern Buzzer_Class buzzer;

#endif
