#include "buzzer.h"

#include "../Config/config.h"

namespace
{
constexpr uint8_t BUZZER_CHANNEL = 7;
constexpr uint32_t BUZZER_BASE_FREQUENCY = 2000;
constexpr uint8_t BUZZER_RESOLUTION = 8;

constexpr uint16_t NOTE_LOW = 440;
constexpr uint16_t NOTE_MIDDLE = 880;
constexpr uint16_t NOTE_HIGH = 1760;
}

void Buzzer_Class::begin(int buzzerPin)
{
    buzzerPin_ = buzzerPin;

    if (playMutex_ == nullptr)
    {
        playMutex_ = xSemaphoreCreateMutex();
    }

#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(buzzerPin_, BUZZER_BASE_FREQUENCY, BUZZER_RESOLUTION);
#else
    ledcSetup(BUZZER_CHANNEL, BUZZER_BASE_FREQUENCY, BUZZER_RESOLUTION);
    ledcAttachPin(buzzerPin_, BUZZER_CHANNEL);
#endif

    noTone(buzzerPin_);
}

void Buzzer_Class::Buzzer_Init(int buzzerPin)
{
    begin(buzzerPin);
}

bool Buzzer_Class::lock()
{
    return playMutex_ == nullptr ||
           xSemaphoreTake(playMutex_, portMAX_DELAY) == pdTRUE;
}

void Buzzer_Class::unlock()
{
    if (playMutex_ != nullptr)
    {
        xSemaphoreGive(playMutex_);
    }
}

void Buzzer_Class::toneFor(uint16_t frequency,
                           uint16_t durationMs,
                           uint16_t silenceMs)
{
    if (buzzerPin_ < 0)
    {
        return;
    }

#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWriteTone(buzzerPin_, frequency);
    delay(durationMs);
    ledcWriteTone(buzzerPin_, 0);
#else
    ledcWriteTone(BUZZER_CHANNEL, frequency);
    delay(durationMs);
    ledcWriteTone(BUZZER_CHANNEL, 0);
#endif

    delay(silenceMs);
}

void Buzzer_Class::playLocked(int soundId)
{
    switch (soundId)
    {
    case S_CONNECTION:
        toneFor(NOTE_MIDDLE, 60, 25);
        toneFor(NOTE_HIGH, 90, 20);
        break;

    case S_DISCONNECTION:
        toneFor(NOTE_HIGH, 70, 25);
        toneFor(NOTE_MIDDLE, 100, 20);
        break;

    case S_POWER_ON:
        toneFor(NOTE_LOW, 70, 25);
        toneFor(NOTE_MIDDLE, 100, 20);
        break;

    case S_POWER_OFF:
        toneFor(NOTE_MIDDLE, 90, 25);
        toneFor(NOTE_LOW, 130, 20);
        break;

    case S_BEEP:
        toneFor(NOTE_MIDDLE, 100, 20);
        break;

    case S_SIREN:
        toneFor(NOTE_LOW, 180, 40);
        toneFor(NOTE_HIGH, 180, 40);
        toneFor(NOTE_LOW, 180, 20);
        break;

    default:
        // 未定义的声音编号不发声。
        break;
    }
}

void Buzzer_Class::play(int soundId)
{
    if (!lock())
    {
        return;
    }

    playLocked(soundId);
    unlock();
}

void Buzzer_Class::powerOn()
{
    play(S_POWER_ON);
}

void Buzzer_Class::powerOff()
{
    play(S_POWER_OFF);
}

void Buzzer_Class::connection()
{
    play(S_CONNECTION);
}

void Buzzer_Class::disconnection()
{
    play(S_DISCONNECTION);
}

void Buzzer_Class::beep()
{
    play(S_BEEP);
}

void Buzzer_Class::siren()
{
    play(S_SIREN);
}

Buzzer_Class buzzer;
