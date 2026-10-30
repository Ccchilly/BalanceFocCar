#ifndef BALANCE_CAR_CONFIG_H
#define BALANCE_CAR_CONFIG_H

#include <cstdint>

// 控制数据
struct remote_data
{
    float velocity = 0.0f;
    float steer = 0.0f;
    bool balanceHeld = false;
    bool connected = false;
};

struct imu_data
{
    float pitch = 0.0f;
    float gyro = 0.0f;
    bool valid = false;
};

struct encoder_data
{
    float angle = 0.0f;
    float velocity = 0.0f;
    bool magnetOk = false;
    bool valid = false;
};

// ESP32 引脚
namespace pins
{
constexpr int BUZZER = 40;
constexpr int RGB_RL = 38;
constexpr int RGB_FB = 39;

constexpr int MPU_SDA = 8;
constexpr int MPU_SCL = 9;
}

// WiFi 参数
namespace wifi_config
{
constexpr char SSID[] = "BalanceCar";
constexpr char PASSWORD[] = "12345678";
constexpr uint16_t UDP_PORT = 4210;
constexpr unsigned long COMMAND_TIMEOUT_MS = 300;
}

// 小车安全参数
namespace car_config
{
constexpr float MAX_VELOCITY = 1.5f;
constexpr float MAX_STEER = 1.0f;
constexpr float MAX_SAFE_TILT_DEG = 10.0f;
}

#endif