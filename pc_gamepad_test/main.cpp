#define WIN32_LEAN_AND_MEAN

#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <Xinput.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

constexpr char ESP32_IP[] = "192.168.4.1";
constexpr unsigned short ESP32_PORT = 4210;

constexpr float STICK_DEADZONE = 0.12f;
constexpr float FORWARD_LIMIT = 1.5f;
constexpr float STEER_LIMIT = 1.0f;

float normalizeStick(SHORT raw)
{
    float value;

    if (raw < 0)
    {
        value = static_cast<float>(raw) / 32768.0f;
    }
    else
    {
        value = static_cast<float>(raw) / 32767.0f;
    }

    if (std::fabs(value) <= STICK_DEADZONE)
    {
        return 0.0f;
    }

    const float sign = value >= 0.0f ? 1.0f : -1.0f;

    float magnitude =
        (std::fabs(value) - STICK_DEADZONE) /
        (1.0f - STICK_DEADZONE);

    if (magnitude > 1.0f)
    {
        magnitude = 1.0f;
    }

    return sign * magnitude;
}

int main()
{
    WSADATA wsaData{};

    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
    {
        std::cerr << "WSAStartup failed\n";
        return 1;
    }

    SOCKET udpSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (udpSocket == INVALID_SOCKET)
    {
        std::cerr << "Could not create UDP socket\n";
        WSACleanup();
        return 1;
    }

    sockaddr_in esp32Address{};
    esp32Address.sin_family = AF_INET;
    esp32Address.sin_port = htons(ESP32_PORT);

    if (inet_pton(AF_INET, ESP32_IP, &esp32Address.sin_addr) != 1)
    {
        std::cerr << "Invalid ESP32 IP address\n";
        closesocket(udpSocket);
        WSACleanup();
        return 1;
    }

    std::cout << "PC gamepad to ESP32 WiFi test\n";
    std::cout << "Move the left stick.\n";
    std::cout << "Press ESC to stop.\n\n";

    while ((GetAsyncKeyState(VK_ESCAPE) & 0x8000) == 0)
    {
        XINPUT_STATE state{};
        bool controllerFound = false;

        for (DWORD index = 0; index < XUSER_MAX_COUNT; ++index)
        {
            if (XInputGetState(index, &state) == ERROR_SUCCESS)
            {
                controllerFound = true;
                break;
            }
        }

        float velocity = 0.0f;
        float steer = 0.0f;

        if (controllerFound)
        {
            const XINPUT_GAMEPAD &pad = state.Gamepad;

            const float leftX = normalizeStick(pad.sThumbLX);
            const float leftY = normalizeStick(pad.sThumbLY);

            // 摇杆向前通常是负值，因此取反
            velocity = -leftY * FORWARD_LIMIT;
            steer = leftX * STEER_LIMIT;
        }

        std::ostringstream message;
        message << std::fixed << std::setprecision(3)
                << velocity << "," << steer << "\n";

        const std::string packet = message.str();

        sendto(
            udpSocket,
            packet.c_str(),
            static_cast<int>(packet.length()),
            0,
            reinterpret_cast<const sockaddr *>(&esp32Address),
            sizeof(esp32Address));

        std::cout << "\rcontroller="
                  << (controllerFound ? "YES" : "NO")
                  << "  velocity=" << std::fixed << std::setprecision(2)
                  << std::setw(5) << velocity
                  << "  steer=" << std::setw(5) << steer
                  << "                  "
                  << std::flush;

        Sleep(20);
    }

    // 退出前发送几次停止命令
    for (int i = 0; i < 5; ++i)
    {
        const char stopPacket[] = "0.000,0.000\n";

        sendto(
            udpSocket,
            stopPacket,
            sizeof(stopPacket) - 1,
            0,
            reinterpret_cast<const sockaddr *>(&esp32Address),
            sizeof(esp32Address));

        Sleep(20);
    }

    closesocket(udpSocket);
    WSACleanup();

    std::cout << "\nStopped.\n";
    return 0;
}