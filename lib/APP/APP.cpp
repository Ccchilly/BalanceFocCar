#include "APP.h"

#include "UserConfig.h"
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLEDevice.h>
#include <BLERemoteCharacteristic.h>
#include <BLERemoteDescriptor.h>
#include <BLERemoteService.h>
#include <BLESecurity.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <string>

namespace
{
constexpr uint16_t HID_SERVICE_UUID = 0x1812;
constexpr uint16_t HID_REPORT_MAP_UUID = 0x2A4B;
constexpr uint16_t HID_REPORT_UUID = 0x2A4D;
constexpr uint16_t HID_REPORT_REFERENCE_UUID = 0x2908;
constexpr uint8_t HID_REPORT_TYPE_INPUT = 1;
constexpr uint16_t HID_GAMEPAD_APPEARANCE = 0x03C4;
constexpr size_t MAX_REPORT_ENDPOINTS = 12;

struct HidAxisField
{
    bool valid = false;
    uint8_t reportId = 0;
    uint16_t bitOffset = 0;
    uint8_t bitSize = 0;
    int32_t logicalMin = 0;
    int32_t logicalMax = 0;
};

struct HidGlobalState
{
    uint32_t usagePage = 0;
    int32_t logicalMin = 0;
    int32_t logicalMax = 0;
    uint8_t reportSize = 0;
    uint8_t reportId = 0;
    uint16_t reportCount = 0;
};

class HidStickParser
{
  public:
    bool parse(const uint8_t *data, size_t length)
    {
        xAxis = HidAxisField{};
        yAxis = HidAxisField{};
        memset(reportBitOffset, 0, sizeof(reportBitOffset));

        HidGlobalState global;
        HidGlobalState globalStack[4];
        size_t stackDepth = 0;
        uint32_t usages[16] = {};
        size_t usageCount = 0;
        uint32_t usageMin = 0;
        uint32_t usageMax = 0;
        bool haveUsageRange = false;

        for (size_t index = 0; index < length;)
        {
            const uint8_t prefix = data[index++];
            if (prefix == 0xFE)
            {
                if (index + 2 > length)
                {
                    break;
                }
                const size_t longLength = data[index];
                index += 2;
                if (index + longLength > length)
                {
                    break;
                }
                index += longLength;
                continue;
            }

            size_t itemSize = prefix & 0x03;
            itemSize = (itemSize == 3) ? 4 : itemSize;
            if (index + itemSize > length)
            {
                break;
            }

            uint32_t value = 0;
            for (size_t byteIndex = 0; byteIndex < itemSize; ++byteIndex)
            {
                value |= static_cast<uint32_t>(data[index + byteIndex]) << (8 * byteIndex);
            }
            index += itemSize;

            const uint8_t type = (prefix >> 2) & 0x03;
            const uint8_t tag = (prefix >> 4) & 0x0F;
            const int32_t signedValue = signExtend(value, static_cast<uint8_t>(itemSize * 8));

            if (type == 1)
            {
                switch (tag)
                {
                case 0:
                    global.usagePage = value;
                    break;
                case 1:
                    global.logicalMin = signedValue;
                    break;
                case 2:
                    global.logicalMax = (global.logicalMin < 0) ? signedValue : static_cast<int32_t>(value);
                    break;
                case 7:
                    global.reportSize = static_cast<uint8_t>(value);
                    break;
                case 8:
                    global.reportId = static_cast<uint8_t>(value);
                    break;
                case 9:
                    global.reportCount = static_cast<uint16_t>(value);
                    break;
                case 10:
                    if (stackDepth < 4)
                    {
                        globalStack[stackDepth++] = global;
                    }
                    break;
                case 11:
                    if (stackDepth > 0)
                    {
                        global = globalStack[--stackDepth];
                    }
                    break;
                default:
                    break;
                }
                continue;
            }

            if (type == 2)
            {
                const uint32_t fullUsage = makeUsage(global.usagePage, value, itemSize);
                if (tag == 0 && usageCount < 16)
                {
                    usages[usageCount++] = fullUsage;
                }
                else if (tag == 1)
                {
                    usageMin = fullUsage;
                    haveUsageRange = true;
                }
                else if (tag == 2)
                {
                    usageMax = fullUsage;
                    haveUsageRange = true;
                }
                continue;
            }

            if (type != 0)
            {
                continue;
            }

            if (tag == 8)
            {
                const bool constant = (value & 0x01U) != 0;
                const bool variable = (value & 0x02U) != 0;
                const uint16_t startOffset = reportBitOffset[global.reportId];

                if (!constant && variable && global.reportSize > 0 && global.reportSize <= 32)
                {
                    for (uint16_t fieldIndex = 0; fieldIndex < global.reportCount; ++fieldIndex)
                    {
                        const uint32_t usage = usageForField(fieldIndex, usages, usageCount, usageMin, usageMax,
                                                             haveUsageRange);
                        const uint16_t bitOffset = startOffset + fieldIndex * global.reportSize;
                        setAxisIfNeeded(usage, global, bitOffset);
                    }
                }

                reportBitOffset[global.reportId] =
                    startOffset + static_cast<uint16_t>(global.reportSize) * global.reportCount;
            }

            usageCount = 0;
            usageMin = 0;
            usageMax = 0;
            haveUsageRange = false;
        }

        return xAxis.valid && yAxis.valid && xAxis.reportId == yAxis.reportId;
    }

    bool decode(uint8_t reportId, const uint8_t *data, size_t length, float &x, float &y) const
    {
        if (!isValid() || reportId != xAxis.reportId)
        {
            return false;
        }

        const uint16_t requiredBits = std::max<uint16_t>(xAxis.bitOffset + xAxis.bitSize,
                                                         yAxis.bitOffset + yAxis.bitSize);
        if (length * 8 < requiredBits)
        {
            return false;
        }

        x = normalize(extractValue(xAxis, data), xAxis);
        y = normalize(extractValue(yAxis, data), yAxis);
        return true;
    }

    bool isValid() const
    {
        return xAxis.valid && yAxis.valid && xAxis.reportId == yAxis.reportId;
    }

    uint8_t axisReportId() const
    {
        return isValid() ? xAxis.reportId : 0;
    }

    void printSummary() const
    {
        if (!isValid())
        {
            Serial.println(F("HID map read, but no standard X/Y stick fields were found."));
            return;
        }

        Serial.print(F("HID left stick: report="));
        Serial.print(xAxis.reportId);
        Serial.print(F(" X(bit="));
        Serial.print(xAxis.bitOffset);
        Serial.print(F(", size="));
        Serial.print(xAxis.bitSize);
        Serial.print(F(") Y(bit="));
        Serial.print(yAxis.bitOffset);
        Serial.print(F(", size="));
        Serial.print(yAxis.bitSize);
        Serial.println(F(")"));
    }

  private:
    HidAxisField xAxis;
    HidAxisField yAxis;
    uint16_t reportBitOffset[256] = {};

    static int32_t signExtend(uint32_t value, uint8_t bits)
    {
        if (bits == 0 || bits >= 32)
        {
            return static_cast<int32_t>(value);
        }
        const uint32_t signBit = 1UL << (bits - 1);
        return static_cast<int32_t>((value ^ signBit) - signBit);
    }

    static uint32_t makeUsage(uint32_t usagePage, uint32_t value, size_t itemSize)
    {
        return (itemSize == 4) ? value : ((usagePage & 0xFFFFU) << 16) | (value & 0xFFFFU);
    }

    static uint32_t usageForField(uint16_t index, const uint32_t *usages, size_t usageCount,
                                  uint32_t usageMin, uint32_t usageMax, bool haveUsageRange)
    {
        if (index < usageCount)
        {
            return usages[index];
        }
        if (usageCount > 0)
        {
            return usages[usageCount - 1];
        }
        if (haveUsageRange && usageMax >= usageMin)
        {
            return std::min<uint32_t>(usageMin + index, usageMax);
        }
        return 0;
    }

    void setAxisIfNeeded(uint32_t usage, const HidGlobalState &global, uint16_t bitOffset)
    {
        constexpr uint32_t GENERIC_DESKTOP_X = 0x00010030;
        constexpr uint32_t GENERIC_DESKTOP_Y = 0x00010031;
        HidAxisField *field = nullptr;

        if (usage == GENERIC_DESKTOP_X && !xAxis.valid)
        {
            field = &xAxis;
        }
        else if (usage == GENERIC_DESKTOP_Y && !yAxis.valid)
        {
            field = &yAxis;
        }

        if (field != nullptr && global.logicalMax > global.logicalMin)
        {
            field->valid = true;
            field->reportId = global.reportId;
            field->bitOffset = bitOffset;
            field->bitSize = global.reportSize;
            field->logicalMin = global.logicalMin;
            field->logicalMax = global.logicalMax;
        }
    }

    static int32_t extractValue(const HidAxisField &field, const uint8_t *data)
    {
        uint32_t value = 0;
        for (uint8_t bit = 0; bit < field.bitSize; ++bit)
        {
            const uint16_t sourceBit = field.bitOffset + bit;
            if ((data[sourceBit / 8] >> (sourceBit % 8)) & 0x01U)
            {
                value |= 1UL << bit;
            }
        }
        return (field.logicalMin < 0) ? signExtend(value, field.bitSize) : static_cast<int32_t>(value);
    }

    static float normalize(int32_t value, const HidAxisField &field)
    {
        const float center = (static_cast<float>(field.logicalMin) + field.logicalMax) * 0.5f;
        const float distance = static_cast<float>(value) - center;
        const float range = (distance >= 0.0f) ? (field.logicalMax - center) : (center - field.logicalMin);
        return (range > 0.0f) ? constrain(distance / range, -1.0f, 1.0f) : 0.0f;
    }
};

struct ReportEndpoint
{
    BLERemoteCharacteristic *characteristic = nullptr;
    uint8_t reportId = 0;
    bool reportIdKnown = false;
    bool allowDecode = false;
};

HidStickParser hidParser;
ReportEndpoint reportEndpoints[MAX_REPORT_ENDPOINTS];
size_t reportEndpointCount = 0;
BLEClient *bleClient = nullptr;
BLEAdvertisedDevice *candidateDevice = nullptr;
volatile bool shouldConnect = false;
volatile bool bleConnected = false;
unsigned long lastRawPrintMs = 0;

void stopGamepad()
{
    appCTRL.Velocity = 0.0f;
    appCTRL.SteerVelocity = 0.0f;
    appCTRL.Connected = false;
    appCTRL.LastReportMs = 0;
}

float applyDeadzone(float value)
{
    const float magnitude = fabsf(value);
    if (magnitude <= GAMEPAD_DEADZONE)
    {
        return 0.0f;
    }
    const float scaled = (magnitude - GAMEPAD_DEADZONE) / (1.0f - GAMEPAD_DEADZONE);
    return copysignf(constrain(scaled, 0.0f, 1.0f), value);
}

const ReportEndpoint *findReportEndpoint(BLERemoteCharacteristic *characteristic)
{
    for (size_t index = 0; index < reportEndpointCount; ++index)
    {
        if (reportEndpoints[index].characteristic == characteristic)
        {
            return &reportEndpoints[index];
        }
    }
    return nullptr;
}

void printRawReport(uint8_t reportId, const uint8_t *data, size_t length, bool decoded, float x, float y)
{
    const unsigned long now = millis();
    if (now - lastRawPrintMs < GAMEPAD_DEBUG_INTERVAL_MS)
    {
        return;
    }
    lastRawPrintMs = now;

    Serial.print(F("HID id="));
    Serial.print(reportId);
    Serial.print(F(" len="));
    Serial.print(length);
    Serial.print(F(" raw="));
    const size_t printLength = std::min<size_t>(length, 24);
    for (size_t index = 0; index < printLength; ++index)
    {
        if (data[index] < 0x10)
        {
            Serial.print('0');
        }
        Serial.print(data[index], HEX);
        if (index + 1 < printLength)
        {
            Serial.print(' ');
        }
    }

    if (decoded)
    {
        Serial.print(F(" stickX="));
        Serial.print(x, 2);
        Serial.print(F(" stickY="));
        Serial.print(y, 2);
        Serial.print(F(" cmdV="));
        Serial.print(appCTRL.Velocity, 2);
        Serial.print(F(" steer="));
        Serial.print(appCTRL.SteerVelocity, 2);
    }
    else
    {
        Serial.print(F(" (unmapped)"));
    }
    Serial.println();
}

void gamepadNotifyCallback(BLERemoteCharacteristic *characteristic, uint8_t *data, size_t length, bool isNotify)
{
    (void)isNotify;
    const ReportEndpoint *endpoint = findReportEndpoint(characteristic);
    uint8_t reportId = (endpoint != nullptr) ? endpoint->reportId : 0;
    const uint8_t *payload = data;
    size_t payloadLength = length;

    if (endpoint != nullptr && !endpoint->reportIdKnown && reportId != 0 &&
        length > 1 && data[0] == reportId)
    {
        payload = data + 1;
        payloadLength = length - 1;
    }

    float x = 0.0f;
    float y = 0.0f;
    const bool decoded = endpoint != nullptr && endpoint->allowDecode &&
                         hidParser.decode(reportId, payload, payloadLength, x, y);
    if (decoded)
    {
        x = applyDeadzone(x);
        y = applyDeadzone(y);
        appCTRL.Velocity = GAMEPAD_FORWARD_SIGN * y * GAMEPAD_FORWARD_LIMIT;
        appCTRL.SteerVelocity = GAMEPAD_STEER_SIGN * x * GAMEPAD_STEER_LIMIT;
        appCTRL.LastReportMs = millis();
        appCTRL.Connected = true;
    }

    printRawReport(reportId, data, length, decoded, x, y);
}

class GamepadClientCallbacks : public BLEClientCallbacks
{
    void onConnect(BLEClient *client) override
    {
        (void)client;
        Serial.println(F("BLE link connected; discovering HID reports..."));
    }

    void onDisconnect(BLEClient *client) override
    {
        (void)client;
        bleConnected = false;
        reportEndpointCount = 0;
        stopGamepad();
        Serial.println(F("Gamepad disconnected. Motor commands cleared."));
    }
};

class GamepadSecurityCallbacks : public BLESecurityCallbacks
{
    uint32_t onPassKeyRequest() override
    {
        return 0;
    }

    void onPassKeyNotify(uint32_t passKey) override
    {
        Serial.print(F("BLE passkey: "));
        Serial.println(passKey);
    }

    bool onSecurityRequest() override
    {
        return true;
    }

    void onAuthenticationComplete(esp_ble_auth_cmpl_t result) override
    {
        Serial.print(F("BLE pairing "));
        Serial.println(result.success ? F("completed") : F("failed"));
    }

    bool onConfirmPIN(uint32_t pin) override
    {
        Serial.print(F("BLE confirm PIN: "));
        Serial.println(pin);
        return true;
    }
};

class GamepadAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice device) override
    {
        bool isHid = false;
        for (int index = 0; index < device.getServiceUUIDCount(); ++index)
        {
            if (device.getServiceUUID(index).equals(BLEUUID(HID_SERVICE_UUID)))
            {
                isHid = true;
                break;
            }
        }

        std::string name = device.haveName() ? device.getName() : "";
        std::string lowerName = name;
        std::transform(lowerName.begin(), lowerName.end(), lowerName.begin(),
                       [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
        const bool nameMatches = lowerName.find("flydigi") != std::string::npos ||
                                 lowerName.find("direwolf") != std::string::npos ||
                                 lowerName.find("xbox wireless") != std::string::npos;
        const bool gamepadAppearance = device.haveAppearance() && device.getAppearance() == HID_GAMEPAD_APPEARANCE;

        if (!isHid && !nameMatches && !gamepadAppearance)
        {
            return;
        }

        Serial.print(F("Gamepad candidate: "));
        Serial.print(name.empty() ? "(no name)" : name.c_str());
        Serial.print(F(" address="));
        Serial.print(device.getAddress().toString().c_str());
        Serial.print(F(" addressType="));
        Serial.print(static_cast<int>(device.getAddressType()));
        Serial.print(F(" rssi="));
        Serial.print(device.getRSSI());
        Serial.print(F(" hid="));
        Serial.print(isHid ? F("YES") : F("NO"));
        Serial.print(F(" appearance="));
        if (device.haveAppearance())
        {
            Serial.println(device.getAppearance(), HEX);
        }
        else
        {
            Serial.println(F("none"));
        }

        BLEDevice::getScan()->stop();
        delete candidateDevice;
        candidateDevice = new BLEAdvertisedDevice(device);
        shouldConnect = true;
    }
};

GamepadClientCallbacks clientCallbacks;
GamepadSecurityCallbacks securityCallbacks;
GamepadAdvertisedDeviceCallbacks advertisedDeviceCallbacks;
BLESecurity bleSecurity;

bool connectGamepad()
{
    if (candidateDevice == nullptr)
    {
        return false;
    }

    if (bleClient == nullptr)
    {
        bleClient = BLEDevice::createClient();
        bleClient->setClientCallbacks(&clientCallbacks);
    }

    Serial.print(F("Connecting to "));
    Serial.println(candidateDevice->getAddress().toString().c_str());
    Serial.println(F("Waiting for the BLE link. Keep the controller in fast-blinking pairing mode."));
    if (!bleClient->connect(candidateDevice))
    {
        Serial.println(F("BLE connection failed."));
        return false;
    }

    bleClient->setMTU(128);
    BLERemoteService *hidService = bleClient->getService(BLEUUID(HID_SERVICE_UUID));
    if (hidService == nullptr)
    {
        Serial.println(F("Connected device has no BLE HID service (0x1812)."));
        bleClient->disconnect();
        return false;
    }

    bool mapParsed = false;
    BLERemoteCharacteristic *reportMap = hidService->getCharacteristic(BLEUUID(HID_REPORT_MAP_UUID));
    if (reportMap != nullptr && reportMap->canRead())
    {
        reportMap->setAuth(ESP_GATT_AUTH_REQ_NO_MITM);
        const std::string mapData = reportMap->readValue();
        if (!mapData.empty())
        {
            mapParsed = hidParser.parse(reinterpret_cast<const uint8_t *>(mapData.data()), mapData.length());
            hidParser.printSummary();
        }
    }

    if (!mapParsed)
    {
        Serial.println(F("Standard stick mapping is not ready; reports will be printed but movement stays at zero."));
    }

    reportEndpointCount = 0;
    std::map<uint16_t, BLERemoteCharacteristic *> *characteristics = hidService->getCharacteristicsByHandle();
    for (const auto &entry : *characteristics)
    {
        BLERemoteCharacteristic *characteristic = entry.second;
        if (!characteristic->getUUID().equals(BLEUUID(HID_REPORT_UUID)) ||
            (!characteristic->canNotify() && !characteristic->canIndicate()) ||
            reportEndpointCount >= MAX_REPORT_ENDPOINTS)
        {
            continue;
        }

        uint8_t reportId = 0;
        uint8_t reportType = HID_REPORT_TYPE_INPUT;
        bool reportIdKnown = false;
        BLERemoteDescriptor *reference = characteristic->getDescriptor(BLEUUID(HID_REPORT_REFERENCE_UUID));
        if (reference != nullptr)
        {
            reference->setAuth(ESP_GATT_AUTH_REQ_NO_MITM);
            const std::string referenceData = reference->readValue();
            if (referenceData.length() >= 2)
            {
                reportId = static_cast<uint8_t>(referenceData[0]);
                reportType = static_cast<uint8_t>(referenceData[1]);
                reportIdKnown = true;
            }
        }

        if (reportType != HID_REPORT_TYPE_INPUT)
        {
            continue;
        }

        reportEndpoints[reportEndpointCount].characteristic = characteristic;
        reportEndpoints[reportEndpointCount].reportId = reportId;
        reportEndpoints[reportEndpointCount].reportIdKnown = reportIdKnown;
        reportEndpoints[reportEndpointCount].allowDecode = false;
        ++reportEndpointCount;
    }

    if (reportEndpointCount == 0)
    {
        Serial.println(F("No notifiable HID input report was found."));
        bleClient->disconnect();
        return false;
    }

    for (size_t index = 0; index < reportEndpointCount; ++index)
    {
        ReportEndpoint &endpoint = reportEndpoints[index];
        if (endpoint.reportIdKnown)
        {
            endpoint.allowDecode = hidParser.isValid() && endpoint.reportId == hidParser.axisReportId();
        }
        else if (reportEndpointCount == 1 && hidParser.isValid())
        {
            endpoint.reportId = hidParser.axisReportId();
            endpoint.allowDecode = true;
        }
        else if (hidParser.isValid() && hidParser.axisReportId() == 0)
        {
            endpoint.allowDecode = true;
        }

        endpoint.characteristic->setAuth(ESP_GATT_AUTH_REQ_NO_MITM);
        endpoint.characteristic->registerForNotify(gamepadNotifyCallback,
                                                   endpoint.characteristic->canNotify());

        Serial.print(F("Subscribed HID input report id="));
        Serial.print(endpoint.reportId);
        Serial.println(endpoint.allowDecode ? F(" (stick)") : F(" (diagnostic only)"));
    }

    bleConnected = true;
    appCTRL.Connected = true;
    appCTRL.LastReportMs = millis();
    Serial.println(F("Gamepad ready. Physical START still controls motor arming."));
    return true;
}

void gamepadTask(void *parameters)
{
    (void)parameters;
    stopGamepad();

    BLEDevice::init("BalanceCar");
    BLEDevice::setSecurityCallbacks(&securityCallbacks);
    BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
    // Some gamepads use legacy BLE pairing. Requiring Secure Connections only
    // can reject them before HID discovery, so allow normal bonding here.
    bleSecurity.setAuthenticationMode(ESP_LE_AUTH_BOND);
    bleSecurity.setCapability(ESP_IO_CAP_NONE);
    bleSecurity.setKeySize(16);
    bleSecurity.setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
    bleSecurity.setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

    BLEScan *scan = BLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(&advertisedDeviceCallbacks);
    scan->setActiveScan(true);
    scan->setInterval(100);
    scan->setWindow(80);

    Serial.println(F("BLE gamepad task started. Put Flydigi Direwolf 3 into BLE pairing mode."));
    Serial.println(F("ESP32-S3 cannot use the controller's 2.4G dongle or Bluetooth Classic mode."));

    for (;;)
    {
        if (shouldConnect)
        {
            shouldConnect = false;
            connectGamepad();
            delete candidateDevice;
            candidateDevice = nullptr;
        }

        if (bleConnected)
        {
            const unsigned long lastReport = appCTRL.LastReportMs;
            if (lastReport != 0 && millis() - lastReport > GAMEPAD_REPORT_TIMEOUT_MS)
            {
                appCTRL.Velocity = 0.0f;
                appCTRL.SteerVelocity = 0.0f;
            }
        }
        else if (!shouldConnect)
        {
            Serial.println(F("Scanning for a BLE HID gamepad..."));
            scan->clearResults();
            scan->start(5, false);
            if (!shouldConnect)
            {
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
} // namespace

AppControl_t appCTRL;

void AppTaskInit::startTask()
{
    static bool started = false;
    if (started)
    {
        return;
    }
    started = true;
    xTaskCreatePinnedToCore(gamepadTask, "BLE_Gamepad", 10240, nullptr, 1, nullptr, 0);
}

AppTaskInit APP;
