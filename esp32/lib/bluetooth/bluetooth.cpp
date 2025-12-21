#include "bluetooth.hpp"

Network::Network(uint16_t port)
    : port(port)
{
}

bool Network::begin(const char* deviceName)
{
    Serial.println("Starting Bluetooth SPP...");

    if (!bt.begin(deviceName))
    {
        Serial.println("Bluetooth init FAILED");
        return false;
    }

    Serial.print("Bluetooth started, pair with device named: ");
    Serial.println(deviceName);
    return true;
}

bool Network::begin()
{
    Serial.println("Starting Bluetooth SPP...");

    if (!bt.begin(DEVICE_NAME))
    {
        Serial.println("Bluetooth init FAILED");
        return false;
    }

    Serial.print("Bluetooth started, pair with device named: ");
    Serial.println(DEVICE_NAME);
    return true;
}

bool Network::ensureClient()
{
    // hasClient() true khi remote đã connect
    if (!bt.hasClient())
    {
        return false;
    }
    return true;
}

bool Network::isConnected()
{
    return bt.hasClient();
}

bool Network::getUint8(uint8_t &buffer)
{
    if (!ensureClient()) return false;
    if (!bt.available()) return false;

    int c = bt.read();
    if (c < 0) return false;

    buffer = static_cast<uint8_t>(c);
    return true;
}

bool Network::getArrayUint8(uint8_t* buffer, size_t num)
{
    if (!buffer || num == 0) return false;
    if (!ensureClient()) return false;

    // non-blocking: chỉ đọc khi đủ byte
    if (bt.available() < (int)num) return false;

    int readCount = bt.readBytes(buffer, num);
    return (readCount == (int)num);
}

bool Network::transmitArrayUint8(const uint8_t* buffer, size_t num)
{
    if (!buffer || num == 0) return false;
    if (!ensureClient()) return false;

    size_t written = bt.write(buffer, num);
    return (written == num);
}

bool Network::transmitUint8(uint8_t byte)
{
    if (!ensureClient()) return false;

    size_t written = bt.write(&byte, 1);
    return (written == 1);
}
