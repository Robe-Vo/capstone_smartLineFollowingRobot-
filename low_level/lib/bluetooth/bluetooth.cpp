// #include "bluetooth.hpp"

// Network::Network(const char* deviceName)
// : name(deviceName)
// {
// }

// bool Network::begin()
// {
//     if (!name) {
//         name = "ESP32_Network";
//     }

//     bool ok = bluetooth.begin(name);
//     if (!ok) {
//         Serial.println(F("[Network] Bluetooth begin FAILED"));
//         return false;
//     }

//     Serial.print(F("[Network] Bluetooth started with name: "));
//     Serial.println(name);
//     return true;
// }

// bool Network::transmit(uint8_t data)
// {
//     return bluetooth.write(data);
// }

// bool Network::transmit(uint8_t* data, size_t len)
// {
//     if (!bluetooth.connected()) {
//         return false;
//     }

//     if (len > sizeof(transmitBuffer)) {
//         len = sizeof(transmitBuffer);
//     }

//     memcpy(transmitBuffer, data, len);
//     size_t written = bluetooth.write(transmitBuffer, len);
//     return (written == len);
// }

// bool Network::readCmd()
// {
//     // Check if connection is working
//     if (!bluetooth.available()) {
//         commandReady = false;
//         return false;
//     }

//     // read ONE command byte
//     cmd = (uint8_t) bluetooth.read();
//     commandReady = true;
//     frameReady   = false;

//     return true;
// }

// bool Network::read(size_t num)
// {
//     if (num > sizeof(receiveBuffer)) {
//         num = sizeof(receiveBuffer);
//     }

//     size_t i = 0;
//     while (i < num) {
//         if (bluetooth.available()) {
//             receiveBuffer[i++] = (uint8_t) bluetooth.read();
//         }
//         // (optional: add timeout here)
//     }

//     frameReady = true;
//     return true;
// }
#include "bluetooth.hpp"

Network::Network(uint16_t port)
    : port(port)
{
}

bool Network::begin()
{
    Serial.println("Starting Bluetooth SPP...");

    // Change this name if you want a different Bluetooth device name
    const char* deviceName = "Behind the scream123";

    if (!bt.begin(deviceName))
    {
        Serial.println("Bluetooth init FAILED");
        return false;
    }

    Serial.print("Bluetooth started, pair with device named: ");
    Serial.println(deviceName);
    return true;
}

// For BluetoothSerial, "client" just means a paired / connected host device
bool Network::ensureClient()
{
    // hasClient() returns true when a remote device is connected
    if (!bt.hasClient())
    {
        // No connected host yet
        return false;
    }
    return true;
}

bool Network::getUint8(uint8_t &buffer)
{
    if (!ensureClient())
        return false;

    if (!bt.available())
        return false;

    int c = bt.read();
    if (c < 0)
        return false;

    buffer = static_cast<uint8_t>(c);
    return true;
}

bool Network::getArrayUint8(uint8_t* buffer, size_t num)
{
    if (!buffer || num == 0)
        return false;

    if (!ensureClient())
        return false;

    if (bt.available() < static_cast<int>(num))
        return false;   // not enough data yet (non-blocking behavior)

    int readCount = bt.readBytes(buffer, num);
    return (readCount == static_cast<int>(num));
}

bool Network::transmitArrayUint8(const uint8_t* buffer, size_t num)
{
    if (!buffer || num == 0)
        return false;

    if (!ensureClient())
        return false;

    size_t written = bt.write(buffer, num);

    if (written != num)
    {
        Serial.println("Bluetooth write failed.");
        return false;
    }

    Serial.print("BT TX ");
    Serial.print(written);
    Serial.print("/");
    Serial.print(num);
    Serial.println(" bytes");

    return true;
}

bool Network::transmitUint8(uint8_t byte)
{
    if (!ensureClient())
        return false;

    size_t written = bt.write(&byte, 1);

    if (written != 1)
    {
        Serial.println("Bluetooth write failed (1 byte).");
        return false;
    }

    Serial.print("BT TX 1 byte: 0x");
    Serial.println(byte, HEX);
    return true;
}
