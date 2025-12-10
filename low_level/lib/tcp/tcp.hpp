#pragma once

#include <Arduino.h>
#include <WiFi.h>

class Network
{
public:
    explicit Network(uint16_t port);

    // connect WiFi and start TCP server
    bool begin();

    // Get one uint8 byte from a connected client (non-blocking style)
    // Returns true if a byte was read and stored into 'buffer'
    bool getUint8(uint8_t &buffer);

    // Get array of uint8 bytes from a connected client (non-blocking style)
    // Returns true if all bytes were read into 'buffer'
    bool getArrayUint8(uint8_t* buffer, size_t num);

    // Send a single byte, return true if sent
    bool transmitUint8(uint8_t byte);

    // Send an array of bytes, return true if all bytes accepted
    bool transmitArrayUint8(const uint8_t* buffer, size_t num);

private:
    WiFiServer server;
    WiFiClient client;   // <<< persistent client >>>
    uint16_t   port;

    // Wi-Fi credentials
    const char* ssid     = "COCO 2261";      // your SSID
    const char* password = "1t6#6Y11";  // your password

    // Ensure we have a connected client; accept if needed
    bool ensureClient();
};
