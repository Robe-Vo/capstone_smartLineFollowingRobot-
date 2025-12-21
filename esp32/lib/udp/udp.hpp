#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

class Network
{
public:
    explicit Network(uint16_t port);

    // Connect WiFi and start UDP listener on 'port'
    bool begin();

    // Non-blocking: returns true if 1 byte was read into 'buffer'
    bool getUint8(uint8_t &buffer);

    // Non-blocking: returns true if exactly 'num' bytes were read as one UDP packet
    bool getArrayUint8(uint8_t* buffer, size_t num);

    // Send 1 byte via UDP to last known remote endpoint
    bool transmitUint8(uint8_t byte);

    // Send 'num' bytes via UDP to last known remote endpoint
    bool transmitArrayUint8(const uint8_t* buffer, size_t num);

private:
    WiFiUDP   udp;
    uint16_t  port;

    // Wi-Fi credentials
    const char* ssid     = "TUAN KIET";   // change to your SSID
    const char* password = "0909223015";   // change to your password

    // const char* ssid = "Galaxy";
    // const char* password = "he124223";

    // Remember last remote who sent us a packet
    IPAddress  remoteIP;
    uint16_t   remotePort = 0;
    bool       haveRemote = false;

    // Internal helpers
    bool ensureRemote() const;
};
