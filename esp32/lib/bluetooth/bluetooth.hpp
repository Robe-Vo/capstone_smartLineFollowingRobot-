#pragma once

#include <Arduino.h>
#include "BluetoothSerial.h"
#include "../../include/cfg.hpp"

/**
 *  === Bluetooth network interface ===
 * 
 * This class provide a network interface over Bluetooth Serial Port Profile (SPP).
 * It allows to send and receive raw over a Bluetooth connection using simple methods. 
 */

class Network
{
public:
    // Constructor
    explicit Network(uint16_t port = 0);

    // Begin bluetooth network
    bool begin(const char* deviceName);
    bool begin();

    // Get a byte from bluetooth
    bool getUint8(uint8_t &buffer);

    // Get an array of bytes from bluetooth
    bool getArrayUint8(uint8_t* buffer, size_t num);

    // Transmit a byte to bluetooth
    bool transmitUint8(uint8_t byte);

    // Transmit an array of bytes to bluetooth
    bool transmitArrayUint8(const uint8_t* buffer, size_t num);
    
    // Check if bluetooth is connected
    bool isConnected();


private:
    // Ensure that a client is connected
    bool ensureClient();

    uint16_t port;              // unused, kept for API compatibility
    BluetoothSerial bt;
};
