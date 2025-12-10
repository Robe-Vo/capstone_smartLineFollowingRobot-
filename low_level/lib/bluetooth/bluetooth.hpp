// #pragma once

// #include <Arduino.h>
// #include <BluetoothSerial.h>
// #include <stdint.h>

// class Network
// {
//     private:
//         BluetoothSerial bluetooth;

//         const char* name;

//         uint8_t cmd = 0x00;
//         uint8_t receiveBuffer[10];
//         uint8_t transmitBuffer[10];
//         uint8_t receiveLength = 0;

//         bool commandReady = false;
//         bool frameReady = false;
//     public:
//         Network(char const* name);

//         bool begin();
//         bool transmit(uint8_t data);
//         bool transmit(uint8_t* data, size_t len);
//         bool readCmd();
//         bool read(size_t);
//         bool read_cmd_isDone() {return commandReady;}
//         bool read_frame_isDone() {return frameReady;}

//         uint8_t getCmd () {return cmd;}
//         uint8_t* getReceiveBuffer()       { return receiveBuffer; }
//         size_t   getReceiveLength() const { return receiveLength; }
//         void clear()
//         {
//             cmd = 0x00;
//             memset(receiveBuffer, 0, sizeof(receiveBuffer));
//             receiveLength = 0;
//             commandReady = false;
//             frameReady = false;
//         }


// };


#pragma once

#include <Arduino.h>
#include "BluetoothSerial.h"

class Network
{
public:
    // 'port' is kept only for compatibility with your old code.
    explicit Network(uint16_t port = 0);

    // Start Bluetooth SPP server
    bool begin();

    bool getUint8(uint8_t &buffer);
    bool getArrayUint8(uint8_t* buffer, size_t num);
    bool transmitArrayUint8(const uint8_t* buffer, size_t num);
    bool transmitUint8(uint8_t byte);

private:
    bool ensureClient();

    uint16_t port;              // unused, kept for API compatibility
    BluetoothSerial bt;
};
