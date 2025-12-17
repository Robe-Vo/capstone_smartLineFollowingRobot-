#pragma once

#include <Arduino.h>
#include "BluetoothSerial.h"

class Network
{
public:
    explicit Network(uint16_t port = 0);

    bool begin();

    // Generic primitives (giữ lại để tương thích)
    bool getUint8(uint8_t &buffer);
    bool getArrayUint8(uint8_t* buffer, size_t num);
    bool transmitArrayUint8(const uint8_t* buffer, size_t num);
    bool transmitUint8(uint8_t byte);

    // ================= OPERATION MODE PACKAGING =================
    // Frame control: 5 bytes = cmd(1) | speed_u16(2) | angle_u16(2)
    // Endian mặc định: LITTLE (speed = b1 | b2<<8)
    struct OpControlFrame {
        uint8_t  cmd;
        uint16_t speed;
        uint16_t angle;
    };

    // Telemetry frame (bạn đang dùng 22 bytes trong main)
    struct OpTelemetry22 {
        uint8_t data[22];
    };

    // Try read exactly 5 bytes and decode into OpControlFrame (non-blocking)
    bool opTryReadControlFrame(OpControlFrame &out);

    // Encode and send telemetry (22 bytes) (non-blocking write)
    bool opSendTelemetry22(const uint8_t* frame22);

    // Backward-compatible wrapper (giữ tên hàm cũ)
    bool getFrame(uint8_t &cmd, uint16_t &speed, uint16_t &angle);

    // Optional: check connection (expose for main)
    bool isConnected();

private:
    bool ensureClient();

    static inline uint16_t u16_le(const uint8_t* p) {
        return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
    }

    uint16_t port;              // unused, kept for API compatibility
    BluetoothSerial bt;
};
