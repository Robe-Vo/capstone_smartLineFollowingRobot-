// ======================= protocol.hpp (IF/ELSE ONLY, CONSISTENT LITTLE-ENDIAN) =======================
// Flow:
// 1) Check mode
// 2) Check cmd group (sys / op / idle)
// 3) Gather buffer (cmd-dependent length, non-blocking)
// 4) Return parsed payload + flag(cmd)
//
// Wire endian (constant):
// - uint16: little-endian
// - float32: IEEE754 little-endian

#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "bluetooth.hpp"

namespace Protocol {

enum class Mode : uint8_t { IDLE = 0x00, OPERATION = 0x01 };

enum class Cmd : uint8_t {
    // Global (system)
    CMD_GLOBAL_OPERATION       = 0xFF,
    CMD_GLOBAL_IDLE            = 0xFE,
    CMD_GLOBAL_PING_MODE       = 0xFD,
    CMD_GLOBAL_EMRGENCY_STOP   = 0xF0,

    // OPERATION
    CMD_OP_PWM_FWD = 0xEF,
    CMD_OP_PWM_BWD = 0xEE,
    CMD_OP_SPD_FWD = 0xED,
    CMD_OP_SPD_BWD = 0xEC,
    CMD_OP_BRAKE   = 0xEB,

    // IDLE sensors
    CMD_IDLE_SENSOR_LINE_READ  = 0xDF,
    CMD_IDLE_SENSOR_ULTRA_READ = 0xDE,
    CMD_IDLE_SENSOR_ULTRA_KICK = 0xDD,
    CMD_IDLE_SENSOR_MPU_READ   = 0xDC,
    CMD_IDLE_ENCODER_ENABLE    = 0xDB,
    CMD_IDLE_ENCODER_DISABLE   = 0xDA,

    // IDLE actuators
    CMD_IDLE_ACTUATOR_MOTOR_ENABLE           = 0xCF,
    CMD_IDLE_ACTUATOR_MOTOR_DISABLE          = 0xCE,
    CMD_IDLE_ACTUATOR_MOTOR_PWM_FWD          = 0xCD, // optional payload (disabled by default)
    CMD_IDLE_ACTUATOR_MOTOR_PWM_BWD          = 0xCC, // optional payload (disabled by default)
    CMD_IDLE_ACTUATOR_MOTOR_SPD_FWD          = 0xCB, // optional payload (disabled by default)
    CMD_IDLE_ACTUATOR_MOTOR_SPD_BWD          = 0xCA, // optional payload (disabled by default)
    CMD_IDLE_ACTUATOR_MOTOR_STOP             = 0xC9,

    CMD_IDLE_ACTUATOR_SERVO_ENABLE       = 0xC8,
    CMD_IDLE_ACTUATOR_SERVO_DISABLE      = 0xC7,
    CMD_IDLE_ACTUATOR_SERVO_WRITE        = 0xC6, // payload: angle_u16
    CMD_IDLE_ACTUATOR_SERVO_READ         = 0xC5,
    CMD_IDLE_ACTUATOR_SERVO_WRITE_CENTER = 0xC4,

    // IDLE: set params (payload length fixed 30)
    CMD_IDLE_SET_LINE_PARAMS     = 0xBF,
    CMD_IDLE_SET_MPU_PARAMS      = 0xBE,
    CMD_IDLE_SET_ULTRA_PARAMS    = 0xBD,
    CMD_IDLE_SET_MOTOR_PARAMS    = 0xBC,
    CMD_IDLE_SET_SERVO_PARAMS    = 0xBB,
    CMD_IDLE_SET_PID_PARAMS      = 0xBA
};

// ======================= Frames =======================

struct IdleCmd {
    uint8_t  cmd = 0;
    uint8_t  len = 0;
    uint8_t  data[30] = {0};

    // LINE params
    uint16_t line_sample_ms = 1;
    uint16_t line_threshold = 0;
    uint16_t line_reserved0 = 0;

    // ULTRA params
    uint16_t ultra_kick_period_ms = 60;
    uint16_t ultra_timeout_ms     = 25;
    uint16_t ultra_reserved0      = 0;

    // MPU params
    uint16_t mpu_sample_rate_hz = 50;
    float    mpu_bias_ax = 0.0f;
    float    mpu_bias_ay = 0.0f;
    float    mpu_bias_az = 0.0f;

    // MOTOR params
    uint16_t motor_pwm_freq_hz = 20000;
    uint16_t motor_pwm_max     = 2047;
    float    motor_ff          = 0.0f;

    // SERVO params
    uint16_t servo_center = 75;
    uint16_t servo_min    = 55;
    uint16_t servo_max    = 105;

    // PID params
    float motor_kp = 0.0f;
    float motor_ki = 0.0f;
    float motor_kd = 0.0f;
    float motor_windup = 0.0f;
};

struct OperationCmd {
    uint8_t  cmd   = 0;
    uint16_t pwm   = 0;      // valid if PWM cmd
    float    speed = 0.0f;   // valid if SPEED cmd
    uint16_t angle = 0;      // valid if payloadLen==6
};

struct RxResult {
    bool ok = false;
    Mode mode = Mode::IDLE;
    uint8_t flag_cmd = 0;
    IdleCmd idle{};
    OperationCmd op{};
};

// ======================= Endian helpers =======================

inline uint16_t u16_le(const uint8_t* p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

inline float f32_le(const uint8_t* p) {
    float v;
    uint8_t tmp[4] = { p[0], p[1], p[2], p[3] };
    memcpy(&v, tmp, 4);
    return v;
}

// ======================= Classify helpers (IF/ELSE ONLY) =======================

inline bool isSystemCmd(uint8_t c) {
    return c == (uint8_t)Cmd::CMD_GLOBAL_OPERATION ||
           c == (uint8_t)Cmd::CMD_GLOBAL_IDLE ||
           c == (uint8_t)Cmd::CMD_GLOBAL_PING_MODE ||
           c == (uint8_t)Cmd::CMD_GLOBAL_EMRGENCY_STOP;
}

bool isIdleCmd(uint8_t c, uint8_t& payloadLen);       // if/else in .cpp
bool isOperationCmd(uint8_t c, uint8_t& payloadLen);  // if/else in .cpp

// ======================= API =======================
RxResult poll(Network& net, Mode currentMode);

} // namespace Protocol
