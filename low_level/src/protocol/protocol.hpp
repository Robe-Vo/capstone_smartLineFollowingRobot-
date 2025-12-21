#pragma once
#include <stdint.h>
#include "bluetooth.hpp"

namespace Protocol {

// ======================= GROUPS =======================
enum class Group : uint8_t {
    UNKNOWN   = 0,
    SYSTEM    = 1,
    IDLE      = 2,
    OPERATION = 3,
};

// ======================= COMMANDS =======================
// Giá trị enum có thể điều chỉnh lại cho khớp MATLAB nếu cần.
// Ở đây giữ đúng tên để toàn bộ switch/case trong code build được.
enum class Cmd : uint8_t {
    // ---- SYSTEM ----
    CMD_GLOBAL_OPERATION      = 0x01,
    CMD_GLOBAL_IDLE           = 0x02,
    CMD_GLOBAL_PING_MODE      = 0x03,
    CMD_GLOBAL_EMRGENCY_STOP  = 0x04,

    // ---- OPERATION ----
    CMD_OP_PWM_FWD            = 0x10,
    CMD_OP_PWM_BWD            = 0x11,
    CMD_OP_SPD_FWD            = 0x12,
    CMD_OP_SPD_BWD            = 0x13,
    CMD_OP_BRAKE              = 0x14,
    CMD_OP_LEGACY_CTRL_5B     = 0x15,  // speed_u16 + angle_u16 (4 bytes payload)

    // ---- IDLE: SENSOR ----
    CMD_IDLE_SENSOR_LINE_READ = 0x20,
    CMD_IDLE_SENSOR_ULTRA_READ= 0x21,
    CMD_IDLE_SENSOR_ULTRA_KICK= 0x22,
    CMD_IDLE_SENSOR_MPU_READ  = 0x23,
    CMD_IDLE_ENCODER_ENABLE   = 0x24,
    CMD_IDLE_ENCODER_DISABLE  = 0x25,

    // ---- IDLE: MOTOR ----
    CMD_IDLE_ACTUATOR_MOTOR_ENABLE   = 0x30,
    CMD_IDLE_ACTUATOR_MOTOR_DISABLE  = 0x31,
    CMD_IDLE_ACTUATOR_MOTOR_PWM_FWD  = 0x32,
    CMD_IDLE_ACTUATOR_MOTOR_PWM_BWD  = 0x33,
    CMD_IDLE_ACTUATOR_MOTOR_SPD_FWD  = 0x34,
    CMD_IDLE_ACTUATOR_MOTOR_SPD_BWD  = 0x35,
    CMD_IDLE_ACTUATOR_MOTOR_STOP     = 0x36,

    // ---- IDLE: SERVO ----
    CMD_IDLE_ACTUATOR_SERVO_ENABLE        = 0x40,
    CMD_IDLE_ACTUATOR_SERVO_DISABLE       = 0x41,
    CMD_IDLE_ACTUATOR_SERVO_WRITE         = 0x42,
    CMD_IDLE_ACTUATOR_SERVO_READ          = 0x43,
    CMD_IDLE_ACTUATOR_SERVO_WRITE_CENTER  = 0x44,

    // ---- IDLE: SET PARAMS (30 bytes) ----
    CMD_IDLE_SET_LINE_PARAMS   = 0x50,
    CMD_IDLE_SET_MPU_PARAMS    = 0x51,
    CMD_IDLE_SET_ULTRA_PARAMS  = 0x52,
    CMD_IDLE_SET_MOTOR_PARAMS  = 0x53,
    CMD_IDLE_SET_SERVO_PARAMS  = 0x54,
    CMD_IDLE_SET_PID_PARAMS    = 0x55,
};

// ======================= PAYLOAD TYPE =======================
enum class Payload : uint8_t {
    NONE    = 0,  // chỉ cmd
    U16_LE  = 1,  // 2 byte
    BYTES30 = 2,  // 30 byte params
    OP5B    = 3,  // legacy 4 byte (speed_u16+angle_u16)
    F32_LE  = 4,  // 4 byte float little-endian
};


// ======================= HELPERS =======================
inline uint16_t get_u16_le(const uint8_t* p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

inline void put_u16_le(uint8_t* p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

// Phân nhóm cmd → group
Group   scan(Cmd cmd);

// Xác định kiểu payload
Payload payloadOf(Cmd cmd);

// Độ dài payload theo kiểu
uint8_t payloadLen(Payload p);

// ======================= RX FRAME =======================
struct RxFrame {
    Cmd     cmd;
    Group   group;
    Payload payload;

    uint16_t u16;          // U16_LE
    uint8_t  bytes30[30];  // BYTES30

    uint16_t speed_u16;    // OP5B legacy
    uint16_t angle_u16;    // OP5B legacy

    float    f32;          // F32_LE (dùng cho speed float)
};

// Non-blocking read 1 frame từ Network
bool tryRead(Network& net, RxFrame& out);

// ACK byte cho SYSTEM/IDLE
constexpr uint8_t ACK_IDLE = 0x20;

} // namespace Protocol
