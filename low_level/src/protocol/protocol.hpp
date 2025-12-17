#pragma once
#include <stdint.h>
#include "bluetooth.hpp" // Network

namespace Protocol {

// ======================= CMD (single source of truth) =======================
enum class Cmd : uint8_t {
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
  CMD_IDLE_ACTUATOR_MOTOR_PWM_FWD          = 0xCD, // optional payload u16
  CMD_IDLE_ACTUATOR_MOTOR_PWM_BWD          = 0xCC, // optional payload u16
  CMD_IDLE_ACTUATOR_MOTOR_SPD_FWD          = 0xCB, // optional payload u16
  CMD_IDLE_ACTUATOR_MOTOR_SPD_BWD          = 0xCA, // optional payload u16
  CMD_IDLE_ACTUATOR_MOTOR_STOP             = 0xC9,

  CMD_IDLE_ACTUATOR_SERVO_ENABLE       = 0xC8,
  CMD_IDLE_ACTUATOR_SERVO_DISABLE      = 0xC7,
  CMD_IDLE_ACTUATOR_SERVO_WRITE        = 0xC6, // payload u16 angle
  CMD_IDLE_ACTUATOR_SERVO_READ         = 0xC5,
  CMD_IDLE_ACTUATOR_SERVO_WRITE_CENTER = 0xC4,

  // IDLE: set params (payload length fixed 30)
  CMD_IDLE_SET_LINE_PARAMS     = 0xBF,
  CMD_IDLE_SET_MPU_PARAMS      = 0xBE,
  CMD_IDLE_SET_ULTRA_PARAMS    = 0xBD,
  CMD_IDLE_SET_MOTOR_PARAMS    = 0xBC,
  CMD_IDLE_SET_SERVO_PARAMS    = 0xBB,
  CMD_IDLE_SET_PID_PARAMS      = 0xBA,

  // Optional legacy OP 5-byte control frame marker if you still use it:
  // cmd(1) + speed_u16(2 LE) + angle_u16(2 LE)
  CMD_OP_LEGACY_CTRL_5B        = 0xF1
};

static constexpr uint8_t ACK_IDLE = 0x20;

// ======================= GROUP =======================
enum class Group : uint8_t { UNKNOWN=0, SYSTEM=1, IDLE=2, OPERATION=3 };

Group scan(Cmd cmd);

// ======================= PAYLOAD TYPE =======================
enum class Payload : uint8_t { NONE=0, U16_LE=1, BYTES30=2, OP5B=3 };

Payload payloadOf(Cmd cmd);
uint8_t payloadLen(Payload p);

// ======================= RX FRAME (decoded) =======================
struct RxFrame {
  Cmd   cmd{Cmd::CMD_GLOBAL_PING_MODE};
  Group group{Group::UNKNOWN};
  Payload payload{Payload::NONE};

  // Payload storage (single place)
  uint16_t u16{0};        // for U16_LE payload
  uint8_t  bytes30[30]{}; // for BYTES30 payload

  // for legacy OP5B
  uint16_t speed_u16{0};
  uint16_t angle_u16{0};
};

// ======================= HELPERS =======================
inline uint16_t get_u16_le(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
inline void put_u16_le(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
}

// ======================= IO API =======================
// Non-blocking: returns true only if full cmd+payload available and read.
bool tryRead(Network& net, RxFrame& out);

// Optional: helper for IDLE policy (ack 0x20 for each IDLE cmd)
inline void sendIdleAckIfNeeded(Network& net, const RxFrame& f) {
  if (f.group == Group::IDLE) net.transmitUint8(ACK_IDLE);
}

} // namespace Protocol
