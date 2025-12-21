// ======================= protocol.hpp (descriptor-table version) =======================
#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <string.h>
#include "bluetooth.hpp"

namespace Protocol
{
  // CMD group
  enum class Group : uint8_t {
    UNKNOWN   = 0,
    SYSTEM    = 1,
    IDLE      = 2,
    OPERATION = 3,
  };

  /**
   * ============= COMMAND DECLARATION =============
   *
   * 0xF_               = SYSTEM,
   * 0xE_               = OPERATION,
   * 0xD_, 0xC_, 0xB_   = IDLE
   *
   * speed: 2 bytes (choose encoding)
   * pwm  : unsigned 11-bit (carried in uint16) 2 bytes
   * angle: int16_t 2 bytes
   */
  enum class Cmd: uint8_t
  {
    // ---- SYSTEM ----
    CMD_GLOBAL_OPERATION      = 0xFF,
    CMD_GLOBAL_IDLE           = 0xFE,
    CMD_GLOBAL_PING_MODE      = 0xFD,
    CMD_GLOBAL_EMRGENCY_STOP  = 0xFC,

    // ---- OPERATION ---- (payload: speed/pwm + angle)
    CMD_OP_SPD_FWD            = 0xEF,   // 2B speed + 2B angle
    CMD_OP_SPD_BWD            = 0xEE,   // 2B speed + 2B angle
    CMD_OP_PWM_FWD            = 0xED,   // 2B pwm   + 2B angle
    CMD_OP_PWM_BWD            = 0xEC,   // 2B pwm   + 2B angle
    CMD_OP_BRAKE              = 0xEB,   // no payload

    // ---- IDLE: MOTOR ----
    CMD_IDLE_ACTUATOR_MOTOR_ENABLE   = 0xDF,   // no payload
    CMD_IDLE_ACTUATOR_MOTOR_DISABLE  = 0xDE,   // no payload
    CMD_IDLE_ACTUATOR_MOTOR_SPD_FWD  = 0xDD,   // 2B speed
    CMD_IDLE_ACTUATOR_MOTOR_SPD_BWD  = 0xDC,   // 2B speed
    CMD_IDLE_ACTUATOR_MOTOR_PWM_FWD  = 0xDB,   // 2B pwm
    CMD_IDLE_ACTUATOR_MOTOR_PWM_BWD  = 0xDA,   // 2B pwm
    CMD_IDLE_ACTUATOR_MOTOR_BRAKE    = 0xD9,   // no payload

    // ---- IDLE: SERVO ----
    CMD_IDLE_ACTUATOR_SERVO_ENABLE        = 0xD8,   // no payload
    CMD_IDLE_ACTUATOR_SERVO_DISABLE       = 0xD7,   // no payload
    CMD_IDLE_ACTUATOR_SERVO_WRITE         = 0xD6,   // 2B angle
    CMD_IDLE_ACTUATOR_SERVO_WRITE_CENTER  = 0xD5,   // no payload
    CMD_IDLE_ACTUATOR_SERVO_READ          = 0xD4,   // no payload (ESP will respond 2B angle)

    // ---- IDLE: SENSOR ----
    CMD_IDLE_SENSOR_LINE_READ  = 0xCF,             // no payload (ESP will respond 5B)
    CMD_IDLE_SENSOR_ULTRA_READ = 0xCE,             // no payload (ESP will respond 2B)
    CMD_IDLE_SENSOR_MPU_READ   = 0xCD,             // no payload (ESP will respond 12B if you implement)

    // ---- IDLE: SET PARAMS ----
    // Recommendation: unify to 30 bytes for all SET_* to avoid ambiguity.
    CMD_IDLE_SET_LINE_PARAMS   = 0xBF,             // 30B
    CMD_IDLE_SET_MPU_PARAMS    = 0xBE,             // 30B
    CMD_IDLE_SET_ULTRA_PARAMS  = 0xBD,             // 30B
    CMD_IDLE_SET_MOTOR_PARAMS  = 0xBC,             // 30B
    CMD_IDLE_SET_SERVO_PARAMS  = 0xBB,             // 30B
    CMD_IDLE_SET_PID_PARAMS    = 0xBA,             // 30B
  };

  // ======================= PAYLOAD TYPE =======================
  // For "2 bytes speed": choose one encoding at compile time:
  //   -DPROTOCOL_OP_SPEED_I16X100     : int16 speed_hz_x100 (recommended)
  //   -DPROTOCOL_OP_SPEED_FLOAT16     : IEEE754 half-float
  // If neither defined, speed is float32 (4 bytes) (NOT 2 bytes).
  enum class Payload : uint8_t {
    NONE       = 0,
    U16_LE     = 1,       // 2 bytes
    I16_LE     = 2,       // 2 bytes (angle, speed_i16...)
    BYTES30    = 3,       // 30 bytes
    OP_PWM     = 4,       // 4 bytes: u16 pwm + i16 angle
    OP_SPD     = 5,       // 4 bytes (2B speed + 2B angle) OR 6 bytes if float32
  };

  // ======================= DESCRIPTOR TABLE =======================
  struct CmdDesc {
    Group   group;
    Payload payload;
  };

  // Call once in setup()
  void initDescriptorTable();

  // O(1) lookup
  extern CmdDesc g_cmdTable[256];

  inline Group   groupOf(uint8_t cmd)   { return g_cmdTable[cmd].group; }
  inline Payload payloadOf(uint8_t cmd) { return g_cmdTable[cmd].payload; }

  // ======================= HELPERS =======================
  inline uint16_t get_u16_le(const uint8_t* p){ return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
  inline int16_t  get_i16_le(const uint8_t* p){ return (int16_t)get_u16_le(p); }
  inline void put_u16_le(uint8_t* p, uint16_t v){ p[0]=(uint8_t)(v&0xFF); p[1]=(uint8_t)((v>>8)&0xFF); }
  inline void put_i16_le(uint8_t* p, int16_t v){ put_u16_le(p, (uint16_t)v); }

  float half_to_float(uint16_t h);

  // ======================= RX FRAME =======================
  struct RxFrame {
    Cmd     cmd = (Cmd)0;
    Group   group = Group::UNKNOWN;
    Payload payload = Payload::NONE;

    uint16_t u16 = 0;            // U16_LE
    int16_t  i16 = 0;            // I16_LE
    uint8_t  bytes30[30]{};      // BYTES30

    // OPERATION (PWM+ANGLE)
    uint16_t pwm_u16 = 0;        // 0..2047 expected (11-bit used)
    int16_t  angle_i16 = 0;

    // OPERATION (SPEED+ANGLE) -> stored as float after decode
    float    speed_f = 0.0f;
  };

  // Non-blocking read 1 frame from Network
  bool tryRead(Network& net, RxFrame& out);

  // Payload length for a cmd (from descriptor + speed encoding)
  uint8_t payloadLen(uint8_t cmd_u8);

  // ACK byte for basic commands (if you use it)
  constexpr uint8_t ACK_IDLE = 0x20;


  // OPERATION signals interface
  // | 5 uint8_t | 1 uint16_t | 6 int16_t | 1 int8_t | 1 float (2 bytes) |
  void package_op_signals(uint8_t* buffer,uint8_t* line,uint16_t ultra,int16_t* mpu,int8_t enc,float speed);
  
} // namespace Protocol
