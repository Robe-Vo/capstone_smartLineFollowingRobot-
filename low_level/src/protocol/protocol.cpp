#pragma once
#include <stdint.h>

namespace Protocol {

  // ===================== CMD LIST (ESP32) =====================
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
    CMD_IDLE_ACTUATOR_MOTOR_ENABLE      = 0xCF,
    CMD_IDLE_ACTUATOR_MOTOR_DISABLE     = 0xCE,
    CMD_IDLE_ACTUATOR_MOTOR_PWM_FWD     = 0xCD,
    CMD_IDLE_ACTUATOR_MOTOR_PWM_BWD     = 0xCC,
    CMD_IDLE_ACTUATOR_MOTOR_SPD_FWD     = 0xCB,
    CMD_IDLE_ACTUATOR_MOTOR_SPD_BWD     = 0xCA,
    CMD_IDLE_ACTUATOR_MOTOR_STOP        = 0xC9,

    CMD_IDLE_ACTUATOR_SERVO_ENABLE       = 0xC8,
    CMD_IDLE_ACTUATOR_SERVO_DISABLE      = 0xC7,
    CMD_IDLE_ACTUATOR_SERVO_WRITE        = 0xC6, // payload u16
    CMD_IDLE_ACTUATOR_SERVO_READ         = 0xC5,
    CMD_IDLE_ACTUATOR_SERVO_WRITE_CENTER = 0xC4,

    // IDLE set params (payload fixed 30)
    CMD_IDLE_SET_LINE_PARAMS     = 0xBF,
    CMD_IDLE_SET_MPU_PARAMS      = 0xBE,
    CMD_IDLE_SET_ULTRA_PARAMS    = 0xBD,
    CMD_IDLE_SET_MOTOR_PARAMS    = 0xBC,
    CMD_IDLE_SET_SERVO_PARAMS    = 0xBB,
    CMD_IDLE_SET_PID_PARAMS      = 0xBA,
  };

  // ===================== ACK =====================
  static constexpr uint8_t ACK_IDLE = 0x20;

  // ===================== GROUP =====================
  enum class Group : uint8_t { UNKNOWN=0xFF, SYSTEM=0, IDLE=1, OPERATION=2 };

  inline Group groupOf(uint8_t cmd)
  {
    // SYSTEM
    if (cmd == (uint8_t)Cmd::CMD_GLOBAL_OPERATION ||
        cmd == (uint8_t)Cmd::CMD_GLOBAL_IDLE ||
        cmd == (uint8_t)Cmd::CMD_GLOBAL_PING_MODE ||
        cmd == (uint8_t)Cmd::CMD_GLOBAL_EMRGENCY_STOP) {
      return Group::SYSTEM;
    }

    // OPERATION
    if (cmd == (uint8_t)Cmd::CMD_OP_PWM_FWD ||
        cmd == (uint8_t)Cmd::CMD_OP_PWM_BWD ||
        cmd == (uint8_t)Cmd::CMD_OP_SPD_FWD ||
        cmd == (uint8_t)Cmd::CMD_OP_SPD_BWD ||
        cmd == (uint8_t)Cmd::CMD_OP_BRAKE) {
      return Group::OPERATION;
    }

    // Known IDLE (by explicit list)
    switch ((Cmd)cmd) {
      case Cmd::CMD_IDLE_SENSOR_LINE_READ:
      case Cmd::CMD_IDLE_SENSOR_ULTRA_READ:
      case Cmd::CMD_IDLE_SENSOR_ULTRA_KICK:
      case Cmd::CMD_IDLE_SENSOR_MPU_READ:
      case Cmd::CMD_IDLE_ENCODER_ENABLE:
      case Cmd::CMD_IDLE_ENCODER_DISABLE:
      case Cmd::CMD_IDLE_ACTUATOR_MOTOR_ENABLE:
      case Cmd::CMD_IDLE_ACTUATOR_MOTOR_DISABLE:
      case Cmd::CMD_IDLE_ACTUATOR_MOTOR_PWM_FWD:
      case Cmd::CMD_IDLE_ACTUATOR_MOTOR_PWM_BWD:
      case Cmd::CMD_IDLE_ACTUATOR_MOTOR_SPD_FWD:
      case Cmd::CMD_IDLE_ACTUATOR_MOTOR_SPD_BWD:
      case Cmd::CMD_IDLE_ACTUATOR_MOTOR_STOP:
      case Cmd::CMD_IDLE_ACTUATOR_SERVO_ENABLE:
      case Cmd::CMD_IDLE_ACTUATOR_SERVO_DISABLE:
      case Cmd::CMD_IDLE_ACTUATOR_SERVO_WRITE:
      case Cmd::CMD_IDLE_ACTUATOR_SERVO_READ:
      case Cmd::CMD_IDLE_ACTUATOR_SERVO_WRITE_CENTER:
      case Cmd::CMD_IDLE_SET_LINE_PARAMS:
      case Cmd::CMD_IDLE_SET_MPU_PARAMS:
      case Cmd::CMD_IDLE_SET_ULTRA_PARAMS:
      case Cmd::CMD_IDLE_SET_MOTOR_PARAMS:
      case Cmd::CMD_IDLE_SET_SERVO_PARAMS:
      case Cmd::CMD_IDLE_SET_PID_PARAMS:
        return Group::IDLE;
      default:
        return Group::UNKNOWN;
    }
  }

  // ===================== PAYLOAD LENGTH =====================
  // return: 0, 2, or 30
  inline uint8_t payloadLen(uint8_t cmd)
  {
    switch ((Cmd)cmd) {
      // u16 payload (LE)
      case Cmd::CMD_OP_PWM_FWD:
      case Cmd::CMD_OP_PWM_BWD:
      case Cmd::CMD_OP_SPD_FWD:
      case Cmd::CMD_OP_SPD_BWD:
      case Cmd::CMD_IDLE_ACTUATOR_SERVO_WRITE:
      case Cmd::CMD_IDLE_ACTUATOR_MOTOR_PWM_FWD:
      case Cmd::CMD_IDLE_ACTUATOR_MOTOR_PWM_BWD:
      case Cmd::CMD_IDLE_ACTUATOR_MOTOR_SPD_FWD:
      case Cmd::CMD_IDLE_ACTUATOR_MOTOR_SPD_BWD:
        return 2;

      // 30-byte params
      case Cmd::CMD_IDLE_SET_LINE_PARAMS:
      case Cmd::CMD_IDLE_SET_MPU_PARAMS:
      case Cmd::CMD_IDLE_SET_ULTRA_PARAMS:
      case Cmd::CMD_IDLE_SET_MOTOR_PARAMS:
      case Cmd::CMD_IDLE_SET_SERVO_PARAMS:
      case Cmd::CMD_IDLE_SET_PID_PARAMS:
        return 30;

      default:
        return 0;
    }
  }

} // namespace Protocol
