#include "protocol.hpp"
#include <string.h>

namespace Protocol {

Group scan(Cmd cmd)
{
  if (cmd == Cmd::CMD_GLOBAL_OPERATION ||
      cmd == Cmd::CMD_GLOBAL_IDLE ||
      cmd == Cmd::CMD_GLOBAL_PING_MODE ||
      cmd == Cmd::CMD_GLOBAL_EMRGENCY_STOP) {
    return Group::SYSTEM;
  }

  if (cmd == Cmd::CMD_OP_PWM_FWD ||
      cmd == Cmd::CMD_OP_PWM_BWD ||
      cmd == Cmd::CMD_OP_SPD_FWD ||
      cmd == Cmd::CMD_OP_SPD_BWD ||
      cmd == Cmd::CMD_OP_BRAKE   ||
      cmd == Cmd::CMD_OP_LEGACY_CTRL_5B) {
    return Group::OPERATION;
  }

  switch (cmd) {
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

Payload payloadOf(Cmd cmd)
{
  switch (cmd) {
    case Cmd::CMD_OP_PWM_FWD:
    case Cmd::CMD_OP_PWM_BWD:
    case Cmd::CMD_IDLE_ACTUATOR_SERVO_WRITE:
    case Cmd::CMD_IDLE_ACTUATOR_MOTOR_PWM_FWD:
    case Cmd::CMD_IDLE_ACTUATOR_MOTOR_PWM_BWD:
    case Cmd::CMD_IDLE_ACTUATOR_MOTOR_SPD_FWD:
    case Cmd::CMD_IDLE_ACTUATOR_MOTOR_SPD_BWD:
      return Payload::U16_LE;

    case Cmd::CMD_OP_SPD_FWD:
    case Cmd::CMD_OP_SPD_BWD:
        return Payload::F32_LE;

    case Cmd::CMD_IDLE_SET_LINE_PARAMS:
    case Cmd::CMD_IDLE_SET_MPU_PARAMS:
    case Cmd::CMD_IDLE_SET_ULTRA_PARAMS:
    case Cmd::CMD_IDLE_SET_MOTOR_PARAMS:
    case Cmd::CMD_IDLE_SET_SERVO_PARAMS:
    case Cmd::CMD_IDLE_SET_PID_PARAMS:
      return Payload::BYTES30;

    case Cmd::CMD_OP_LEGACY_CTRL_5B:
      return Payload::OP5B;

    default:
      return Payload::NONE;
  }
}

uint8_t payloadLen(Payload p)
{
    switch (p) {
      case Payload::NONE:    return 0;
      case Payload::U16_LE:  return 2;
      case Payload::BYTES30: return 30;
      case Payload::OP5B:    return 4;
      case Payload::F32_LE:  return 4;   // float32
      default:               return 0;
    }
}


bool tryRead(Network& net, RxFrame& out)
{
  uint8_t cmd_u8 = 0;
  if (!net.getUint8(cmd_u8)) {
    return false;
  }

  Cmd     cmd = (Cmd)cmd_u8;
  Payload pl  = payloadOf(cmd);

  out.cmd     = cmd;
  out.group   = scan(cmd);
  out.payload = pl;
  out.u16     = 0;
  out.speed_u16 = 0;
  out.angle_u16 = 0;
  memset(out.bytes30, 0, sizeof(out.bytes30));

  switch (pl) {
    case Payload::NONE:
      break;

    case Payload::U16_LE: {
      uint8_t buf[2];
      if (!net.getArrayUint8(buf, 2)) return false;
      out.u16 = get_u16_le(buf);
      break;
    }

    case Payload::BYTES30: {
      if (!net.getArrayUint8(out.bytes30, 30)) return false;
      break;
    }

    case Payload::OP5B: {
      uint8_t buf[4];
      if (!net.getArrayUint8(buf, 4)) return false;
      out.speed_u16 = get_u16_le(&buf[0]);
      out.angle_u16 = get_u16_le(&buf[2]);
      break;
    }

    case Payload::F32_LE: {
      uint8_t buf[4];
      if (!net.getArrayUint8(buf, 4)) return false;
      float v;
      memcpy(&v, buf, 4);   // ESP32 little-endian nên copy thẳng
      out.f32 = v;
      break;
    }

    default:
      break;
  }

  return true;
}

} // namespace Protocol
