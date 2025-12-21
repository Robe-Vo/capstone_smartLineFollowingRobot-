// ======================= capstone/esp32/src/telemetry/telemetry.cpp (REWRITE) =======================
#include "telemetry.hpp"

namespace Telemetry {

  static IDLE_FRAME          s_idle{};
  static OPERATION_RX_FRAME  s_op{};
  static OPERATION_TELEM22   s_tx{};

  const IDLE_FRAME& idleRx()        { return s_idle; }
  const OPERATION_RX_FRAME& opRx()  { return s_op;   }

  // ----- local helpers: map cmd -> group / payload len (copy logic from Protocol) -----
  static Protocol::Group groupOfCmd(uint8_t cmd)
  {
    using namespace Protocol;

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
        cmd == (uint8_t)Cmd::CMD_OP_BRAKE ||
        cmd == (uint8_t)Cmd::CMD_OP_LEGACY_CTRL_5B) {
      return Group::OPERATION;
    }

    // IDLE
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

  static uint8_t payloadLenCmd(uint8_t cmd)
  {
    using namespace Protocol;
    switch ((Cmd)cmd) {
      // u16 payload (LE)
      case Cmd::CMD_OP_PWM_FWD:
      case Cmd::CMD_OP_PWM_BWD:
      case Cmd::CMD_IDLE_ACTUATOR_SERVO_WRITE:
      case Cmd::CMD_IDLE_ACTUATOR_MOTOR_PWM_FWD:
      case Cmd::CMD_IDLE_ACTUATOR_MOTOR_PWM_BWD:
      case Cmd::CMD_IDLE_ACTUATOR_MOTOR_SPD_FWD:
      case Cmd::CMD_IDLE_ACTUATOR_MOTOR_SPD_BWD:
        return 2;

      // legacy OP5B payload: 4 bytes (speed_u16 + angle_u16)
      case Cmd::CMD_OP_LEGACY_CTRL_5B:
        return 4;

      // 30-byte params
      case Cmd::CMD_IDLE_SET_LINE_PARAMS:
      case Cmd::CMD_IDLE_SET_MPU_PARAMS:
      case Cmd::CMD_IDLE_SET_ULTRA_PARAMS:
      case Cmd::CMD_IDLE_SET_MOTOR_PARAMS:
      case Cmd::CMD_IDLE_SET_SERVO_PARAMS:
      case Cmd::CMD_IDLE_SET_PID_PARAMS:
        return 30;

      // float32 payload (speed)
      case Cmd::CMD_OP_SPD_FWD:
      case Cmd::CMD_OP_SPD_BWD:
        return 4;

      default:
        return 0;
    }
  }

  static float get_f32_le(const uint8_t* p)
  {
    float x = 0.0f;
    uint8_t b[4] = {p[0], p[1], p[2], p[3]};
    memcpy(&x, b, 4);
    return x;
  }

  // ----- frame update helpers -----
  static void updateIdle(uint8_t cmd, const uint8_t* payload, uint8_t len)
  {
    s_idle.cmd = cmd;
    s_idle.has_u16 = false;
    s_idle.has_params30 = false;

    if (len == 2 && payload) {
      s_idle.u16 = get_u16_le(payload);
      s_idle.has_u16 = true;
    } else if (len == 30 && payload) {
      for (int i = 0; i < 30; ++i) s_idle.params30[i] = payload[i];
      s_idle.has_params30 = true;
    }

    s_idle.seq++;
  }

  static void updateOp(uint8_t cmd, const uint8_t* payload, uint8_t len)
  {
    s_op.cmd = cmd;
    s_op.has_u16 = false;
    s_op.has_op5b = false;

    if (len == 2 && payload) {
      s_op.u16 = get_u16_le(payload);
      s_op.has_u16 = true;
    } else if (len == 4 && payload) {
      s_op.speed_u16 = get_u16_le(&payload[0]);
      s_op.angle_u16 = get_u16_le(&payload[2]);
      s_op.has_op5b = true;
    } else if (len == 4 && payload) {
      // (reserved for f32 if you want to store it here later)
    }

    s_op.seq++;
  }

  // ----- RX poll -----
  bool rxPoll(Network& net, Protocol::Group& outGroup, uint8_t& outCmd)
  {
    uint8_t cmd = 0;
    if (!net.getUint8(cmd)) return false;

    outGroup = groupOfCmd(cmd);
    outCmd   = cmd;

    const uint8_t len = payloadLenCmd(cmd);

    uint8_t buf30[30];
    const uint8_t* payload = nullptr;

    if (len > 0) {
      if (!net.getArrayUint8(buf30, len)) return false;
      payload = buf30;
    }

    if (outGroup == Protocol::Group::IDLE) {
      updateIdle(cmd, payload, len);
    } else if (outGroup == Protocol::Group::OPERATION) {
      // Keep storing u16/OP5B here; float32 is handled by Protocol::tryRead in app.cpp
      updateOp(cmd, payload, len);
    }

    return true;
  }

  // ----- Telemetry 22 bytes (NEW LAYOUT) -----
  void updateOpTelem22(const uint8_t line5[5],
                       const int16_t mpu6_i16[6],
                       int8_t enc_count_dt_i8,
                       int16_t speed_hz_x100_i16,
                       int16_t steer_deg_i16)
  {
    uint8_t* p = s_tx.data;

    // 5*u8 line
    for (int i = 0; i < 5; ++i) p[i] = line5 ? line5[i] : 0;
    int idx = 5;

    // 6*i16 mpu
    for (int k = 0; k < 6; ++k) {
      put_u16_le(&p[idx], (uint16_t)(mpu6_i16 ? mpu6_i16[k] : 0));
      idx += 2;
    }

    // i8 enc_count_dt
    p[idx++] = (uint8_t)enc_count_dt_i8;

    // i16 speed (Hz*100)
    put_u16_le(&p[idx], (uint16_t)speed_hz_x100_i16); idx += 2;

    // i16 steer_deg
    put_u16_le(&p[idx], (uint16_t)steer_deg_i16); idx += 2;

    s_tx.seq++;
  }

  bool sendOpTelem22(Network& net)
  {
    return net.opSendTelemetry22(s_tx.data);
  }

} // namespace Telemetry
