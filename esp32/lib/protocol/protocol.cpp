// ======================= protocol.cpp =======================
#include "protocol.hpp"

namespace Protocol
{
  CmdDesc g_cmdTable[256];

  static inline CmdDesc makeDesc(Group g, Payload p){ CmdDesc d{g,p}; return d; }

  float half_to_float(uint16_t h)
  {
    uint32_t sign = (uint32_t)(h & 0x8000) << 16;
    uint32_t exp  = (h >> 10) & 0x1F;
    uint32_t mant = h & 0x03FF;

    uint32_t f;
    if (exp == 0) {
      if (mant == 0) {
        f = sign;
      } else {
        exp = 1;
        while ((mant & 0x0400) == 0) { mant <<= 1; exp--; }
        mant &= 0x03FF;
        uint32_t exp_f  = (exp + (127 - 15)) << 23;
        uint32_t mant_f = mant << 13;
        f = sign | exp_f | mant_f;
      }
    } else if (exp == 31) {
      uint32_t exp_f  = 0xFFu << 23;
      uint32_t mant_f = mant << 13;
      f = sign | exp_f | mant_f;
    } else {
      uint32_t exp_f  = (exp + (127 - 15)) << 23;
      uint32_t mant_f = mant << 13;
      f = sign | exp_f | mant_f;
    }

    float out;
    memcpy(&out, &f, 4);
    return out;
  }

  void initDescriptorTable()
  {
    // default UNKNOWN/NONE
    for (int i = 0; i < 256; ++i) g_cmdTable[i] = makeDesc(Group::UNKNOWN, Payload::NONE);

    // prefix-based group (0xF_=SYS, 0xE_=OP, 0xD_/0xC_/0xB_=IDLE)
    for (int i = 0; i < 256; ++i) {
      uint8_t hi = (uint8_t)i & 0xF0;
      if (hi == 0xF0) g_cmdTable[i].group = Group::SYSTEM;
      else if (hi == 0xE0) g_cmdTable[i].group = Group::OPERATION;
      else if (hi == 0xD0 || hi == 0xC0 || hi == 0xB0) g_cmdTable[i].group = Group::IDLE;
    }

    // SYSTEM: no payload
    g_cmdTable[(uint8_t)Cmd::CMD_GLOBAL_OPERATION].payload     = Payload::NONE;
    g_cmdTable[(uint8_t)Cmd::CMD_GLOBAL_IDLE].payload          = Payload::NONE;
    g_cmdTable[(uint8_t)Cmd::CMD_GLOBAL_PING_MODE].payload     = Payload::NONE;
    g_cmdTable[(uint8_t)Cmd::CMD_GLOBAL_EMRGENCY_STOP].payload = Payload::NONE;

    // OPERATION
    g_cmdTable[(uint8_t)Cmd::CMD_OP_PWM_FWD].payload = Payload::OP_PWM;
    g_cmdTable[(uint8_t)Cmd::CMD_OP_PWM_BWD].payload = Payload::OP_PWM;
    g_cmdTable[(uint8_t)Cmd::CMD_OP_SPD_FWD].payload = Payload::OP_SPD;
    g_cmdTable[(uint8_t)Cmd::CMD_OP_SPD_BWD].payload = Payload::OP_SPD;
    g_cmdTable[(uint8_t)Cmd::CMD_OP_BRAKE].payload   = Payload::NONE;

    // IDLE MOTOR
    g_cmdTable[(uint8_t)Cmd::CMD_IDLE_ACTUATOR_MOTOR_ENABLE].payload  = Payload::NONE;
    g_cmdTable[(uint8_t)Cmd::CMD_IDLE_ACTUATOR_MOTOR_DISABLE].payload = Payload::NONE;
    g_cmdTable[(uint8_t)Cmd::CMD_IDLE_ACTUATOR_MOTOR_SPD_FWD].payload = Payload::I16_LE; // 2B speed (encoding below)
    g_cmdTable[(uint8_t)Cmd::CMD_IDLE_ACTUATOR_MOTOR_SPD_BWD].payload = Payload::I16_LE;
    g_cmdTable[(uint8_t)Cmd::CMD_IDLE_ACTUATOR_MOTOR_PWM_FWD].payload = Payload::U16_LE;
    g_cmdTable[(uint8_t)Cmd::CMD_IDLE_ACTUATOR_MOTOR_PWM_BWD].payload = Payload::U16_LE;
    g_cmdTable[(uint8_t)Cmd::CMD_IDLE_ACTUATOR_MOTOR_BRAKE].payload   = Payload::NONE;

    // IDLE SERVO
    g_cmdTable[(uint8_t)Cmd::CMD_IDLE_ACTUATOR_SERVO_ENABLE].payload       = Payload::NONE;
    g_cmdTable[(uint8_t)Cmd::CMD_IDLE_ACTUATOR_SERVO_DISABLE].payload      = Payload::NONE;
    g_cmdTable[(uint8_t)Cmd::CMD_IDLE_ACTUATOR_SERVO_WRITE].payload        = Payload::I16_LE;
    g_cmdTable[(uint8_t)Cmd::CMD_IDLE_ACTUATOR_SERVO_WRITE_CENTER].payload = Payload::NONE;
    g_cmdTable[(uint8_t)Cmd::CMD_IDLE_ACTUATOR_SERVO_READ].payload         = Payload::NONE;

    // IDLE SENSOR (request frames have no payload)
    g_cmdTable[(uint8_t)Cmd::CMD_IDLE_SENSOR_LINE_READ].payload  = Payload::NONE;
    g_cmdTable[(uint8_t)Cmd::CMD_IDLE_SENSOR_ULTRA_READ].payload = Payload::NONE;
    g_cmdTable[(uint8_t)Cmd::CMD_IDLE_SENSOR_MPU_READ].payload   = Payload::NONE;

    // IDLE SET PARAMS (unify to 30 bytes)
    g_cmdTable[(uint8_t)Cmd::CMD_IDLE_SET_LINE_PARAMS].payload  = Payload::BYTES30;
    g_cmdTable[(uint8_t)Cmd::CMD_IDLE_SET_MPU_PARAMS].payload   = Payload::BYTES30;
    g_cmdTable[(uint8_t)Cmd::CMD_IDLE_SET_ULTRA_PARAMS].payload = Payload::BYTES30;
    g_cmdTable[(uint8_t)Cmd::CMD_IDLE_SET_MOTOR_PARAMS].payload = Payload::BYTES30;
    g_cmdTable[(uint8_t)Cmd::CMD_IDLE_SET_SERVO_PARAMS].payload = Payload::BYTES30;
    g_cmdTable[(uint8_t)Cmd::CMD_IDLE_SET_PID_PARAMS].payload   = Payload::BYTES30;
  }

  uint8_t payloadLen(uint8_t cmd_u8)
  {
    Payload p = payloadOf(cmd_u8);
    switch (p) {
      case Payload::NONE:    return 0;
      case Payload::U16_LE:  return 2;
      case Payload::I16_LE:  return 2;
      case Payload::BYTES30: return 30;
      case Payload::OP_PWM:  return 4; // u16 pwm + i16 angle
      case Payload::OP_SPD:
      {
#if defined(PROTOCOL_OP_SPEED_I16X100) || defined(PROTOCOL_OP_SPEED_FLOAT16)
        return 4; // 2B speed + 2B angle
#else
        return 6; // float32 + i16 angle (NOT 2 bytes)
#endif
      }
      default: return 0;
    }
  }

  bool tryRead(Network& net, RxFrame& out)
  {
    uint8_t cmd_u8 = 0;
    if (!net.getUint8(cmd_u8)) return false;

    out.cmd     = (Cmd)cmd_u8;
    out.group   = groupOf(cmd_u8);
    out.payload = payloadOf(cmd_u8);

    out.u16 = 0;
    out.i16 = 0;
    memset(out.bytes30, 0, sizeof(out.bytes30));
    out.pwm_u16 = 0;
    out.angle_i16 = 0;
    out.speed_f = 0.0f;

    switch (out.payload)
    {
      case Payload::NONE:
        return true;

      case Payload::U16_LE:
      {
        uint8_t b[2];
        if (!net.getArrayUint8(b, 2)) return false;
        out.u16 = get_u16_le(b);
        return true;
      }

      case Payload::I16_LE:
      {
        uint8_t b[2];
        if (!net.getArrayUint8(b, 2)) return false;
        out.i16 = get_i16_le(b);
        return true;
      }

      case Payload::BYTES30:
      {
        if (!net.getArrayUint8(out.bytes30, 30)) return false;
        return true;
      }

      case Payload::OP_PWM:
      {
        uint8_t b[4];
        if (!net.getArrayUint8(b, 4)) return false;
        out.pwm_u16   = get_u16_le(&b[0]);
        out.angle_i16 = get_i16_le(&b[2]);
        return true;
      }

      case Payload::OP_SPD:
      {
#if defined(PROTOCOL_OP_SPEED_I16X100)
        uint8_t b[4];
        if (!net.getArrayUint8(b, 4)) return false;
        int16_t sp = get_i16_le(&b[0]);     // Hz*100
        out.speed_f = (float)sp / 100.0f;
        out.angle_i16 = get_i16_le(&b[2]);
        return true;

#elif defined(PROTOCOL_OP_SPEED_FLOAT16)
        uint8_t b[4];
        if (!net.getArrayUint8(b, 4)) return false;
        uint16_t hf = get_u16_le(&b[0]);    // float16
        out.speed_f = half_to_float(hf);
        out.angle_i16 = get_i16_le(&b[2]);
        return true;

#else
        uint8_t b[6];
        if (!net.getArrayUint8(b, 6)) return false;
        float v;
        memcpy(&v, &b[0], 4);
        out.speed_f = v;
        out.angle_i16 = get_i16_le(&b[4]);
        return true;
#endif
      }

      default:
        return false;
    }
  }


  // OPERATION signals interface
  // | 5*u8 line | 1*u16 ultra | 6*i16 mpu | 1*i8 enc | 1 "float" (2 bytes) |
  // speed encoding (2 bytes) selected by compile-time flags:
  //   -DPROTOCOL_OP_SPEED_I16X100   : int16 = round(speed*100)
  //   -DPROTOCOL_OP_SPEED_FLOAT16   : IEEE754 float16
  // If neither is defined, it will still pack float16 (2 bytes) using the float16 converter below.
  void package_op_signals(uint8_t* buffer,
                          uint8_t* line,
                          uint16_t ultra,
                          int16_t*  mpu,
                          int8_t    enc,
                          float     speed)
  {
    uint16_t idx = 0;

    // 1) line[5]
    for (uint8_t i = 0; i < 5; ++i) buffer[idx++] = line[i];

    // 2) ultra u16 LE
    buffer[idx++] = (uint8_t)(ultra & 0xFF);
    buffer[idx++] = (uint8_t)((ultra >> 8) & 0xFF);

    // 3) mpu[6] i16 LE
    for (uint8_t i = 0; i < 6; ++i) {
      uint16_t v = (uint16_t)mpu[i];
      buffer[idx++] = (uint8_t)(v & 0xFF);
      buffer[idx++] = (uint8_t)((v >> 8) & 0xFF);
    }

    // 4) enc i8
    buffer[idx++] = (uint8_t)enc;

    // 5) speed (2 bytes)
  #if defined(PROTOCOL_OP_SPEED_I16X100)
    // int16 Hz*100
    float scaled_f = speed * 100.0f;
    int32_t scaled_i = (scaled_f >= 0.0f) ? (int32_t)(scaled_f + 0.5f) : (int32_t)(scaled_f - 0.5f);
    if (scaled_i > 32767)  scaled_i = 32767;
    if (scaled_i < -32768) scaled_i = -32768;
    uint16_t sp16 = (uint16_t)(int16_t)scaled_i;
    buffer[idx++] = (uint8_t)(sp16 & 0xFF);
    buffer[idx++] = (uint8_t)((sp16 >> 8) & 0xFF);

  #else
    // float16 IEEE754
    // Pack float->half without calling any extra helper functions.
    union { float f; uint32_t u; } in;
    in.f = speed;

    uint32_t sign = (in.u >> 31) & 0x1;
    int32_t  exp  = (int32_t)((in.u >> 23) & 0xFF);
    uint32_t frac = in.u & 0x7FFFFF;

    uint16_t h = 0;

    if (exp == 255) {
      // Inf/NaN
      if (frac == 0) {
        h = (uint16_t)((sign << 15) | (0x1F << 10));               // Inf
      } else {
        h = (uint16_t)((sign << 15) | (0x1F << 10) | 0x0200);     // qNaN
      }
    } else if (exp == 0) {
      // Zero / subnormal float32 -> zero half (keep sign)
      h = (uint16_t)(sign << 15);
    } else {
      // Normal float32
      int32_t exp16 = exp - 127 + 15;

      if (exp16 >= 31) {
        // Overflow -> Inf
        h = (uint16_t)((sign << 15) | (0x1F << 10));
      } else if (exp16 <= 0) {
        // Underflow -> subnormal half (or zero)
        // Convert mantissa with implicit leading 1 then shift.
        // mant32 = 1.frac (24 bits)
        uint32_t mant32 = (1u << 23) | frac;

        // shift to get to half subnormal: need (1 - exp16) extra shifts beyond normal alignment
        int32_t shift = (1 - exp16) + (23 - 10); // brings mantissa to 10 bits
        if (shift >= 31) {
          h = (uint16_t)(sign << 15); // too small -> zero
        } else {
          // Round to nearest even
          uint32_t mant = mant32 >> shift;
          uint32_t rem  = mant32 & ((1u << shift) - 1u);
          uint32_t half = 1u << (shift - 1);
          bool round_up = (rem > half) || (rem == half && (mant & 1u));
          if (round_up) mant += 1u;

          h = (uint16_t)((sign << 15) | (uint16_t)(mant & 0x03FF));
        }
      } else {
        // Normal half
        // Take top 10 bits of frac (with rounding)
        uint32_t frac_round = frac;

        // Round-to-nearest-even at bit 13 (since 23->10: drop 13 bits)
        uint32_t mant = frac_round >> 13;
        uint32_t rem  = frac_round & 0x1FFF;
        uint32_t half = 0x1000;
        bool round_up = (rem > half) || (rem == half && (mant & 1u));
        if (round_up) {
          mant += 1u;
          if (mant == 0x400) { // mantissa overflow -> increment exponent
            mant = 0;
            exp16 += 1;
            if (exp16 >= 31) {
              h = (uint16_t)((sign << 15) | (0x1F << 10)); // Inf
              buffer[idx++] = (uint8_t)(h & 0xFF);
              buffer[idx++] = (uint8_t)((h >> 8) & 0xFF);
              return;
            }
          }
        }

        h = (uint16_t)((sign << 15) | ((uint16_t)exp16 << 10) | (uint16_t)(mant & 0x03FF));
      }
    }

    buffer[idx++] = (uint8_t)(h & 0xFF);
    buffer[idx++] = (uint8_t)((h >> 8) & 0xFF);
  #endif
  }

} // namespace Protocol
  