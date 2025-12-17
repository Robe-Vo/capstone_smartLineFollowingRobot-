#include "telemetry.hpp"

namespace Telemetry {

  static IDLE_FRAME s_idle{};
  static OPERATION_RX_FRAME s_op{};
  static OPERATION_TELEM22 s_tx{};

  const IDLE_FRAME& idleRx() { return s_idle; }
  const OPERATION_RX_FRAME& opRx() { return s_op; }

  static void updateIdle(uint8_t cmd, const uint8_t* payload, uint8_t len)
  {
    s_idle.cmd = cmd;
    s_idle.has_u16 = false;
    s_idle.has_params30 = false;

    if (len == 2 && payload) {
      s_idle.u16 = get_u16_le(payload);
      s_idle.has_u16 = true;
    } else if (len == 30 && payload) {
      for (int i=0;i<30;i++) s_idle.params30[i] = payload[i];
      s_idle.has_params30 = true;
    }

    s_idle.seq++;
  }

  static void updateOp(uint8_t cmd, const uint8_t* payload, uint8_t len)
  {
    s_op.cmd = cmd;
    s_op.has_u16 = false;

    if (len == 2 && payload) {
      s_op.u16 = get_u16_le(payload);
      s_op.has_u16 = true;
    }

    s_op.seq++;
  }

  bool rxPoll(Network& net, Protocol::Group& outGroup)
  {
    uint8_t cmd = 0;
    if (!net.getUint8(cmd)) return false;

    outGroup = Protocol::groupOf(cmd);
    const uint8_t len = Protocol::payloadLen(cmd);

    // If payload needed, ensure we can read it in this poll
    uint8_t buf30[30];
    const uint8_t* payload = nullptr;

    if (len > 0) {
      if (!net.getArrayUint8(buf30, len)) return false;
      payload = buf30;
    }

    // Update frames based on group
    if (outGroup == Protocol::Group::IDLE) {
      updateIdle(cmd, payload, len);
    } else if (outGroup == Protocol::Group::OPERATION) {
      updateOp(cmd, payload, len);
    } else {
      // SYSTEM/UNKNOWN: no frames updated here; app can check cmd directly if desired
    }

    return true;
  }

  void updateOpTelem22(const uint8_t line5[5],
                       uint16_t ultra_u16,
                       const int16_t mpu6_i16[6],
                       int8_t enc_count_i8,
                       int16_t speed_i16)
  {
    uint8_t* p = s_tx.data;

    // 5*u8 line
    for (int i=0;i<5;i++) p[i] = line5 ? line5[i] : 0;
    int idx = 5;

    // u16 ultra
    put_u16_le(&p[idx], ultra_u16); idx += 2;

    // 6*i16 mpu
    for (int k=0;k<6;k++) {
      put_u16_le(&p[idx], (uint16_t)(mpu6_i16 ? mpu6_i16[k] : 0));
      idx += 2;
    }

    // i8 enc
    p[idx++] = (uint8_t)enc_count_i8;

    // i16 speed
    put_u16_le(&p[idx], (uint16_t)speed_i16); idx += 2;

    s_tx.seq++;
  }

  bool sendOpTelem22(Network& net)
  {
    return net.opSendTelemetry22(s_tx.data);
  }

} // namespace Telemetry
