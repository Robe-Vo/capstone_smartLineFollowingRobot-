// ======================= capstone/esp32/src/telemetry/telemetry.hpp (REWRITE) =======================
#pragma once
#include <stdint.h>

#include "bluetooth.hpp"
#include "Protocol/protocol.hpp"

namespace Telemetry {

  // ===================== RX frames (updated by Telemetry::rxPoll) =====================
  struct IDLE_FRAME {
    uint8_t  cmd = 0;
    bool     has_u16 = false;
    uint16_t u16 = 0;          // LE-decoded generic payload (angle/pwm/speed...)
    bool     has_params30 = false;
    uint8_t  params30[30]{};
    uint32_t seq = 0;
  };

  struct OPERATION_RX_FRAME {
    uint8_t  cmd = 0;
    bool     has_u16 = false;
    uint16_t u16 = 0;          // speed, pwm,... tùy cmd
    bool     has_op5b = false;
    uint16_t speed_u16 = 0;    // legacy OP5B
    uint16_t angle_u16 = 0;    // legacy OP5B
    uint32_t seq = 0;
  };

  // 22-byte telemetry frame + counter
  // NEW LAYOUT (22 bytes):
  //  5*u8 line
  //  6*i16 mpu (12 bytes)
  //  1*i8  enc_count_dt (pulse count within dt, clamped to int8)
  //  1*i16 speed_hz_x100
  //  1*i16 steer_deg (or steer target)
  struct OPERATION_TELEM22 {
    uint8_t  data[22]{};
    uint32_t seq = 0;
  };

  const IDLE_FRAME&          idleRx();
  const OPERATION_RX_FRAME&  opRx();

  // Read 1 command from Network, update corresponding RX frame, return group + cmd
  bool rxPoll(Network& net, Protocol::Group& outGroup, uint8_t& outCmd);

  // ===================== TX telemetry (22 bytes) =====================
  void updateOpTelem22(const uint8_t line5[5],
                       const int16_t mpu6_i16[6],
                       int8_t   enc_count_dt_i8,
                       int16_t  speed_hz_x100_i16,
                       int16_t  steer_deg_i16);

  bool sendOpTelem22(Network& net);

  // Helpers
  inline uint16_t get_u16_le(const uint8_t* p){
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
  }
  inline void put_u16_le(uint8_t* p, uint16_t v){
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
  }

} // namespace Telemetry
