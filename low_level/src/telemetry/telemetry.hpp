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

    uint32_t seq = 0;          // increments on update
  };

  struct OPERATION_RX_FRAME {
    uint8_t  cmd = 0;
    bool     has_u16 = false;
    uint16_t u16 = 0;          // LE-decoded payload (pwm11/speed_u16...)
    uint32_t seq = 0;          // increments on update
  };

  // ===================== TX telemetry (22B) =====================
  struct OPERATION_REV_FRAME {
    uint8_t data[22]{};
    uint32_t seq = 0;          // increments on update
  };

  // ===================== API =====================

  // Poll RX: reads at most one full cmd+payload per call (non-blocking).
  // Returns true if a command was fully read and frames updated.
  bool rxPoll(Network& net, Protocol::Group& outGroup);

  // Access latest decoded RX frames (pull model)
  const IDLE_FRAME& idleRx();
  const OPERATION_RX_FRAME& opRx();

  // Pack + send 22B telemetry (push)
  void updateOpTelem22(const uint8_t line5[5],
                       uint16_t ultra_u16,
                       const int16_t mpu6_i16[6],
                       int8_t enc_count_i8,
                       int16_t speed_i16);

  bool sendOpTelem22(Network& net);

  // Helpers (kept here only)
  inline uint16_t get_u16_le(const uint8_t* p){
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
  }
  inline void put_u16_le(uint8_t* p, uint16_t v){
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
  }

} // namespace Telemetry
