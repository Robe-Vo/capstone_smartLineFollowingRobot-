#include "system_handlers.hpp"

#include "protocol.hpp"
#include "bluetooth.hpp"
#include "esp_system.h"
#include "state.hpp"

namespace SystemHandlers
{
  void to_idle(const Protocol::RxFrame& f)
  {
    // ACK byte
    net.transmitUint8(0x20);
    IDLE_init();
    robotMode = Mode::IDLE;
  }

  void to_operation(const Protocol::RxFrame& f)
  {
    // ACK byte
    net.transmitUint8(0x20);    
    OPERATION_init();
    robotMode = Mode::OPERATION;
  }

  void ping_mode(const Protocol::RxFrame& f)
  {
    // ACK byte
    net.transmitUint8(0x20);

    if      (robotMode == Mode::IDLE)       net.transmitUint8((uint8_t)0);
    else if (robotMode == Mode::OPERATION)  net.transmitUint8((uint8_t)1);
  }

  void emergency_stop(const Protocol::RxFrame& f)
  {
    // ACK byte
    net.transmitUint8(0x20);
    esp_restart();
  }
}