#include <Arduino.h>
#include "dispatch.hpp"

#include "protocol.hpp"
#include "state.hpp"
#include <string.h>
#include "state.hpp"
#include "actuators.hpp"
#include "encoder.hpp"
#include "controller.hpp"

/**
 *          ===== OPERATION HANDLERS =====
 * 
 *  Set flag and storage data whenever they are called
 */
namespace OpHandlers 
{
  void op_spd_fwd(const Protocol::RxFrame& f);
  void op_spd_bwd(const Protocol::RxFrame& f);
  void op_pwm_fwd(const Protocol::RxFrame& f);
  void op_pwm_bwd(const Protocol::RxFrame& f);
  void op_brake(const Protocol::RxFrame& f);
}

