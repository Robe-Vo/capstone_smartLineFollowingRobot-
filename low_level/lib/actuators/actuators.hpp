// ======================= actuators.hpp =======================
#pragma once
#include <Arduino.h>

namespace Encoder
{
    // mode: 0 = EMA bậc 1, 1 = IIR bậc 2 (cascade 2 EMA)
    void  speed_filter_init();
    void  speed_filter_config(uint8_t mode, uint16_t alpha_raw, uint16_t beta_raw);
    void  speed_filter_update(float rpm_raw);
    float speed_get_rpm();
}
