/**
 *  This is the library for line following robot using two differential drives
 *  and Ackermann mechanism for driving direction.
 */

#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>
#include <stdint.h>
namespace Encoder
{
    void init(uint16_t ppr,
              uint16_t dt_speed_ms,
              uint16_t accum_window_ms,
              float    alpha);

    void updateCount();      // ISR
    void updateSpeed();      // gọi mỗi dt_speed_ms
    float getRPM();
}

namespace Motor
{
    void init();
    void setPWM(int pwm);
}
