#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>

#include "../../include/pin.hpp"
#include "../../include/cfg.hpp"


/*
 * === Motor actuator interface ===
 *
 * This namespace provide functions to setup and run DC motor.
 * The PWM resolution is 11-bit (0-2047).   
 */
namespace Drive
{

  // Setup motor pins
  void setup();

  // Set motor PWM duty (0-2047) and direction
  void setPWM(uint16_t duty, bool direction);

  // Stop motor
  void brake();
}

/*
 * === Servo actuator interface ===
 *
 * This namespace provide functions to setup and run Servo motor.
 */
namespace Steer
{
  // Setup servo pin
  void setup();

  // Enable servo (attach)
  void enable();

  // Disable servo (detach)
  void disable();

  // Check if servo is enabled
  inline bool isEnabled();

  // Write angle to servo (deg)
  void writeAngle(int16_t angle);
  void writeMidAngle();
}

