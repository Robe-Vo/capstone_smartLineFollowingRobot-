#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "../../include/pin.hpp"
#include "../../include/cfg.hpp"

/*
 * === Encoder actuator interface ===
 *
 * This namespace provide Encoder counting and speed.
 */
namespace Encoder
{

  enum class SpeedMode : uint8_t { PERIOD_B = 0, ACC_AB = 1 };

  // Calculation buffer
  
  static uint32_t timer = 0;

  // Setup pin
  void setup();
  
  // Encoder ISR function
  void IRAM_ATTR ISR_encoder_channel_A();
  void IRAM_ATTR ISR_encoder_channel_B();

  // Get accumulation
  int8_t get_count();

  // Check if motor is rotation
  // If not, reset count
  bool isRotating();
    

  // Measure speed of motor
  void calculate_vec(uint16_t dt_ms);

  // Speed accessors
  float get_iir_vec_hz(); // After IIR filter
  float get_vec_hz();
  float get_raw_vec_hz();
  SpeedMode get_mode();
}