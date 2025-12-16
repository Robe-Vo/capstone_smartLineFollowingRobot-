#pragma once
#include <Arduino.h>
#include "esp_timer.h"
#include "driver/gpio.h"

/*
  Actuators + Encoder hybrid measurement (Period + Accumulate)

  PWM requirement:
  - Receive speed_u16 from PC already in range 0..2^11 (0..2048)
  - If > 2^11 => saturate at 2^11
  - Hardware LEDC 11-bit duty is 0..2047 => we map any 2048 to 2047.

  Encoder:
  - Quadrature x4 counting (interrupt on both A and B, CHANGE)
  - Period measurement uses dt between valid edges
  - Hybrid: period (low speed) + accumulate (high speed) with IIR + jerk limit + zero-speed logic
*/

namespace Encoder {

/* ------------ Config ------------ */
struct HybridConfig {
  // Period window constraints (Tw range) used as HARD clamp on period magnitude
  float Tw_min_s = 0.005f;  // 5 ms
  float Tw_max_s = 0.007f;  // 7 ms

  // Hybrid blend thresholds (Hz)
  float fLow_hz  = 40.0f;
  float fHigh_hz = 80.0f;

  // EWMA time constant
  float tau_iir_s = 0.050f; // 50 ms
  float alpha_min = 0.05f;
  float alpha_max = 0.30f;

  // Jerk limiter (Hz/s)
  float a_max_hz_per_s = 500.0f;

  // Zero-speed logic
  float k_zero = 3.0f;
  float Tmin_zero_s = 0.035f;
  float tau_zero_s  = 0.060f;
  float f_dead_hz   = 1.0f;
  float Tdead_s     = 0.060f;

  // Physical max clamp (Hz)
  float f_hard_max_hz = 300.0f;

  // Period glitch reject around predicted dt
  float soft_dt_min_ratio = 0.35f;
  float soft_dt_max_ratio = 2.50f;
};

extern portMUX_TYPE mux_enc;

/* ------------ API ------------ */
void speed_filter_init();
void speed_filter_config(uint8_t mode, uint16_t a_raw, uint16_t b_raw);

void encoder_begin(uint8_t pinA, uint8_t pinB, uint16_t ppr_effective, const HybridConfig& cfg);

void speed_filter_update(float raw); // compat stub

void hybrid_update(float dt_s);

float  speed_get_hz_raw_period();
float  speed_get_hz_raw_accum();
float  speed_get_hz_hybrid();
float  speed_get_hz_iir();
float  speed_get_hz_out();

int32_t encoder_get_total();
int8_t  encoder_get_dir();
uint32_t encoder_get_period_us();
uint32_t encoder_get_last_pulse_ms();
int32_t  encoder_snapshot_delta_and_reset_accum();

void IRAM_ATTR isr_encoder_AB();

} // namespace Encoder


/* ======================= Motor PWM 11-bit helper ======================= */
namespace MotorPWM11 {

// LEDC 11-bit duty range
static constexpr uint8_t  PWM_BITS = 11;
static constexpr uint16_t PWM_MAX  = (1u << PWM_BITS) - 1u; // 2047
static constexpr uint16_t SPEED_MAX_INPUT = (1u << PWM_BITS); // 2048 (per user convention)

void begin(uint8_t pwm_pin, uint8_t out1_pin, uint8_t out2_pin,
           uint8_t ledc_channel, uint32_t pwm_freq_hz);

void driveForward(uint16_t speed_u16);   // speed_u16 expected 0..2048; >2048 saturate
void driveBackward(uint16_t speed_u16);
void stopMotor();

uint16_t clamp_speed_to_duty(uint16_t speed_u16);

} // namespace MotorPWM11
