#pragma once
#include <Arduino.h>
#include "esp_timer.h"
#include "driver/gpio.h"

/*
  Actuators + Encoder hybrid measurement (Period + Accumulate) for quick prototype.

  Requirements implemented:
  - Encoder -> Period / Accumulate
           -> HARD clamp (physical) + SOFT clamp (prediction)
           -> Hybrid blend (speed-based)
           -> IIR1 (EWMA, alpha adaptive by dt/tau)
           -> Jerk limiter
           -> (Output for inner loop / telemetry)
  - Position from total encoder count (unfiltered)

  NOTE (per user request): "encoder resolution = 1"
  => This implementation exposes speed in "Hz" = pulses/second of the chosen count domain.
     By setting ENCODER_PPR_EFFECTIVE = 1, Hz directly equals measured pulse frequency.

  IMPORTANT:
  - ISR uses esp_timer_get_time() and gpio_get_level() (ISR-safe on ESP32).
  - Shared variables protected by portMUX.
*/

namespace Encoder {

/* ------------ Config ------------ */
struct HybridConfig {
  // Period window constraints (Tw range) used as HARD clamp on period
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

  // Zero-speed (period-friendly)
  float k_zero = 3.0f;
  float Tmin_zero_s = 0.035f; // 30–40 ms
  float tau_zero_s  = 0.060f; // 50–80 ms
  float f_dead_hz   = 1.0f;
  float Tdead_s     = 0.060f;

  // HARD clamp by physical max speed in Hz (pulse/s)
  float f_hard_max_hz = 300.0f;

  // SOFT clamp ratios around predicted dt (period glitch reject)
  float soft_dt_min_ratio = 0.35f;
  float soft_dt_max_ratio = 2.50f;
};

/* ------------ API (giữ tên hàm gần cấu trúc cũ) ------------ */
// Init module / filter
void speed_filter_init();
void speed_filter_config(uint8_t mode, uint16_t a_raw, uint16_t b_raw); // keep signature (compat)

// Encoder wiring init
void encoder_begin(uint8_t pinA, uint8_t pinB, uint16_t ppr_effective /*set 1*/, const HybridConfig& cfg);

// Update pipeline with a new raw speed sample (compat stub; can still call)
void speed_filter_update(float raw);

// New: update hybrid measurement using dt from your scheduler (e.g., TS_SPEED_MS)
void hybrid_update(float dt_s, uint32_t now_ms);

// Telemetry getters
float  speed_get_hz_raw_period();   // after clamps (period path)
float  speed_get_hz_raw_accum();    // accumulate path
float  speed_get_hz_hybrid();       // before IIR
float  speed_get_hz_iir();          // after IIR
float  speed_get_hz_out();          // after jerk limiter + zero logic

// Position (unfiltered)
int32_t encoder_get_total();
int8_t  encoder_get_dir();
uint32_t encoder_get_period_us();   // raw period
uint32_t encoder_get_last_pulse_ms();

// Frame delta pulses (for "frequency of sampling" diagnostics)
int32_t encoder_snapshot_delta_and_reset_accum();

// Internal ISR (called by attachInterrupt)
void IRAM_ATTR isr_encoder_AB();

/* ------------ Shared state (extern for quick reuse if needed) ------------ */
extern portMUX_TYPE mux_enc;

} // namespace Encoder


/* Motor PWM 16-bit helper (kept in actuators library for quick paste) */
namespace MotorPWM16 {
void begin(uint8_t pwm_pin, uint8_t out1_pin, uint8_t out2_pin,
           uint8_t ledc_channel, uint32_t pwm_freq_hz, uint8_t pwm_res_bits);

void driveForward(uint16_t pwm);
void driveBackward(uint16_t pwm);
void stopMotor();
}
