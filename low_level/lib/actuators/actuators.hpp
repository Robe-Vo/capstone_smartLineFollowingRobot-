// esp32/lib/actuators/actuators.hpp
#pragma once
#include <Arduino.h>
#include "esp_timer.h"
#include "driver/gpio.h"
#include "../../src/state/state.hpp"

#include "../../include/cfg.hpp"

/*
  Actuators + Encoder hybrid measurement (Period + Accumulate)
*/

namespace Encoder {

/* ------------ Config ------------ */
struct HybridConfig {
  // Period window constraints (Tw range)
  float Tw_min_s = ENC_TW_MIN_S;
  float Tw_max_s = ENC_TW_MAX_S;

  // Hybrid blend thresholds (Hz)
  float fLow_hz  = ENC_F_LOW_HZ;
  float fHigh_hz = ENC_F_HIGH_HZ;

  // IIR time constant + alpha bounds
  float tau_iir_s = ENC_TAU_IIR_S;
  float alpha_min = ENC_ALPHA_MIN;
  float alpha_max = ENC_ALPHA_MAX;

  // Jerk limiter (Hz/s)
  float a_max_hz_per_s = ENC_A_MAX_HZ_PER_S;

  // Zero-speed logic
  float k_zero      = ENC_K_ZERO;
  float Tmin_zero_s = ENC_TMIN_ZERO_S;
  float tau_zero_s  = ENC_TAU_ZERO_S;
  float f_dead_hz   = ENC_F_DEAD_HZ;
  float Tdead_s     = ENC_TDEAD_S;

  // Physical clamp (Hz)
  float f_hard_max_hz = ENC_F_HARD_MAX_HZ;

  // Glitch reject window trên dt
  float soft_dt_min_ratio = ENC_SOFT_DT_MIN_RATIO;
  float soft_dt_max_ratio = ENC_SOFT_DT_MAX_RATIO;
};

namespace Encoder {
  float speed_get_hz_out();  // đã có
  float speed_get_mps();     // bạn thêm: dùng đường kính bánh + gear + PPR
}

extern portMUX_TYPE mux_enc;

/* ------------ API ------------ */
void speed_filter_init();
void speed_filter_config(uint8_t mode, uint16_t a_raw, uint16_t b_raw);

void encoder_begin(uint8_t pinA, uint8_t pinB, uint16_t ppr_effective, const HybridConfig& cfg);

// Legacy compat: wrapper/no-op
void speed_filter_update(float raw);

// Gọi mỗi vòng điều khiển với dt_s
void hybrid_update(float dt_s);

// Accessors
float  speed_get_hz_raw_period();
float  speed_get_hz_raw_accum();
float  speed_get_hz_hybrid();
float  speed_get_hz_iir();
float  speed_get_hz_out();

int32_t  encoder_get_total();
int8_t   encoder_get_dir();
uint32_t encoder_get_period_us();
uint32_t encoder_get_last_pulse_ms();
int32_t  encoder_snapshot_delta_and_reset_accum();

// ISR encoder A+B
void IRAM_ATTR isr_encoder_AB();

} // namespace Encoder

/* ======================= Motor PWM 11-bit helper ======================= */
namespace MotorPWM11 {

static constexpr uint8_t  PWM_BITS = 11;
static constexpr uint16_t PWM_MAX  = (1u << PWM_BITS) - 1u; // 2047
static constexpr uint16_t SPEED_MAX_INPUT = (1u << PWM_BITS); // 2048

void begin(uint8_t pwm_pin, uint8_t out1_pin, uint8_t out2_pin,
           uint8_t ledc_channel, uint32_t pwm_freq_hz);

void driveForward(uint16_t speed_u16);
void driveBackward(uint16_t speed_u16);
void stopMotor();

uint16_t clamp_speed_to_duty(uint16_t speed_u16);

// Deadband setting (defined in actuators.cpp)
void setDeadband(uint16_t db);
uint16_t getDeadband();

} // namespace MotorPWM11
