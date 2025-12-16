#include "actuators.hpp"
#include <math.h>

namespace Encoder {

/* ======================= ENCODER SHARED STATE ======================= */
portMUX_TYPE mux_enc = portMUX_INITIALIZER_UNLOCKED;

static volatile int32_t  encoder_pulse_total = 0;
static volatile int32_t  encoder_pulse_accum = 0;

static volatile uint8_t  enc_prev_state = 0;
static volatile int8_t   encoder_direction = +1;

static volatile uint32_t enc_last_edge_us  = 0;
static volatile uint32_t enc_period_us     = 0;
static volatile uint32_t enc_last_pulse_ms = 0;

static uint8_t  PIN_A = 0;
static uint8_t  PIN_B = 0;
static uint16_t ENCODER_PPR_EFFECTIVE = 1;

static HybridConfig HC{};

/* ======================= FILTER / HYBRID STATE ======================= */
static float f_raw_period_hz = 0.0f;
static float f_raw_accum_hz  = 0.0f;
static float f_hybrid_hz     = 0.0f;
static float f_iir_hz        = 0.0f;
static float f_out_hz        = 0.0f;

static float rpm_dummy = 0.0f;

/* ======================= Helpers ======================= */
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline float alpha_adaptive(float dt_s) {
  float a = 1.0f - expf(-dt_s / HC.tau_iir_s);
  return clampf(a, HC.alpha_min, HC.alpha_max);
}

static inline float smoothstep(float x) {
  x = clampf(x, 0.0f, 1.0f);
  return x * x * (3.0f - 2.0f * x);
}

static inline float blend_weight(float f_pred_hz) {
  if (f_pred_hz <= HC.fLow_hz)  return 0.0f; // favor period at low speed
  if (f_pred_hz >= HC.fHigh_hz) return 1.0f; // favor accumulate at high speed
  float t = (f_pred_hz - HC.fLow_hz) / (HC.fHigh_hz - HC.fLow_hz);
  return smoothstep(t);
}

static inline bool soft_dt_ok(uint32_t dt_us, float f_pred_hz) {
  if (f_pred_hz <= 1e-3f) return true;
  float dt_pred_us = (1.0f / f_pred_hz) * 1e6f;
  float lo = HC.soft_dt_min_ratio * dt_pred_us;
  float hi = HC.soft_dt_max_ratio * dt_pred_us;
  return ((float)dt_us >= lo) && ((float)dt_us <= hi);
}

static inline float hard_clamp_period_hz(float f_hz) {
  float f_max_from_Tw = 1.0f / HC.Tw_min_s;
  float f_min_from_Tw = 1.0f / HC.Tw_max_s;
  f_hz = clampf(f_hz, f_min_from_Tw, f_max_from_Tw);
  f_hz = clampf(f_hz, 0.0f, HC.f_hard_max_hz);
  return f_hz;
}

static inline float zero_speed_logic(float v_hz, float dt_s, float t_since_pulse_s) {
  float dt_pred_s = (v_hz > 1e-3f) ? (1.0f / v_hz) : 0.0f;
  float Tzero = fmaxf(HC.Tmin_zero_s, HC.k_zero * dt_pred_s);

  if (t_since_pulse_s > Tzero) {
    v_hz = v_hz * expf(-dt_s / HC.tau_zero_s);
    if (v_hz < HC.f_dead_hz && t_since_pulse_s > HC.Tdead_s) {
      v_hz = 0.0f;
    }
  }
  return v_hz;
}

static inline float jerk_limit(float prev, float x, float dt_s) {
  float df_max = HC.a_max_hz_per_s * dt_s;
  float d = clampf(x - prev, -df_max, +df_max);
  return prev + d;
}

/* ======================= API ======================= */
void speed_filter_init() {
  f_raw_period_hz = 0.0f;
  f_raw_accum_hz  = 0.0f;
  f_hybrid_hz     = 0.0f;
  f_iir_hz        = 0.0f;
  f_out_hz        = 0.0f;
  rpm_dummy       = 0.0f;
}

void speed_filter_config(uint8_t mode, uint16_t a_raw, uint16_t b_raw) {
  (void)mode; (void)a_raw; (void)b_raw;
}

void encoder_begin(uint8_t pinA, uint8_t pinB, uint16_t ppr_effective, const HybridConfig& cfg) {
  PIN_A = pinA;
  PIN_B = pinB;
  ENCODER_PPR_EFFECTIVE = (ppr_effective == 0) ? 1 : ppr_effective;
  HC = cfg;

  pinMode(PIN_A, INPUT_PULLUP);
  pinMode(PIN_B, INPUT_PULLUP);

  uint8_t a0 = digitalRead(PIN_A) ? 1 : 0;
  uint8_t b0 = digitalRead(PIN_B) ? 1 : 0;
  enc_prev_state = (a0 << 1) | b0;

  uint32_t now_us = (uint32_t)esp_timer_get_time();
  portENTER_CRITICAL(&mux_enc);
  encoder_pulse_total = 0;
  encoder_pulse_accum = 0;
  encoder_direction   = +1;
  enc_last_edge_us    = now_us;
  enc_period_us       = 0;
  enc_last_pulse_ms   = (uint32_t)(now_us / 1000u);
  portEXIT_CRITICAL(&mux_enc);

  attachInterrupt(digitalPinToInterrupt(PIN_A), isr_encoder_AB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B), isr_encoder_AB, CHANGE);
}

void speed_filter_update(float raw) { rpm_dummy = raw; }

float speed_get_hz_raw_period() { return f_raw_period_hz; }
float speed_get_hz_raw_accum()  { return f_raw_accum_hz;  }
float speed_get_hz_hybrid()     { return f_hybrid_hz;     }
float speed_get_hz_iir()        { return f_iir_hz;        }
float speed_get_hz_out()        { return f_out_hz;        }

int32_t encoder_get_total() {
  int32_t v;
  portENTER_CRITICAL(&mux_enc);
  v = encoder_pulse_total;
  portEXIT_CRITICAL(&mux_enc);
  return v;
}

int8_t encoder_get_dir() {
  int8_t d;
  portENTER_CRITICAL(&mux_enc);
  d = encoder_direction;
  portEXIT_CRITICAL(&mux_enc);
  return d;
}

uint32_t encoder_get_period_us() {
  uint32_t T;
  portENTER_CRITICAL(&mux_enc);
  T = enc_period_us;
  portEXIT_CRITICAL(&mux_enc);
  return T;
}

uint32_t encoder_get_last_pulse_ms() {
  uint32_t t;
  portENTER_CRITICAL(&mux_enc);
  t = enc_last_pulse_ms;
  portEXIT_CRITICAL(&mux_enc);
  return t;
}

int32_t encoder_snapshot_delta_and_reset_accum() {
  int32_t d;
  portENTER_CRITICAL(&mux_enc);
  d = encoder_pulse_accum;
  encoder_pulse_accum = 0;
  portEXIT_CRITICAL(&mux_enc);
  return d;
}

void hybrid_update(float dt_s) {
  // Snapshot ISR values
  uint32_t T_us, last_ms;
  int8_t dir;

  portENTER_CRITICAL(&mux_enc);
  T_us    = enc_period_us;
  last_ms = enc_last_pulse_ms;
  dir     = encoder_direction;
  portEXIT_CRITICAL(&mux_enc);

  // Current time in same ms-domain as enc_last_pulse_ms
  uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000u);

  // ---------- Period path (Hz) ----------
  float fP = 0.0f;
  bool period_valid = false;

  if (T_us > 0) {
    // accept only if pulse is not too old relative to now_ms
    uint32_t age_ms = now_ms - last_ms;
    if (age_ms <= (uint32_t)(HC.Tmin_zero_s * 1000.0f + 200.0f)) {
      fP = 1.0e6f / (float)T_us;
      if (dir < 0) fP = -fP;
      period_valid = true;
    }
  }

  float fP_abs = fabsf(fP);
  fP_abs = hard_clamp_period_hz(fP_abs);

  float f_pred = fabsf(f_out_hz);
  if (period_valid && !soft_dt_ok(T_us, f_pred)) {
    period_valid = false;
    fP_abs = 0.0f;
  }

  f_raw_period_hz = (dir < 0) ? -fP_abs : +fP_abs;

  // ---------- Accumulate path (Hz) ----------
  int32_t dcount = encoder_snapshot_delta_and_reset_accum();
  float fA = (dt_s > 1e-6f) ? ((float)dcount / dt_s) : 0.0f;
  f_raw_accum_hz = fA;

  float fA_abs = fabsf(fA);
  fA_abs = clampf(fA_abs, 0.0f, HC.f_hard_max_hz);
  float fA_signed = (fA < 0) ? -fA_abs : +fA_abs;

  // ---------- Hybrid blend ----------
  float w = blend_weight(f_pred);
  if (!period_valid || fabsf(f_raw_period_hz) < 1e-3f) w = 1.0f;

  f_hybrid_hz = (1.0f - w) * f_raw_period_hz + w * fA_signed;

  // ---------- IIR1 ----------
  float a = alpha_adaptive(dt_s);
  f_iir_hz = f_iir_hz + a * (f_hybrid_hz - f_iir_hz);

  // ---------- Zero-speed decay ----------
  float t_since_pulse_s = (now_ms - last_ms) * 1e-3f;
  f_iir_hz = (f_iir_hz >= 0.0f)
            ? zero_speed_logic(f_iir_hz, dt_s, t_since_pulse_s)
            : -zero_speed_logic(-f_iir_hz, dt_s, t_since_pulse_s);

  // ---------- Jerk limiter ----------
  f_out_hz = jerk_limit(f_out_hz, f_iir_hz, dt_s);
}

/* ======================= ENCODER ISR (quadrature + period) ======================= */
void IRAM_ATTR isr_encoder_AB() {
  uint32_t now_us = (uint32_t)esp_timer_get_time();

  uint8_t a = gpio_get_level((gpio_num_t)PIN_A) ? 1 : 0;
  uint8_t b = gpio_get_level((gpio_num_t)PIN_B) ? 1 : 0;
  uint8_t newState = (a << 1) | b;

  static const int8_t quad_table[4][4] = {
      {  0, +1, -1,  0 },
      { -1,  0,  0, +1 },
      { +1,  0,  0, -1 },
      {  0, -1, +1,  0 }
  };

  uint8_t oldState = enc_prev_state & 0x03;
  int8_t step = quad_table[oldState][newState & 0x03];
  enc_prev_state = newState;

  if (step == 0) return;

  uint32_t dt = now_us - enc_last_edge_us;
  enc_last_edge_us = now_us;

  // reject too-fast glitch
  if (dt <= 50) return;

  portENTER_CRITICAL_ISR(&mux_enc);
  encoder_pulse_total += step;
  encoder_pulse_accum += step;
  encoder_direction   = (step > 0) ? +1 : -1;
  enc_period_us       = dt;
  enc_last_pulse_ms   = (uint32_t)(now_us / 1000u);
  portEXIT_CRITICAL_ISR(&mux_enc);
}

} // namespace Encoder


/* ======================= MOTOR PWM 11-bit ======================= */
namespace MotorPWM11 {

static uint8_t PWM_PIN = 0;
static uint8_t OUT1_PIN = 0;
static uint8_t OUT2_PIN = 0;
static uint8_t CH = 0;

uint16_t clamp_speed_to_duty(uint16_t speed_u16) {
  // User convention: speed_u16 in 0..2048, saturate at 2048
  if (speed_u16 > SPEED_MAX_INPUT) speed_u16 = SPEED_MAX_INPUT;

  // Map to duty 0..2047
  // 0..2047 -> same
  // 2048 -> 2047 (hardware max)
  if (speed_u16 >= PWM_MAX) return PWM_MAX;
  return speed_u16;
}

void begin(uint8_t pwm_pin, uint8_t out1_pin, uint8_t out2_pin,
           uint8_t ledc_channel, uint32_t pwm_freq_hz) {

  PWM_PIN  = pwm_pin;
  OUT1_PIN = out1_pin;
  OUT2_PIN = out2_pin;
  CH       = ledc_channel;

  pinMode(OUT1_PIN, OUTPUT);
  pinMode(OUT2_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);

#if defined(ESP32)
  ledcSetup(CH, pwm_freq_hz, PWM_BITS);
  ledcAttachPin(PWM_PIN, CH);
  ledcWrite(CH, 0);
#endif

  digitalWrite(OUT1_PIN, LOW);
  digitalWrite(OUT2_PIN, LOW);
}

void driveForward(uint16_t speed_u16) {
  uint16_t duty = clamp_speed_to_duty(speed_u16);
  digitalWrite(OUT1_PIN, HIGH);
  digitalWrite(OUT2_PIN, LOW);
#if defined(ESP32)
  ledcWrite(CH, duty);
#endif
}

void driveBackward(uint16_t speed_u16) {
  uint16_t duty = clamp_speed_to_duty(speed_u16);
  digitalWrite(OUT1_PIN, LOW);
  digitalWrite(OUT2_PIN, HIGH);
#if defined(ESP32)
  ledcWrite(CH, duty);
#endif
}

void stopMotor() {
#if defined(ESP32)
  ledcWrite(CH, 0);
#endif
  digitalWrite(OUT1_PIN, LOW);
  digitalWrite(OUT2_PIN, LOW);
}

} // namespace MotorPWM11
