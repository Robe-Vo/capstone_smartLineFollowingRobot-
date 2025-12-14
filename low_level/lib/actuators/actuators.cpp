#include "actuators.hpp"
#include <math.h>

namespace Encoder {

/* ======================= ENCODER SHARED STATE ======================= */
portMUX_TYPE mux_enc = portMUX_INITIALIZER_UNLOCKED;

// Total pulses (signed) (unfiltered position source)
static volatile int32_t  encoder_pulse_total = 0;

// Accumulate pulses since last snapshot (for accumulate measurement)
static volatile int32_t  encoder_pulse_accum = 0;

// Quadrature state
static volatile uint8_t  enc_prev_state = 0;
static volatile int8_t   encoder_direction = +1;

// Period measurement (us) + last valid pulse time (ms)
static volatile uint32_t enc_last_edge_us  = 0;
static volatile uint32_t enc_period_us     = 0;
static volatile uint32_t enc_last_pulse_ms = 0;

// Pins
static uint8_t PIN_A = 0;
static uint8_t PIN_B = 0;

// Effective resolution (user requested 1 => Hz equals pulse frequency)
static uint16_t ENCODER_PPR_EFFECTIVE = 1;

// Config
static HybridConfig HC{};

/* ======================= FILTER / HYBRID STATE ======================= */
static float f_raw_period_hz = 0.0f;
static float f_raw_accum_hz  = 0.0f;
static float f_hybrid_hz     = 0.0f;
static float f_iir_hz        = 0.0f;
static float f_out_hz        = 0.0f;

/* compat stub */
static float rpm_dummy = 0.0f;

// Helpers
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
  if (f_pred_hz <= 1e-3f) return true; // no prediction => don't reject
  float dt_pred_us = (1.0f / f_pred_hz) * 1e6f;
  float lo = HC.soft_dt_min_ratio * dt_pred_us;
  float hi = HC.soft_dt_max_ratio * dt_pred_us;
  return ((float)dt_us >= lo) && ((float)dt_us <= hi);
}

static inline float hard_clamp_period_hz(float f_hz) {
  // HARD clamp by Tw window (physical)
  float f_max_from_Tw = 1.0f / HC.Tw_min_s;
  float f_min_from_Tw = 1.0f / HC.Tw_max_s;
  f_hz = clampf(f_hz, f_min_from_Tw, f_max_from_Tw);

  // Additional physical max clamp (motor limit)
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
  // keep old interface; this module now works in Hz domain
  f_raw_period_hz = 0.0f;
  f_raw_accum_hz  = 0.0f;
  f_hybrid_hz     = 0.0f;
  f_iir_hz        = 0.0f;
  f_out_hz        = 0.0f;
  rpm_dummy       = 0.0f;
}

void speed_filter_config(uint8_t mode, uint16_t a_raw, uint16_t b_raw) {
  // keep signature for compatibility with old COM config cmd (0xEC)
  // mode/a_raw/b_raw are not used here; left for quick prototype extension.
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
  enc_last_pulse_ms   = 0;
  portEXIT_CRITICAL(&mux_enc);

  attachInterrupt(digitalPinToInterrupt(PIN_A), isr_encoder_AB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B), isr_encoder_AB, CHANGE);
}

void speed_filter_update(float raw) {
  // compatibility stub (old code fed rpm_raw); here keep for quick paste without breaking
  rpm_dummy = raw;
}

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

void hybrid_update(float dt_s, uint32_t now_ms) {
  // Snapshot shared ISR values
  uint32_t T_us, last_ms;
  int8_t dir;

  portENTER_CRITICAL(&mux_enc);
  T_us    = enc_period_us;
  last_ms = enc_last_pulse_ms;
  dir     = encoder_direction;
  portEXIT_CRITICAL(&mux_enc);

  // ---------- Period path (Hz) ----------
  // With ENCODER_PPR_EFFECTIVE=1: f_period_hz = 1e6 / T_us
  float fP = 0.0f;
  bool period_valid = false;

  if (T_us > 0 && (now_ms - last_ms) <= (uint32_t)(HC.Tmin_zero_s * 1000.0f + 200.0f)) {
    fP = 1.0e6f / (float)T_us;
    // apply direction as sign if needed (telemetry often magnitude only)
    if (dir < 0) fP = -fP;
    period_valid = true;
  }

  // HARD clamp on magnitude
  float fP_abs = fabsf(fP);
  fP_abs = hard_clamp_period_hz(fP_abs);

  // SOFT clamp using predicted (previous output magnitude)
  float f_pred = fabsf(f_out_hz);
  if (period_valid && !soft_dt_ok(T_us, f_pred)) {
    // reject period glitch
    period_valid = false;
    fP_abs = 0.0f;
  }

  // restore sign
  f_raw_period_hz = (dir < 0) ? -fP_abs : +fP_abs;

  // ---------- Accumulate path (Hz) ----------
  // accumulate is delta pulses per dt
  int32_t dcount = encoder_snapshot_delta_and_reset_accum();
  float fA = (dt_s > 1e-6f) ? ((float)dcount / dt_s) : 0.0f;
  f_raw_accum_hz = fA;

  // clamp accumulate magnitude by physical max
  float fA_abs = fabsf(fA);
  fA_abs = clampf(fA_abs, 0.0f, HC.f_hard_max_hz);
  float fA_signed = (fA < 0) ? -fA_abs : +fA_abs;

  // ---------- Hybrid blend ----------
  float w = blend_weight(f_pred);

  // If period invalid => force accumulate
  if (!period_valid || fabsf(f_raw_period_hz) < 1e-3f) w = 1.0f;

  f_hybrid_hz = (1.0f - w) * f_raw_period_hz + w * fA_signed;

  // ---------- IIR1 (adaptive alpha) ----------
  float a = alpha_adaptive(dt_s);
  f_iir_hz = f_iir_hz + a * (f_hybrid_hz - f_iir_hz);

  // ---------- Zero-speed decay (period-friendly) ----------
  // t_since_pulse uses last_ms
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
  // enc_last_pulse_ms must be written from a millisecond tick in main (pass-in),
  // but to keep old structure: store "coarse" ms using now_us/1000 here.
  enc_last_pulse_ms   = (uint32_t)(now_us / 1000u);
  portEXIT_CRITICAL_ISR(&mux_enc);
}

} // namespace Encoder


/* ======================= MOTOR PWM 16-bit ======================= */
namespace MotorPWM16 {

static uint8_t PWM_PIN = 0;
static uint8_t OUT1_PIN = 0;
static uint8_t OUT2_PIN = 0;
static uint8_t CH = 0;

void begin(uint8_t pwm_pin, uint8_t out1_pin, uint8_t out2_pin,
           uint8_t ledc_channel, uint32_t pwm_freq_hz, uint8_t pwm_res_bits) {

  PWM_PIN  = pwm_pin;
  OUT1_PIN = out1_pin;
  OUT2_PIN = out2_pin;
  CH       = ledc_channel;

  pinMode(OUT1_PIN, OUTPUT);
  pinMode(OUT2_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);

#if defined(ESP32)
  ledcSetup(CH, pwm_freq_hz, pwm_res_bits); // 16-bit
  ledcAttachPin(PWM_PIN, CH);
  ledcWrite(CH, 0);
#endif

  digitalWrite(OUT1_PIN, LOW);
  digitalWrite(OUT2_PIN, LOW);
}

void driveForward(uint16_t pwm) {
  digitalWrite(OUT1_PIN, HIGH);
  digitalWrite(OUT2_PIN, LOW);
#if defined(ESP32)
  ledcWrite(CH, pwm);
#endif
}

void driveBackward(uint16_t pwm) {
  digitalWrite(OUT1_PIN, LOW);
  digitalWrite(OUT2_PIN, HIGH);
#if defined(ESP32)
  ledcWrite(CH, pwm);
#endif
}

void stopMotor() {
#if defined(ESP32)
  ledcWrite(CH, 0);
#endif
  digitalWrite(OUT1_PIN, LOW);
  digitalWrite(OUT2_PIN, LOW);
}

} // namespace MotorPWM16
