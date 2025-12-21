// esp32/lib/actuators/actuators.cpp
#include "actuators.hpp"
#include <math.h>

#include "../../include/cfg.hpp"
#include "../../include/pin.hpp"

// ======================= ENCODER (Hybrid: Period + Accumulate) =======================

namespace Encoder {

portMUX_TYPE mux_enc = portMUX_INITIALIZER_UNLOCKED;

// ISR shared state
static uint8_t  PIN_A = 0;
static uint8_t  PIN_B = 0;

static volatile int32_t  encoder_pulse_total = 0;
static volatile int32_t  encoder_pulse_accum = 0;     // accumulate delta for hybrid
static volatile int8_t   encoder_direction   = 0;

static volatile uint32_t enc_last_edge_us    = 0;
static volatile uint32_t enc_period_us       = 0;
static volatile uint32_t enc_last_pulse_ms   = 0;
static volatile uint8_t  enc_prev_state      = 0;

// Effective PPR used by accumulate conversion
static uint16_t ENCODER_PPR_EFFECTIVE = ENC_EFFECTIVE_PPR;

// Runtime hybrid config
static HybridConfig HC;

// Outputs
static float f_raw_period_hz = 0.0f;
static float f_raw_accum_hz  = 0.0f;
static float f_hybrid_hz     = 0.0f;
static float f_iir_hz        = 0.0f;
static float f_out_hz        = 0.0f;

// ---------- helpers ----------
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline float smoothstep(float t) {
  t = clampf(t, 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
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
  f_hz = clampf(f_hz, 0.0f, f_max_from_Tw);
  f_hz = clampf(f_hz, 0.0f, HC.f_hard_max_hz);
  return f_hz;
}

static inline float alpha_adaptive(float dt_s) {
  float a = dt_s / (HC.tau_iir_s + dt_s);
  a = clampf(a, HC.alpha_min, HC.alpha_max);
  return a;
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

// ======================= API =======================

void speed_filter_init() {
  f_raw_period_hz = 0.0f;
  f_raw_accum_hz  = 0.0f;
  f_hybrid_hz     = 0.0f;
  f_iir_hz        = 0.0f;
  f_out_hz        = 0.0f;

  portENTER_CRITICAL(&mux_enc);
  encoder_pulse_total = 0;
  encoder_pulse_accum = 0;
  encoder_direction   = 0;
  enc_last_edge_us    = 0;
  enc_period_us       = 0;
  enc_last_pulse_ms   = (uint32_t)(esp_timer_get_time() / 1000u);
  enc_prev_state      = 0;
  portEXIT_CRITICAL(&mux_enc);
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

  // init prev state
  uint8_t a = gpio_get_level((gpio_num_t)PIN_A) ? 1 : 0;
  uint8_t b = gpio_get_level((gpio_num_t)PIN_B) ? 1 : 0;
  enc_prev_state = (a << 1) | b;

  attachInterrupt(digitalPinToInterrupt(PIN_A), isr_encoder_AB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B), isr_encoder_AB, CHANGE);

  speed_filter_init();
}

void speed_filter_update(float raw) { (void)raw; }

// -------------------- Accessors --------------------
float speed_get_hz_raw_period() { return f_raw_period_hz; }
float speed_get_hz_raw_accum()  { return f_raw_accum_hz; }
float speed_get_hz_hybrid()     { return f_hybrid_hz; }
float speed_get_hz_iir()        { return f_iir_hz; }
float speed_get_hz_out()        { return f_out_hz; }

int32_t encoder_get_total() {
  portENTER_CRITICAL(&mux_enc);
  int32_t v = encoder_pulse_total;
  portEXIT_CRITICAL(&mux_enc);
  return v;
}

int8_t encoder_get_dir() {
  portENTER_CRITICAL(&mux_enc);
  int8_t v = encoder_direction;
  portEXIT_CRITICAL(&mux_enc);
  return v;
}

uint32_t encoder_get_period_us() {
  portENTER_CRITICAL(&mux_enc);
  uint32_t v = enc_period_us;
  portEXIT_CRITICAL(&mux_enc);
  return v;
}

uint32_t encoder_get_last_pulse_ms() {
  portENTER_CRITICAL(&mux_enc);
  uint32_t v = enc_last_pulse_ms;
  portEXIT_CRITICAL(&mux_enc);
  return v;
}

int32_t encoder_snapshot_delta_and_reset_accum() {
  portENTER_CRITICAL(&mux_enc);
  int32_t d = encoder_pulse_accum;
  encoder_pulse_accum = 0;
  portEXIT_CRITICAL(&mux_enc);
  return d;
}

// ======================= HYBRID UPDATE =======================

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
      float fP_edge  = 1.0e6f / (float)T_us;                      // edge/s
      float fP_wheel = fP_edge / (float)(ENC_RESOLUTION * 2.0f);  // wheel Hz
      fP = (dir < 0) ? -fP_wheel : +fP_wheel;
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
  float fA = (dt_s > 1e-6f) ? ((float)dcount / (dt_s * (float)ENCODER_PPR_EFFECTIVE)) : 0.0f;
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

  void IRAM_ATTR isr_encoder_AB()
  {
    // Chỉ đo encoder khi đang OPERATION để tránh nhiễu khi IDLE
    if (!State::isOperation()) {
      return;
    }

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

    // ===== Accumulate: dùng cả A và B (x4) =====
    portENTER_CRITICAL_ISR(&mux_enc);
    encoder_pulse_total += step;
    encoder_pulse_accum += step;
    encoder_direction   = (step > 0) ? +1 : -1;
    portEXIT_CRITICAL_ISR(&mux_enc);

    // ===== Period: chỉ sử dụng cạnh ở channel B =====
    // Bit0 là B, nếu B không đổi -> không cập nhật period
    if (((oldState ^ newState) & 0x01u) == 0u) {
      return;
    }

    uint32_t dt = now_us - enc_last_edge_us;
    enc_last_edge_us = now_us;

    // loại glitch quá nhanh
    if (dt < 50u) {
      return;
    }

    portENTER_CRITICAL_ISR(&mux_enc);
    enc_period_us = dt;
    enc_last_pulse_ms = (uint32_t)(now_us / 1000u);
    portEXIT_CRITICAL_ISR(&mux_enc);
  }

} // namespace Encoder

/* ======================= Motor PWM 11-bit helper ======================= */
namespace MotorPWM11 {

static uint8_t PWM_PIN = 0;
static uint8_t OUT1_PIN = 0;
static uint8_t OUT2_PIN = 0;
static uint8_t CH = 0;

// Deadband duty (0 disables). Initialized from cfg.hpp.
static uint16_t s_deadband = MOTOR_DEADBAND_DUTY_DEFAULT;

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

  if (duty > 0 && duty < s_deadband) duty = s_deadband;
  if (duty > 2047) duty = 2047;

  digitalWrite(OUT1_PIN, HIGH);
  digitalWrite(OUT2_PIN, LOW);
#if defined(ESP32)
  ledcWrite(CH, duty);
#endif
}

void driveBackward(uint16_t speed_u16) {
  uint16_t duty = clamp_speed_to_duty(speed_u16);

  if (duty > 0 && duty < s_deadband) duty = s_deadband;
  if (duty > 2047) duty = 2047;

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

// Deadband setting
void setDeadband(uint16_t db) { s_deadband = db; }
uint16_t getDeadband() { return s_deadband; }

} // namespace MotorPWM11
