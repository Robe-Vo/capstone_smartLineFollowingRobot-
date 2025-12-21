#include "controller.hpp"
// ======================= pid.cpp (FULL PID LIB) =======================

namespace Controller
{
  static inline float saturation(float x, float lo, float hi)
  {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
  }

  static inline uint16_t saturation(int32_t x, uint16_t lo, uint16_t hi)
  {
    if (x < (int32_t)lo) return lo;
    if (x > (int32_t)hi) return hi;
    return (uint16_t)x;
  }

  static inline float iir_1pole(float prev, float x, float alpha)
  {
    return prev + alpha * (x - prev);
  }

  static inline uint16_t slew_limit_u16(uint16_t prev, uint16_t target, float dt_s, float slew_per_s)
  {
    if (slew_per_s <= 0.0f || dt_s <= 0.0f) return target;

    float max_delta = slew_per_s * dt_s;
    float delta = (float)target - (float)prev;

    if (delta >  max_delta) delta =  max_delta;
    if (delta < -max_delta) delta = -max_delta;

    float out = (float)prev + delta;
    if (out < 0.0f) out = 0.0f;
    if (out > 65535.0f) out = 65535.0f;
    return (uint16_t)(out + 0.5f);
  }

  // Init
  void init(State& s, const Cfg& /*c*/)
  {
    reset(s);
  }

  // Reset PID
  void reset(State& s)
  {
    s.i = 0.0f;
    s.d_filt = 0.0f;
    s.e_prev = 0.0f;
    s.u_prev = 0.0f;
    s.out_prev = 0;
    s.first = true;
  }

  uint16_t update(State& s,
                  const Cfg& c,
                  float ref,
                  float meas,
                  float dt_s)
  {
    return update(s, c, ref, meas, dt_s, nullptr);
  }

  uint16_t update(State& s,
                  const Cfg& c,
                  float ref,
                  float meas,
                  float dt_s,
                  Debug* dbg)
  {
    if (dt_s <= 0.0f) {
      if (dbg) { dbg->duty_out = s.out_prev; }
      return s.out_prev;
    }

    // Error (Hz)
    float e = ref - meas;

    // P
    float p = c.kp * e;

    // I (integrate in "duty units")
    // i += ki * e * dt
    s.i += (c.ki * e * dt_s);

    // Integrator clamp (anti-windup by clamping)
    if (c.i_limit > 0.0f) {
      s.i = saturation(s.i, -c.i_limit, +c.i_limit);
    }

    // D (derivative of error)
    float d = 0.0f;
    if (!s.first) {
      float de = (e - s.e_prev) / dt_s;   // Hz/s
      float d_raw = c.kd * de;            // duty

      if (c.d_tau_s > 0.0f) {
        // 1st order LPF on derivative term
        // alpha = dt / (tau + dt)
        float alpha = dt_s / (c.d_tau_s + dt_s);
        s.d_filt = iir_1pole(s.d_filt, d_raw, alpha);
        d = s.d_filt;
      } else {
        d = d_raw;
        s.d_filt = d_raw;
      }
    } else {
      // first call: derivative not reliable
      s.d_filt = 0.0f;
      d = 0.0f;
      s.first = false;
    }

    // Unclamped output (float duty units)
    float u = p + s.i + d;

    // Save prev error
    s.e_prev = e;

    // Convert to duty command (before clamp)
    // For speed loop, negative u usually means "0 duty" if direction handled outside
    if (u < 0.0f) u = 0.0f;

    // Clamp to output range
    uint16_t out = saturation((int32_t)(u + 0.5f), c.out_min, c.out_max);

    // Apply deadband (only if output > 0)
    if (out > 0 && c.deadband > 0) {
      uint32_t tmp = (uint32_t)out + (uint32_t)c.deadband;
      if (tmp > c.out_max) tmp = c.out_max;
      out = (uint16_t)tmp;
    }

    // Slew limit (duty/s)
    if (c.out_slew_per_s > 0.0f) {
      out = slew_limit_u16(s.out_prev, out, dt_s, c.out_slew_per_s);
    }

    s.out_prev = out;
    s.u_prev = u;

    if (dbg) {
      dbg->err = e;
      dbg->p = p;
      dbg->i = s.i;
      dbg->d = d;
      dbg->u = u;
      dbg->duty_out = out;
    }

    return out;
  }

} // namespace PID
