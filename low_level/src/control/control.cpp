// ======================= control.cpp =======================
// Mission of this library:
// 1) Compute speed PID for DC motor (no raw sensor processing here).
// 2) Apply actuator outputs: DC motor PWM (11-bit) + steering servo angle.

#include "control.hpp"
#include <math.h>

// Actuator drivers (mission: hardware actuation lives in actuators lib, not here)
#include "actuators.hpp"

// Servo (optional convenience inside this library)
#include <ESP32Servo.h>
#include "cfg.hpp"
#include "pin.hpp"

namespace Control {

static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline uint16_t clampu16(uint16_t x, uint16_t lo, uint16_t hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

uint16_t clampSpeedCmdToDuty11(uint16_t speed_cmd_u16) {
    // Mission: enforce PC command range 0..2047 and map into duty range 0..2047.
    if (speed_cmd_u16 > 2048) speed_cmd_u16 = 2047;
    return speed_cmd_u16;
}

void PID_setup(SpeedCtrlState& st,
               float kp, float ki, float kd,
               float windup_limit) {
    // Mission: initialize PID gains and reset internal states (integrator, derivative memory).
    st.kp = kp;
    st.ki = ki;
    st.kd = kd;

    st.windup_limit = (windup_limit < 0.0f) ? -windup_limit : windup_limit;

    st.integ  = 0.0f;
    st.e_prev = 0.0f;
    st.d_filt = 0.0f;

    st.duty_last = 0;
}

void PID_setOutputLimits(SpeedCtrlState& st, uint16_t duty_min, uint16_t duty_max) {
    // Mission: limit motor duty output to safe 11-bit bounds.
    if (duty_max < duty_min) {
        uint16_t t = duty_min; duty_min = duty_max; duty_max = t;
    }
    st.duty_min = duty_min;
    st.duty_max = duty_max;
    if (st.duty_max > 2047) st.duty_max = 2047;
}

void PID_setDerivativeFilter(SpeedCtrlState& st, float tau_d_s) {
    // Mission: reduce D-term noise amplification using 1st-order LPF on derivative.
    st.tau_d_s = (tau_d_s < 0.0f) ? 0.0f : tau_d_s;
}

void PID_setDutySlew(SpeedCtrlState& st, float duty_slew_per_s) {
    // Mission: limit duty change rate to reduce current spikes and brownout risk.
    st.duty_slew_per_s = (duty_slew_per_s < 0.0f) ? 0.0f : duty_slew_per_s;
}

static inline float derivFilterStep(float d_in, float d_prev, float dt_s, float tau_s) {
    // Mission: derivative low-pass filter step (internal helper).
    if (tau_s <= 1e-6f) return d_in;
    float a = dt_s / (tau_s + dt_s);
    return d_prev + a * (d_in - d_prev);
}

static inline uint16_t slewU16(uint16_t prev, uint16_t target, float dt_s, float slew_per_s) {
    // Mission: slew limiting for duty output (internal helper).
    if (slew_per_s <= 1e-6f || dt_s <= 1e-6f) return target;

    float max_step = slew_per_s * dt_s;
    float d = (float)target - (float)prev;

    if (d >  max_step) return (uint16_t)lroundf((float)prev + max_step);
    if (d < -max_step) return (uint16_t)lroundf((float)prev - max_step);
    return target;
}

void PID_update(SpeedCtrlState& st,
                float speed_ref,
                float speed_fdb,
                float dt_s,
                uint16_t& duty_out,
                ValueBuffer* dbg) {
    // Mission: compute motor duty command from speed reference and feedback using PID.
    // Note: This function assumes speed_fdb is already computed (no raw encoder processing here).

    if (dt_s <= 1e-6f) dt_s = 1e-3f;

    float e = speed_ref - speed_fdb;

    // Integral term with anti-windup clamp
    st.integ += e * dt_s;
    st.integ = clampf(st.integ, -st.windup_limit, +st.windup_limit);

    // Derivative on error (optionally filtered)
    float d_raw = (e - st.e_prev) / dt_s;
    st.d_filt = derivFilterStep(d_raw, st.d_filt, dt_s, st.tau_d_s);

    float p = st.kp * e;
    float i = st.ki * st.integ;
    float d = st.kd * st.d_filt;

    float u = p + i + d;

    // Convert to integer duty and clamp to limits
    int32_t duty_i = (int32_t)lroundf(u);
    if (duty_i < (int32_t)st.duty_min) duty_i = (int32_t)st.duty_min;
    if (duty_i > (int32_t)st.duty_max) duty_i = (int32_t)st.duty_max;

    uint16_t duty_cmd = (uint16_t)duty_i;

    // Optional slew limit
    duty_cmd = slewU16(st.duty_last, duty_cmd, dt_s, st.duty_slew_per_s);

    st.duty_last = duty_cmd;
    st.e_prev = e;

    duty_out = duty_cmd;

    if (dbg) {
        dbg->speed_ref = speed_ref;
        dbg->speed_fdb = speed_fdb;
        dbg->err = e;
        dbg->p_term = p;
        dbg->i_term = i;
        dbg->d_term = d;
        dbg->u_raw = u;
        dbg->duty_out = duty_cmd;
    }
}

void controlActuators(uint16_t speed_cmd_u16, uint16_t steer_angle_deg) {
    // Mission: apply motor + servo commands (no PID computation).
    // Motor duty: clamp command range then send to PWM driver.
    uint16_t duty11 = clampSpeedCmdToDuty11(speed_cmd_u16);

    if (duty11 == 0) {
        MotorPWM11::stopMotor();
    } else {
        MotorPWM11::driveForward(duty11);
    }

    // Servo: clamp and write (optional convenience)
    static Servo s;
    static bool inited = false;
    if (!inited) {
        s.attach((int)SERVO_PIN);
        inited = true;
    }

    steer_angle_deg = clampu16(steer_angle_deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
    s.write((int)steer_angle_deg);
}

} // namespace Control
