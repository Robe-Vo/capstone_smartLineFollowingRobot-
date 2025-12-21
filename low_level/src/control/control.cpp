// esp32/src/control/control.cpp
// ======================= control.cpp =======================

#include "control.hpp"
#include <math.h>

#include "actuators.hpp"
#include <ESP32Servo.h>

#include "cfg.hpp"
#include "pin.hpp"

namespace Control {

// ---------- helpers ----------
static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline uint16_t clampu16(uint16_t x, uint16_t lo, uint16_t hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline float derivFilterStep(float d_in, float d_prev, float dt_s, float tau_s)
{
    if (tau_s <= 1e-6f) return d_in;
    float a = dt_s / (tau_s + dt_s);
    return d_prev + a * (d_in - d_prev);
}

static inline uint16_t slewU16(uint16_t prev, uint16_t target, float dt_s, float slew_per_s)
{
    if (slew_per_s <= 1e-6f || dt_s <= 1e-6f) return target;

    float max_step = slew_per_s * dt_s;
    float d = (float)target - (float)prev;

    if (d >  max_step) return (uint16_t)lroundf((float)prev + max_step);
    if (d < -max_step) return (uint16_t)lroundf((float)prev - max_step);
    return target;
}

// ======================= PID CONFIG =======================

void PID_init(SpeedCtrlState& st,
              float kp, float ki, float kd,
              float windup_limit)
{
    st.kp = kp;
    st.ki = ki;
    st.kd = kd;

    st.windup_limit = (windup_limit < 0.0f) ? -windup_limit : windup_limit;

    st.integ  = 0.0f;
    st.e_prev = 0.0f;
    st.d_filt = 0.0f;

    st.duty_last = 0;
}

void PID_setOutputLimits(SpeedCtrlState& st, uint16_t duty_min, uint16_t duty_max)
{
    if (duty_max < duty_min) {
        uint16_t t = duty_min;
        duty_min = duty_max;
        duty_max = t;
    }
    st.duty_min = duty_min;
    st.duty_max = duty_max;
    if (st.duty_max > 2047) st.duty_max = 2047;
}

void PID_setDerivativeFilter(SpeedCtrlState& st, float tau_d_s)
{
    st.tau_d_s = (tau_d_s < 0.0f) ? 0.0f : tau_d_s;
}

void PID_setDutySlew(SpeedCtrlState& st, float duty_slew_per_s)
{
    st.duty_slew_per_s = (duty_slew_per_s < 0.0f) ? 0.0f : duty_slew_per_s;
}

void PID_setupFromCfg(SpeedCtrlState& st)
{
    PID_init(st,
             PID_KP_DEFAULT,
             PID_KI_DEFAULT,
             PID_KD_DEFAULT,
             PID_WINDUP_LIMIT_DEFAULT);

    PID_setOutputLimits(st,
                        PID_DUTY_MIN_DEFAULT,
                        PID_DUTY_MAX_DEFAULT);

    PID_setDerivativeFilter(st, PID_TAU_D_DEFAULT);
    PID_setDutySlew(st, PID_DUTY_SLEW_PER_S_DEFAULT);

    // Deadband mặc định
    st.deadband_duty = MOTOR_DEADBAND_DUTY_DEFAULT;
}


// ======================= PID UPDATE =======================

void PID_update(SpeedCtrlState& st,
                float speed_ref,
                float speed_fdb,
                float dt_s,
                uint16_t& duty_out,
                ValueBuffer* dbg)
{
    float e = speed_ref - speed_fdb;

    float p = st.kp * e;

    st.integ += st.ki * e * dt_s;
    if (st.windup_limit > 0.0f) {
        st.integ = clampf(st.integ, -st.windup_limit, st.windup_limit);
    }
    float i = st.integ;

    float de = (e - st.e_prev) / ((dt_s > 1e-6f) ? dt_s : 1e-6f);
    float d_raw = st.kd * de;
    st.d_filt = derivFilterStep(d_raw, st.d_filt, dt_s, st.tau_d_s);
    float d = st.d_filt;

    st.e_prev = e;

    float u = p + i + d;

    int32_t duty_i32 = (int32_t)lroundf(u);
    if (duty_i32 < (int32_t)st.duty_min) duty_i32 = (int32_t)st.duty_min;
    if (duty_i32 > (int32_t)st.duty_max) duty_i32 = (int32_t)st.duty_max;
    uint16_t duty_cmd = (uint16_t)duty_i32;

    // Deadband cho PID: nếu duty nhỏ nhưng khác 0 thì nâng lên ngưỡng deadband
    // (để thắng ma sát tĩnh; không "đập về 0" gây kẹt vòng kín)
    if (st.deadband_duty > 0 && duty_cmd > 0 && duty_cmd < st.deadband_duty) {
        duty_cmd = st.deadband_duty;
    }

    duty_cmd = slewU16(st.duty_last, duty_cmd, dt_s, st.duty_slew_per_s);
    st.duty_last = duty_cmd;

    duty_out = duty_cmd;

    if (dbg) {
        dbg->speed_ref = speed_ref;
        dbg->speed_fdb = speed_fdb;
        dbg->err       = e;
        dbg->p_term    = p;
        dbg->i_term    = i;
        dbg->d_term    = d;
        dbg->u_raw     = u;
        dbg->duty_out  = duty_cmd;
    }
}

// ======================= ACTUATOR APPLY =======================

uint16_t clampSpeedCmdToDuty11(uint16_t speed_cmd_u16)
{
    if (speed_cmd_u16 > 2047) speed_cmd_u16 = 2047;
    return speed_cmd_u16;
}

void controlActuators(uint16_t speed_cmd_u16, uint16_t steer_angle_deg)
{
    // Motor
    uint16_t duty11 = clampSpeedCmdToDuty11(speed_cmd_u16);
    if (duty11 == 0) {
        MotorPWM11::stopMotor();
    } else {
        MotorPWM11::driveForward(duty11);
    }

    // Servo
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
