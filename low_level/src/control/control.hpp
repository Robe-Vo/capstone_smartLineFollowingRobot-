#pragma once
#include <stdint.h>

namespace Control {

struct SpeedCtrlState {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    float windup_limit = 0.0f;

    float integ  = 0.0f;
    float e_prev = 0.0f;

    float tau_d_s = 0.0f;
    float d_filt  = 0.0f;

    uint16_t duty_min = 0;
    uint16_t duty_max = 2047;

    float duty_slew_per_s = 0.0f;

    uint16_t duty_last = 0;

    // Deadband duty chỉ dùng cho PID (không áp dụng cho lệnh PWM debug)
    uint16_t deadband_duty = 0;
};

struct ValueBuffer {
    float    speed_ref = 0.0f;
    float    speed_fdb = 0.0f;
    float    err       = 0.0f;

    float    p_term = 0.0f;
    float    i_term = 0.0f;
    float    d_term = 0.0f;

    float    u_raw  = 0.0f;
    uint16_t duty_out = 0;

    uint16_t angle_cmd = 0;
    uint16_t angle_out = 0;
};

void PID_init(SpeedCtrlState& st,
              float kp, float ki, float kd,
              float windup_limit);

void PID_setOutputLimits(SpeedCtrlState& st, uint16_t duty_min, uint16_t duty_max);
void PID_setDerivativeFilter(SpeedCtrlState& st, float tau_d_s);
void PID_setDutySlew(SpeedCtrlState& st, float duty_slew_per_s);

// load từ cfg.hpp (PID_*_DEFAULT + deadband)
void PID_setupFromCfg(SpeedCtrlState& st);

void PID_update(SpeedCtrlState& st,
                float speed_ref,
                float speed_fdb,
                float dt_s,
                uint16_t& duty_out,
                ValueBuffer* dbg = nullptr);

void controlActuators(uint16_t speed_cmd_u16, uint16_t steer_angle_deg);
uint16_t clampSpeedCmdToDuty11(uint16_t speed_cmd_u16);

} // namespace Control
