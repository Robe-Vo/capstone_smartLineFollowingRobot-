// ======================= control.hpp =======================
// Mission of this library:
// 1) Compute speed PID for DC motor (no raw sensor processing here).
// 2) Apply actuator outputs: DC motor PWM (11-bit) + steering servo angle.

#pragma once
#include <stdint.h>

namespace Control {

    // ----------------------- PID STATE -----------------------
    // Mission: store speed PID parameters and internal states for the DC motor loop.
    struct SpeedCtrlState {
        // PID gains (mission: tuning parameters)
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;

        // Anti-windup limit for integrator (mission: prevent integrator runaway)
        float windup_limit = 0.0f;

        // Internal states (mission: controller memory)
        float integ  = 0.0f;
        float e_prev = 0.0f;

        // Optional derivative filter (mission: reduce noise amplification on D term)
        float tau_d_s = 0.0f;  // 0 => no filter
        float d_filt  = 0.0f;

        // Output clamp (mission: enforce 11-bit motor duty range)
        uint16_t duty_min = 0;
        uint16_t duty_max = 2047;

        // Optional slew limit (mission: limit duty change rate to reduce current spikes)
        float duty_slew_per_s = 0.0f;

        // Last applied duty (mission: enable slew limiting and prevent redundant writes)
        uint16_t duty_last = 0;
    };

    // ----------------------- DEBUG BUFFER -----------------------
    // Mission: optional container for logging PID values without doing Serial prints inside control loop.
    struct ValueBuffer {
        float speed_ref = 0.0f;  // input reference
        float speed_fdb = 0.0f;  // feedback
        float err       = 0.0f;  // error

        float p_term = 0.0f;     // proportional term
        float i_term = 0.0f;     // integral term
        float d_term = 0.0f;     // derivative term

        float u_raw  = 0.0f;     // unclamped controller output (float)
        uint16_t duty_out = 0;   // final duty after clamp/slew

        uint16_t angle_cmd = 0;  // requested angle
        uint16_t angle_out = 0;  // applied angle (after clamp/slew)
    };

    // ======================= PID CONFIG =======================
    // Mission: initialize PID gains + reset PID internal states.
    void PID_setup(SpeedCtrlState& st,
                   float kp, float ki, float kd,
                   float windup_limit);

    // Mission: configure duty output saturation range (11-bit, 0..2047).
    void PID_setOutputLimits(SpeedCtrlState& st, uint16_t duty_min, uint16_t duty_max);

    // Mission: configure derivative filtering time constant (tau_d_s=0 disables).
    void PID_setDerivativeFilter(SpeedCtrlState& st, float tau_d_s);

    // Mission: configure maximum duty change rate (0 disables).
    void PID_setDutySlew(SpeedCtrlState& st, float duty_slew_per_s);

    // ======================= PID UPDATE =======================
    // Mission: compute motor duty command from speed_ref/speed_fdb using PID.
    // Notes:
    // - Does NOT process raw encoder signals; expects speed_fdb already computed (Hz/rpm/mm/s).
    // - duty_out is 0..2047 (11-bit).
    void PID_update(SpeedCtrlState& st,
                    float speed_ref,
                    float speed_fdb,
                    float dt_s,
                    uint16_t& duty_out,
                    ValueBuffer* dbg = nullptr);

    // ======================= ACTUATOR APPLY =======================
    // Mission: apply actuator outputs (DC motor + servo) with clamping only.
    // Notes:
    // - speed_cmd_u16 is expected in PC convention: 0..2048, saturate if higher.
    // - steer_angle_deg is in degrees; library clamps to [SERVO_MIN_DEG..SERVO_MAX_DEG].
    // - Does NOT compute PID; it only applies already-decided commands.
    void controlActuators(uint16_t speed_cmd_u16, uint16_t steer_angle_deg);

    // ======================= UTILITIES =======================
    // Mission: convert command range (0..65535) into motor duty range (0..2047).
    uint16_t clampSpeedCmdToDuty11(uint16_t speed_cmd_u16);
}
