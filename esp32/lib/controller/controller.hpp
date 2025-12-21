#pragma once

#include <Arduino.h>

#include "actuators.hpp"
#include "../../include/cfg.hpp"


/**
 *            ==== Controller ====
 * 
 * This namespace calculate duty by PID algorithm
 * **/

namespace Controller
{
    // For configuration
    struct Cfg 
    {
        float kp = PID_KP_DEFAULT;
        float ki = PID_KI_DEFAULT;
        float kd = PID_KD_DEFAULT;

        // Derivative filter time constant (seconds). 0 => no filter (raw derivative)
        float d_tau_s = PID_TAU_D_DEFAULT;

        // Integrator clamp in "output units" (duty). 0 => no clamp
        float i_limit = PID_WINDUP_LIMIT_DEFAULT;

        // Output clamp (duty)
        uint16_t out_min = PID_DUTY_MIN_DEFAULT;
        uint16_t out_max = PID_DUTY_MAX_DEFAULT;

        // Deadband duty to overcome static friction (added when output > 0)
        uint16_t deadband = MOTOR_DEADBAND_DUTY_DEFAULT;

        // Slew limit on output duty (duty per second). 0 => disabled
        float out_slew_per_s = PID_DUTY_SLEW_PER_S_DEFAULT;
    };

    // For calculation
    struct State 
    {
        float i = 0.0f;        // integrator state (duty)
        float d_filt = 0.0f;   // filtered derivative state
        float e_prev = 0.0f;   // previous error
        float u_prev = 0.0f;   // previous unclamped output (before duty clamp)
        uint16_t out_prev = 0; // previous final output (after clamps)
        bool  first = true;    // first update flag
        bool is_firstUpload = true;
     };

    // For debug 
    struct Debug {
        float ref;
        float meas;
        float err;

        float p;
        float i;
        float d;

        float u_raw;        // before clamp
        float u;    // after clamp (float)
        uint16_t duty_out;  // final duty after slew + deadband

        bool saturated;     // output hit clamp
    };


    // Init/reset state
    void init(State& s, const Cfg& c);

    // Reset only integrator/derivative (useful when ref=0 or mode switch)
    void reset(State& s);

    // Core PID update: returns duty in [out_min..out_max]
    uint16_t update(State& s,
                    const Cfg& c,
                    float ref,
                    float meas,
                    float dt_s);
    
    // For debug
    uint16_t update(State& s,
                  const Cfg& c,
                  float ref,
                  float meas,
                  float dt_s,
                  Debug* dbg);


}