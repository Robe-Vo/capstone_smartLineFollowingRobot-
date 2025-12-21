#include "encoder.hpp"

namespace Encoder
{
    /**
     * ------ Encoder declaration -------
     *
     * This namespace provides:
     *  - Pulse counting (int8) for telemetry to MATLAB (count per dt window)
     *  - Motor speed estimation in Hz using:
     *      (1) Period method (channel B only)
     *      (2) Accumulation method (both channels A+B)
     *  - Hybrid switching with hysteresis (avoid mode chattering)
     *  - Alpha IIR filtering on speed
     *  - Jerk (slew-rate) limiting on speed output
     */

    // Ínit value
    static volatile int8_t  count = 0;             // Pulse count in last dt window (int8, for transmit to MATLAB)
    static volatile int32_t acc_count = 0;         // Accumulated pulse count in dt window (int32, for speed by accumulation)
    static volatile uint32_t timer_period_B = 0;   // Period between B edges (us) - for speed by period

    // Timer
    static volatile uint32_t timer_last_pulse_us = 0;   // Timestamp of last encoder pulse (us) - for stop detect
    static volatile uint32_t timer_last_B_pulse_us = 0; // Timestamp of last B edge (us) - for period measure
    static volatile int8_t   direction_from_B = 1;      // Direction estimate (updated on B edge)

    // Velocity
    static float raw_vec_hz = 0.0f;     // Raw selected speed (Hz) before alpha + jerk
    static float vec_hz = 0.0f;         // Final speed output (Hz) after alpha + jerk (publish this)
    static float iir_vec_hz = 0.0f;     // Alpha-filtered speed state (Hz)

    // Speed mode
    static SpeedMode s_mode = SpeedMode::PERIOD_B;

    // ======= Critical section guard (simple) =======
    // NOTE: Arduino-ESP32 provides portMUX_TYPE for IRAM-safe critical sections.
    // If you already have a global mux elsewhere, reuse it.
    static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

    // ======= Helpers =======

    // Saturate float value to [lo..hi]
    static inline float saturation(float x, float lo, float hi)
    {
        if (x < lo) return lo;
        if (x > hi) return hi;
        return x;
    }

    // Saturate int32 to int8 range [-128..127]
    static inline int8_t sat_i8(int32_t x)
    {
        if (x > 127)  return 127;
        if (x < -128) return -128;
        return (int8_t)x;
    }

    // Limit jerk (slew rate limit) on output speed (Hz)
    static inline float jerk_limit(float prev, float target, float dt_s, float jerk_hz_per_s)
    {
        float max_delta = jerk_hz_per_s * dt_s;
        float delta = target - prev;
        delta = saturation(delta, -max_delta, +max_delta);
        return prev + delta;
    }

    // Alpha filter (IIR one-pole low-pass)
    static inline float iir_alpha(float prev, float meas, float alpha)
    {
        // alpha in (0..1): higher => less smoothing, faster response
        return prev + alpha * (meas - prev);
    }

    // Estimate direction using A/B states sampled on B edge.
    // If direction is reversed in practice, swap +1 and -1 here.
    static inline int8_t update_direction(uint8_t A, uint8_t B)
    {
        // Common mapping for simple quadrature sign estimation
        return (A == B) ? +1 : -1;
    }

    /**
     * ------ API FUNCTION -------
     */

    // Setup function
    void setup()
    {
        // Init timers
        uint32_t now = micros();
        timer_last_pulse_us   = now;
        timer_last_B_pulse_us = now;
        timer_period_B        = 0;
        direction_from_B      = 1;

        // Reset counters
        count = 0;
        acc_count = 0;

        // Reset speed states
        raw_vec_hz = 0.0f;
        vec_hz     = 0.0f;
        iir_vec_hz = 0.0f;
        s_mode     = SpeedMode::PERIOD_B;

        // ISR encoder channel A
        pinMode(ENCODER_CHANNEL_A_PIN, INPUT_PULLDOWN);
        attachInterrupt(digitalPinToInterrupt(ENCODER_CHANNEL_A_PIN), ISR_encoder_channel_A, CHANGE);

        // ISR encoder channel B
        pinMode(ENCODER_CHANNEL_B_PIN, INPUT_PULLDOWN);
        attachInterrupt(digitalPinToInterrupt(ENCODER_CHANNEL_B_PIN), ISR_encoder_channel_B, CHANGE);
    }

    // ISR encoder channel A
    void IRAM_ATTR ISR_encoder_channel_A()
    {
        // Accumulate pulse for accumulation speed method (A+B)
        // NOTE: direction is estimated on B edges only (simple method).
        portENTER_CRITICAL_ISR(&s_mux);
        acc_count++;
        timer_last_pulse_us = micros();
        portEXIT_CRITICAL_ISR(&s_mux);
    }

    // ISR encoder channel B
    void IRAM_ATTR ISR_encoder_channel_B()
    {
        // Accumulate pulse for accumulation speed method (A+B)
        // Also measure period on channel B (period method)
        uint32_t now = micros();

        portENTER_CRITICAL_ISR(&s_mux);

        acc_count++;
        timer_last_pulse_us = now;

        uint32_t dt = now - timer_last_B_pulse_us;

        // Reject unrealistically small dt (glitch) and keep a separate STOP timeout for zero-speed.
        if (dt >= ENC_MIN_PERIOD_US) {
            timer_period_B = dt;
            timer_last_B_pulse_us = now;

            // Direction update on B edge
            uint8_t A = (uint8_t)digitalRead(ENCODER_CHANNEL_A_PIN);
            uint8_t B = (uint8_t)digitalRead(ENCODER_CHANNEL_B_PIN);
            direction_from_B = update_direction(A, B);
        }

        portEXIT_CRITICAL_ISR(&s_mux);
    }

    // Get pulse count in last dt window (int8) and reset it
    int8_t get_count()
    {
        // count is only updated in calculate_vec()
        int8_t c = count;
        count = 0;
        return c;
    }

    // Check if motor is still rotating
    // If not, reset count and speed states
    bool isRotating()
    {
        uint32_t now = micros();
        uint32_t last_us;

        portENTER_CRITICAL(&s_mux);
        last_us = timer_last_pulse_us;
        portEXIT_CRITICAL(&s_mux);

        if ((now - last_us) > (uint32_t)TS_ENCODER_STOP_TIMEOUT_US) {
            // No pulses recently => treat as stop, reset internal states
            portENTER_CRITICAL(&s_mux);
            acc_count = 0;
            timer_period_B = 0;
            portEXIT_CRITICAL(&s_mux);

            count = 0;
            raw_vec_hz = 0.0f;
            vec_hz = 0.0f;
            iir_vec_hz = 0.0f;
            s_mode = SpeedMode::PERIOD_B;
            return false;
        }
        return true;
    }

    /**
     * ==== Calculate speed ====
     *
     * This function will:
     * 1. Calculate speed by period and accumulation
     * 2. Select which one is used (with hysteresis to avoid chattering)
     * 3. Apply alpha IIR filtering
     * 4. Remove jerk (slew-rate limit) in speed output
     *
     * Call this periodically every dt_ms (example: 10 ms).
     */
    void calculate_vec(uint16_t dt_ms)
    {
        const float dt_s = (float)dt_ms / 1000.0f;
        if (dt_s <= 0.0f) return;

        // Snapshot ISR-updated variables atomically
        int32_t  acc_dt = 0;
        uint32_t period_us = 0;
        int8_t   dirB = 1;
        uint32_t last_pulse_us = 0;

        portENTER_CRITICAL(&s_mux);
        acc_dt = acc_count;
        acc_count = 0; // reset for next dt window (accumulation method)
        period_us = timer_period_B;
        dirB = direction_from_B;
        last_pulse_us = timer_last_pulse_us;
        portEXIT_CRITICAL(&s_mux);

        // Update int8 count for transmit (pulse count in dt window)
        count = sat_i8(acc_dt);

        // Stop detect for speed (force zero if no pulses)
        uint32_t now = micros();
        if ((now - last_pulse_us) > (uint32_t)TS_ENCODER_STOP_TIMEOUT_US) {
            raw_vec_hz = 0.0f;
            iir_vec_hz = iir_alpha(iir_vec_hz, 0.0f, ENC_ALPHA);
            vec_hz     = jerk_limit(vec_hz, iir_vec_hz, dt_s, ENC_JERK_LIMIT_HZ_PER_S);
            return;
        }

        // ======= Speed by accumulation (A+B edges) =======
        // hz_acc = (pulses/dt) / counts_per_rev
        float hz_acc = 0.0f;
        if (ENC_COUNTS_PER_REV_ACC > 0) {
            hz_acc = ((float)acc_dt / dt_s) / (float)ENC_COUNTS_PER_REV_ACC;
            hz_acc *= (float)dirB; // apply sign estimate
        }

        // ======= Speed by period (B channel only) =======
        // hz_per = (1/period) / edges_per_rev_B
        float hz_per = 0.0f;
        if (period_us > 0 && ENC_EDGES_PER_REV_B > 0) {
            hz_per = (1e6f / (float)period_us) / (float)ENC_EDGES_PER_REV_B;
            hz_per *= (float)dirB; // apply sign estimate
        }

        // ======= Select speed measurement (with hysteresis) =======
        // Use filtered speed magnitude for stable switching.
        float abs_est = fabsf(iir_vec_hz);

        if (s_mode == SpeedMode::PERIOD_B) {
            if (abs_est > ENC_SW_UP_HZ) s_mode = SpeedMode::ACC_AB;
        } else {
            if (abs_est < ENC_SW_DN_HZ) s_mode = SpeedMode::PERIOD_B;
        }

        // Selected raw speed
        float hz_sel = 0.0f;

        if (s_mode == SpeedMode::ACC_AB) {
            hz_sel = hz_acc;
            // If acc_dt is too small (low speed), fall back to period if available
            if (acc_dt == 0 && period_us > 0) hz_sel = hz_per;
        } else {
            hz_sel = hz_per;
            // If no valid period yet, fall back to accumulation
            if (period_us == 0) hz_sel = hz_acc;
        }

        raw_vec_hz = hz_sel;

        // ======= Alpha filtering =======
        iir_vec_hz = iir_alpha(iir_vec_hz, raw_vec_hz, ENC_ALPHA);

        // ======= Remove jerk (slew limit) on output speed =======
        vec_hz = jerk_limit(vec_hz, iir_vec_hz, dt_s, ENC_JERK_LIMIT_HZ_PER_S);

        static uint32_t k = 0;
k++;
if ((k % 10) == 0) {
  Serial.printf("[SPD] raw=%.3f iir=%.3f out=%.3f mode=%d\n",
                (double)raw_vec_hz, (double)iir_vec_hz, (double)vec_hz,
                (s_mode == SpeedMode::ACC_AB) ? 1 : 0);
}
    }

    // Optional getters (if your header has them)
    float get_vec_hz() { return vec_hz; }
    float get_raw_vec_hz() { return raw_vec_hz; }
    float get_iir_vec_hz() { return iir_vec_hz; }
    SpeedMode get_mode() {return s_mode;}
} // namespace Encoder
