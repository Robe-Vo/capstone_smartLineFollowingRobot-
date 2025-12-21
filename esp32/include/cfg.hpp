#pragma once
#include <stdint.h>

//  ======================= BLUETOOTH =======================
// NOTE: keep as const pointer (can still be changed by assigning another string literal if needed)
extern const char* DEVICE_NAME;

// ======================= TIMING (milliseconds) =======================
// NOTE: runtime-tunable if you want; otherwise leave unchanged in cfg.cpp
extern uint32_t TS_CONTROLLER_MS;     // Speed control loop
extern uint32_t TS_COMMUNICATION_MS;  // RX / TX poll
extern uint32_t TS_KICK_ULTRA;        // Default ultrasonic trigger period
extern uint32_t TS_RESET_TMR;         // Tick wrap

// alias nếu chỗ khác dùng hậu tố _MS
// NOTE: no longer constexpr; keep a variable alias to preserve old structure
extern uint32_t TS_KICK_ULTRA_MS;

// ======================= DC MOTOR / ENCODER / SERVO =======================
extern uint32_t PWM_FREQ_HZ;   // 20 kHz
extern uint8_t  PWM_CH_MOTOR;

// Encoder disc: 11 pulses/rev/channel → x4
extern uint16_t ENC_RESOLUTION;
extern uint16_t ENC_EFFECTIVE_PPR;

// Servo limits (default)
extern uint16_t SERVO_MIN_DEG;
extern uint16_t SERVO_MAX_DEG;
extern uint16_t SERVO_MID_DEG;

// ======================= SAFETY =======================
extern uint32_t OP_TIMEOUT_MS;

// ======================= PID: SPEED LOOP (DEFAULTS) =======================
// Dùng khi chưa nhận setupPID
extern float PID_KP_DEFAULT;
extern float PID_KI_DEFAULT;
extern float PID_KD_DEFAULT;

// |integral| ≤ WINDUP
extern float PID_WINDUP_LIMIT_DEFAULT;

// Duty clamp 11-bit
extern uint16_t PID_DUTY_MIN_DEFAULT;
extern uint16_t PID_DUTY_MAX_DEFAULT;

// D-term filter (s), 0 = off
extern float PID_TAU_D_DEFAULT;

// Slew limit (counts/s), 0 = off
extern float PID_DUTY_SLEW_PER_S_DEFAULT;

// Deadband duty cho PID (không áp dụng cho lệnh PWM debug)
extern uint16_t MOTOR_DEADBAND_DUTY_DEFAULT;


/**
 * ======================= ENCODER (DEFAULTS) =======================
 * */ 

// 1) Period glitch reject (us). Ignore unrealistically small periods (interrupt jitter/glitch).
//    Start around 200~500 us depending on max speed.
extern uint32_t ENC_MIN_PERIOD_US;

// 2) Stop/zero-speed timeout (us). If no pulses for this time -> speed = 0.
//    Typical: 100ms~300ms. Use longer if you run very slow.
extern uint32_t TS_ENCODER_STOP_TIMEOUT_US;

// 3) Counts per revolution for accumulation method (A+B edges).
//    Example: disc=11 pulses per channel per rev.
//      - If you increment on CHANGE for both A and B (and you count every edge): approx 4x => 44.
//    You MUST set correctly for correct Hz scale.
extern uint32_t ENC_COUNTS_PER_REV_ACC;

// 4) Edges per revolution for period method on B only.
//    If you measure CHANGE on B only: 2 edges per B pulse => 22.
//    If you measure only RISING: 11.
extern uint32_t ENC_EDGES_PER_REV_B;

// 5) Alpha filter coefficient (0..1). Higher = faster response, less smoothing.
extern float ENC_ALPHA;

// 6) Jerk (slew-rate) limit for speed output [Hz/s].
extern float ENC_JERK_LIMIT_HZ_PER_S;

// 7) Hybrid switching thresholds with hysteresis (Hz).
//    Switch to ACC at high speed, back to PERIOD at low speed.
extern float ENC_SW_UP_HZ;
extern float ENC_SW_DN_HZ;
