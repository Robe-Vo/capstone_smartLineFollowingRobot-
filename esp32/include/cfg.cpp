// cfg.cpp (you must add this file once in your project)
#include "cfg.hpp"

//  ======================= BLUETOOTH =======================
const char* DEVICE_NAME = "Behind the scream122";

// ======================= TIMING (milliseconds) =======================
uint32_t TS_CONTROLLER_MS    = 10;   // Speed control loop
uint32_t TS_COMMUNICATION_MS = 50;   // RX / TX poll
uint32_t TS_KICK_ULTRA       = 60;   // Default ultrasonic trigger period
uint32_t TS_RESET_TMR        = 100;  // Tick wrap

// alias nếu chỗ khác dùng hậu tố _MS
uint32_t TS_KICK_ULTRA_MS = 60;

// ======================= DC MOTOR / ENCODER / SERVO =======================
uint32_t PWM_FREQ_HZ  = 20000;  // 20 kHz
uint8_t  PWM_CH_MOTOR = 0;

// Encoder disc: 11 pulses/rev/channel → x4
uint16_t ENC_RESOLUTION    = 11;
uint16_t ENC_EFFECTIVE_PPR = (uint16_t)(11 * 4u);

// Servo limits (default)
uint16_t SERVO_MIN_DEG = 55;
uint16_t SERVO_MAX_DEG = 105;
uint16_t SERVO_MID_DEG = 75;

// ======================= SAFETY =======================
uint32_t OP_TIMEOUT_MS = 200;

// ======================= PID: SPEED LOOP (DEFAULTS) =======================
// Dùng khi chưa nhận setupPID
float PID_KP_DEFAULT = 1.0f;
float PID_KI_DEFAULT = 0.0f;
float PID_KD_DEFAULT = 0.0f;

// |integral| ≤ WINDUP
float PID_WINDUP_LIMIT_DEFAULT = 0.0f;

// Duty clamp 11-bit
uint16_t PID_DUTY_MIN_DEFAULT = 0;
uint16_t PID_DUTY_MAX_DEFAULT = 2047;

// D-term filter (s), 0 = off
float PID_TAU_D_DEFAULT = 0.0f;

// Slew limit (counts/s), 0 = off
float PID_DUTY_SLEW_PER_S_DEFAULT = 0.0f;

// Deadband duty cho PID (không áp dụng cho lệnh PWM debug)
uint16_t MOTOR_DEADBAND_DUTY_DEFAULT = 500;


/**
 * ======================= ENCODER (DEFAULTS) =======================
 * */ 

// 1) Period glitch reject (us). Ignore unrealistically small periods (interrupt jitter/glitch).
//    Start around 200~500 us depending on max speed.
uint32_t ENC_MIN_PERIOD_US = 300;

// 2) Stop/zero-speed timeout (us). If no pulses for this time -> speed = 0.
//    Typical: 100ms~300ms. Use longer if you run very slow.
uint32_t TS_ENCODER_STOP_TIMEOUT_US = 150000;

// 3) Counts per revolution for accumulation method (A+B edges).
uint32_t ENC_COUNTS_PER_REV_ACC = (uint32_t)ENC_EFFECTIVE_PPR * 4u;

// 4) Edges per revolution for period method on B only.
uint32_t ENC_EDGES_PER_REV_B = (uint32_t)ENC_EFFECTIVE_PPR * 2u;

// 5) Alpha filter coefficient (0..1). Higher = faster response, less smoothing.
float ENC_ALPHA = 0.25f;

// 6) Jerk (slew-rate) limit for speed output [Hz/s].
float ENC_JERK_LIMIT_HZ_PER_S = 200.0f;

// 7) Hybrid switching thresholds with hysteresis (Hz).
float ENC_SW_UP_HZ = 25.0f;
float ENC_SW_DN_HZ = 18.0f;
