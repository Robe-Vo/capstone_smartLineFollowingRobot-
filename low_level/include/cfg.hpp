#pragma once
#include <stdint.h>

// ======================= TIMING (milliseconds) =======================
constexpr uint32_t TS_CONTROLLER_MS    = 10;   // Speed control loop
constexpr uint32_t TS_COMMUNICATION_MS = 50;   // RX / TX poll
constexpr uint32_t TS_KICK_ULTRA       = 60;   // Default ultrasonic trigger period
constexpr uint32_t TS_RESET_TMR        = 100;  // Tick wrap

// alias nếu chỗ khác dùng hậu tố _MS
constexpr uint32_t TS_KICK_ULTRA_MS = TS_KICK_ULTRA;

// ======================= DC MOTOR / ENCODER / SERVO =======================
constexpr uint32_t PWM_FREQ_HZ  = 20000;  // 20 kHz
constexpr uint8_t  PWM_CH_MOTOR = 0;

// Encoder disc: 11 pulses/rev/channel → x4
constexpr uint16_t ENC_RESOLUTION    = 11;
constexpr uint16_t ENC_EFFECTIVE_PPR = ENC_RESOLUTION * 4u;

// Servo limits (default)
constexpr uint16_t SERVO_MIN_DEG = 55;
constexpr uint16_t SERVO_MAX_DEG = 105;
constexpr uint16_t SERVO_MID_DEG = 75;

// ======================= SAFETY =======================
constexpr uint32_t OP_TIMEOUT_MS = 200;

// ======================= PID: SPEED LOOP (DEFAULTS) =======================
// Dùng khi chưa nhận setupPID
constexpr float PID_KP_DEFAULT = 1.0f;
constexpr float PID_KI_DEFAULT = 0.0f;
constexpr float PID_KD_DEFAULT = 0.0f;

// |integral| ≤ WINDUP
constexpr float PID_WINDUP_LIMIT_DEFAULT = 0.0f;

// Duty clamp 11-bit
constexpr uint16_t PID_DUTY_MIN_DEFAULT = 0;
constexpr uint16_t PID_DUTY_MAX_DEFAULT = 2047;

// D-term filter (s), 0 = off
constexpr float PID_TAU_D_DEFAULT = 0.0f;

// Slew limit (counts/s), 0 = off
constexpr float PID_DUTY_SLEW_PER_S_DEFAULT = 0.0f;

// Deadband duty cho PID (không áp dụng cho lệnh PWM debug)
constexpr uint16_t MOTOR_DEADBAND_DUTY_DEFAULT = 500;

// ======================= ENCODER HYBRID (DEFAULTS) =======================
// Period window (s)
constexpr float ENC_TW_MIN_S = 0.005f;
constexpr float ENC_TW_MAX_S = 0.2000f;

// Blend thresholds (Hz)
constexpr float ENC_F_LOW_HZ  = 40.0f;
constexpr float ENC_F_HIGH_HZ = 80.0f;

// IIR
constexpr float ENC_TAU_IIR_S = 0.050f;
constexpr float ENC_ALPHA_MIN = 0.05f;
constexpr float ENC_ALPHA_MAX = 0.30f;

// Jerk limit (Hz/s)
constexpr float ENC_A_MAX_HZ_PER_S = 500.0f;

// Zero-speed detection
constexpr float    ENC_K_ZERO           = 3.0f;
constexpr float    ENC_TMIN_ZERO_S      = 0.035f;
constexpr float    ENC_TAU_ZERO_S       = 0.060f;
constexpr float    ENC_F_DEAD_HZ        = 1.0f;
constexpr float    ENC_TDEAD_S          = 0.060f;
constexpr uint32_t ENC_ZERO_TIMEOUT_MS  = 200;

// Physical max speed (Hz)
constexpr float ENC_F_HARD_MAX_HZ = 300.0f;

// Glitch reject window trên dt
constexpr float ENC_SOFT_DT_MIN_RATIO = 0.35f;
constexpr float ENC_SOFT_DT_MAX_RATIO = 2.50f;

// ======================= IDLE PARAM DEFAULTS =======================
// setupLine
constexpr uint8_t LINE_SAMPLE_DEFAULT = 4;        // số mẫu trung bình line sensor

// setupUltra
constexpr uint16_t ULTRA_TRIG_PERIOD_MS_DEFAULT = TS_KICK_ULTRA; // period đá mặc định

// setupServo (servo default đã khai báo ở trên)
// setupMotor
constexpr uint16_t MOTOR_DEADBAND_DUTY_INIT = MOTOR_DEADBAND_DUTY_DEFAULT;
