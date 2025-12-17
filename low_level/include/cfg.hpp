#pragma once
#include <stdint.h>

// ======================= TIMING =======================
constexpr uint32_t TS_CONTROLLER_MS = 10;   
constexpr uint32_t TS_TELEM_MS      = 50;   
constexpr uint32_t TIME_KICK_ULTRA  = 60;   

// ======================= DC MOTOR / ENCODER / SERVO =======================
constexpr uint32_t PWM_FREQ_HZ = 20000;
constexpr uint8_t  PWM_CH_MOTOR = 0;

constexpr uint16_t ENC_RESOLUTION   = 11;

constexpr uint16_t SERVO_MIN_DEG = 55;
constexpr uint16_t SERVO_MAX_DEG = 105;
constexpr uint16_t SERVO_MID_DEG = 75;

// ======================= SAFETY =======================
constexpr uint32_t OP_TIMEOUT_MS = 200; 
