#pragma once
#include <Arduino.h>
#include <stdint.h>

namespace Actuators {

/* ======================= PWM (LEDC 11-bit) ======================= */
static constexpr uint8_t  PWM_RES_BITS   = 11;
static constexpr uint16_t PWM_MAX_11BIT  = 2047;

/* ======================= Pins (EDIT to match your wiring) ======================= */
struct Pins {
  int motor_in1;
  int motor_in2;
  int motor_pwm;
  int servo_pin;
};

/* ======================= Motor config ======================= */
struct MotorCfg {
  uint8_t  ledc_channel;   // e.g. 0
  uint32_t pwm_freq_hz;     // e.g. 20000
};

/* ======================= Servo config ======================= */
struct ServoCfg {
  uint16_t min_deg;     // e.g. 55
  uint16_t max_deg;     // e.g. 110
  uint16_t center_deg;  // e.g. 86
};

/* ======================= Public API ======================= */
void begin(const Pins& pins, const MotorCfg& motor, const ServoCfg& servo);

void driveForward_pwm11(uint16_t pwm11);
void driveBackward_pwm11(uint16_t pwm11);
void stopMotor();

void steerEnable(bool en);
bool steerIsEnabled();
void steerToDeg(uint16_t deg);
uint16_t steerClamp(uint16_t deg);

uint16_t clampPwm11(uint16_t v);

} // namespace Actuators
