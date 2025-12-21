#include "actuators.hpp"

/*
 * === Motor actuator interface ===
 *
 * This namespace provide functions to setup and run DC motor.
 * The PWM resolution is 11-bit (0-2047).   
 */
namespace Drive
{

  // Duty buffer - use for update PWM
  static uint16_t currentDuty = 0; 
  static int8_t currentDirection = 0;

  // Setup motor pins
  void setup()
  {
    // Setup output pin
    pinMode(MOTOR_OUT_1_PIN,OUTPUT);
    pinMode(MOTOR_OUT_2_PIN,OUTPUT);
    
    // Configurate PWM function
    // Default: using PWM channel 0, at frequency 20000 - 11 bits resolution
    ledcSetup(PWM_CH_MOTOR,PWM_FREQ_HZ,11);
    ledcAttachPin(MOTOR_PWM_PIN,PWM_CH_MOTOR);
  }

  // Set motor PWM duty (0-2047) and direction
  void setPWM(uint16_t duty, bool direction)
  {
    // Saturate duty signal
    if (duty > 2047)  duty = 2047;

    // Check if nessecary to update direction
    if (currentDirection != direction)
    {
      // Update direction
      currentDirection = direction;
      digitalWrite(MOTOR_OUT_1_PIN, currentDirection);
      digitalWrite(MOTOR_OUT_2_PIN, !direction);
    }

    // Check if nessecary to update duty
    if (currentDuty != duty)
    {      
      // Update duty
      currentDuty = duty;
      ledcWrite(PWM_CH_MOTOR,currentDuty);
    }
    else return;
  }

  // Stop motor
  void brake()
  {
    // Set PWM duty = 0
    ledcWrite(PWM_CH_MOTOR,0);
    digitalWrite(MOTOR_OUT_1_PIN,HIGH);
    digitalWrite(MOTOR_OUT_2_PIN,HIGH);
    delay(500);
    digitalWrite(MOTOR_OUT_1_PIN,LOW);
    digitalWrite(MOTOR_OUT_2_PIN,LOW);
  }

}

/*
 * === Servo actuator interface ===
 *
 * This namespace provide functions to setup and run Servo motor.
 */
namespace Steer
{
  // Servo object
  Servo servo;  
  // Angle buffer
  int16_t currentAngle = SERVO_MID_DEG;

  // Setup servo pin
  void setup()
  {
    Steer::enable();
    Steer::writeMidAngle();
  }

  // Enable servo (attach)
  void enable()
  {
    if (!Steer::isEnabled()) servo.attach(SERVO_PIN);
  }

  // Disable servo (detach)
  void disable()
  {
    servo.detach();
  }

  // Check if servo is enabled
  inline bool isEnabled() {return servo.attached();}

  // Write angle to servo (deg)
  void writeAngle(int16_t angle)
  {
    // Saturate input angle
    if (angle > SERVO_MAX_DEG) angle = SERVO_MAX_DEG;
    else if (angle < SERVO_MIN_DEG) angle = SERVO_MIN_DEG;

    // Check if nessecary to update duty
    if (currentAngle != angle)
    {
      currentAngle = angle;
      servo.write(currentAngle);
    }
  }

  void writeMidAngle()
  {
    Steer::writeAngle(SERVO_MID_DEG);
  }
}
