/**
 *  This is the library for line following robot using two differential drives
 *  and Ackermann mechanism for driving direction.
 */

#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>

class Actuators
{
public:
    // ========= Driving motor =========
    class Drive
    {
    private:
        // Pin declaration
        uint8_t MOTOR_OUT_1_PIN;
        uint8_t MOTOR_OUT_2_PIN;
        uint8_t MOTOR_PWM_PIN;
        uint8_t ENCODER_CHANNEL_A;
        uint8_t ENCODER_CHANNEL_B;

        size_t        ENCODER_RESOLUTION;
        unsigned long WAIT_RESPONSE_TIME;

        // Flags
        bool   available  = false;  // drive enable flag
        bool   rotate_flag = false; // set true when encoder tick arrives
        int8_t direction  = 0;      // 0: idle, 1: fwd, 2: rev

        // Feedback
        bool          encoder_calculation_flag = false; // true after first pulse
        float         responded_speed = 0.0f;
        unsigned long encoder_count   = 0;
        unsigned long timer           = 0;  // last encoder pulse time [ms]

        // Change direction according to "direction" state
        void change_direction();

        // Update direction based on input boolean (true = forward, false = backward)
        void update_direction(bool dir_fwd);

    public:
        Drive(uint8_t MOTOR_OUT_1_PIN,
              uint8_t MOTOR_OUT_2_PIN,
              uint8_t MOTOR_PWM_PIN,
              uint8_t ENCODER_CHANNEL_1,
              uint8_t ENCODER_CHANNEL_2,
              size_t  RESOLUTION,
              unsigned long wait_time);

        // Get
        int8_t  get_directionStatus();
        int16_t get_respondedSpeed();
        bool    isRotating(unsigned long tmr);

        // Update action state of motor
        void enable();
        void disable();
        void brake();

        // Motor action
        bool driving(uint8_t speed, bool direction_fwd);

        // Motor respond | Encoder
        void Endcoder_channel_A_ISR();
        void Endcoder_channel_B_ISR();
    };

    // ========= Steering motor =========
    class Steer
    {
    private:
        // Output pin
        uint8_t STEERING_PIN;

        // Angle range [deg]
        uint16_t offset_angle = 0;
        int      low_limit    = 55;   // min steering angle [deg]
        int      high_limit   = 110;  // max steering angle [deg]

        Servo servo;

        int saturation(int angle_deg);

    public:
        Steer(uint8_t STEERING_PIN);

        // Set angle in degrees (0..180)
        void turn(int angle_deg);

        // Disable servo
        void disable();

        // Enable servo
        void enable();
    };
};
