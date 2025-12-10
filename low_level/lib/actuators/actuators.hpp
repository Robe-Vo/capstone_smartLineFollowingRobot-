/**
 *                                                  ================= Note =================
 * 
 *      This is the library for line following robot using two differetial drives and ackerman mechanism for driving direction   
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

                size_t ENCODER_RESOLUTION;
                unsigned long WAIT_RESPONSE_TIME;

                // Flag bit
                bool available = false;     // Flag bit to turn off motor
                bool rotate_flag = true;
                int8_t direction = 0;

                // Feeback signal
                bool encoder_calculation_flag = false; // Flag bit for caluclation -> prevent first data deviation
                float responded_speed = 0;
                unsigned long encoder_count = 0;
                unsigned long timer = 0;

                // Change direction
                void change_direction();
                // Update direction
                void update_direction(bool);
            
            public:
                Drive(uint8_t MOTOR_OUT_1_PIN,uint8_t MOTOR_OUT_2_PIN,uint8_t MOTOR_PWM_PIN,
                      uint8_t ENCODER_CHANNEL_1,uint8_t ENCODER_CHANNEL_2,size_t RESOLUTION,
                      unsigned long wait_time);

                // Get 
                int8_t get_directionStatus();
                int16_t get_respondedSpeed();
                bool isRotating(unsigned long tmr);


                // Update action state of motor
                void enable();
                void disable();
                void brake();

                // Motor action
                bool driving(uint8_t,bool);
                
                // Motor respond | Encoder
                void Endcoder_channel_A_ISR();
                void Endcoder_channel_B_ISR();
        };

        // ========= Steering motor =========
        class Steer
        {
            private:
                // Declare output pin 
                uint8_t STEERING_PIN;
                
                // Angle range
                uint16_t offset_angle = 0;   
                int low_limit = 60;
                int high_limit = 120;

                Servo servo;

                int saturation(int);
            public:
                Steer(uint8_t STEERING_PIN);

                // Set angle
                void turn(int pulse);    // Send pulse signal (angle = 0 || angle > 500)
                // Disable servo
                void disable();
                // Enable servo
                void enable();
        };
};