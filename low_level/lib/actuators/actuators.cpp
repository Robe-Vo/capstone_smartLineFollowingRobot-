#include "actuators.hpp"

/** =========================================================================================================
 *                                          Drive function
   =========================================================================================================*/

// ======================= Private function declarations =======================

// Change direction of motor
void Actuators::Drive::change_direction()
{
    switch(this->direction)
    {
        // IDLE
        case 0:
            digitalWrite(MOTOR_OUT_1_PIN,false);
            digitalWrite(MOTOR_OUT_2_PIN,false);
            encoder_calculation_flag = false;
        break;

        // Run forward
        case 1:
            digitalWrite(MOTOR_OUT_1_PIN,true);
            digitalWrite(MOTOR_OUT_2_PIN,false);
        break;
        
        // Run backward
        case 2:
            digitalWrite(MOTOR_OUT_1_PIN,false);
            digitalWrite(MOTOR_OUT_2_PIN,true);
        break;
    }
}

void Actuators::Drive::update_direction(bool direction)
{
    int8_t direction_flag = this->direction;
    if (direction_flag == 0)
        {
            if (direction == true)
            {
                this->direction = 1;
                change_direction();
            }
            else
            {
                this->direction = -1;
                change_direction();
            }
        } 
        else if (direction_flag > 0 and direction == false)
        {   
            this->direction = -1;
            change_direction();
        }
        else if (direction_flag < 0 and direction == true)
        {
            this->direction = 1;
            change_direction();
        }
}


// ======================= Get function declarations =======================

// Get current control direction 
int8_t Actuators::Drive::get_directionStatus()
{
    return this->direction;
}

// Get respond speed
int16_t Actuators::Drive::get_respondedSpeed()
{
    return this->responded_speed;
}

// If the motor is rotation
bool Actuators::Drive::isRotating(unsigned long tmr)
{
    if (!rotate_flag) // The encoder pulse has not been kicked yet 
    {
        if ((tmr-timer) > WAIT_RESPONSE_TIME)
        {
            return false;
        } 
        else
        {
            return true;
        }
    }
    else // Receive signal from encoder
    {
        // Update time
        this->rotate_flag = false;
        return true;
    }
}

// ======================= Public function declarations =======================

// Declare pin function
Actuators::Drive::Drive(uint8_t MOTOR_OUT_1_PIN,uint8_t MOTOR_OUT_2_PIN,uint8_t MOTOR_PWM_PIN,
                        uint8_t ENCODER_CHANNEL_1,uint8_t ENCODER_CHANNEL_2,size_t RESOLUTION,
                        unsigned long wait_time)
{
    this->MOTOR_OUT_1_PIN   = MOTOR_OUT_1_PIN;
    this->MOTOR_OUT_2_PIN   = MOTOR_OUT_2_PIN;
    this->MOTOR_PWM_PIN     = MOTOR_PWM_PIN;
    this->ENCODER_CHANNEL_A = ENCODER_CHANNEL_1;
    this->ENCODER_CHANNEL_B = ENCODER_CHANNEL_2;
    this->ENCODER_RESOLUTION = RESOLUTION;
    this->WAIT_RESPONSE_TIME = wait_time;
    
    pinMode(MOTOR_OUT_1_PIN,OUTPUT);
    pinMode(MOTOR_OUT_2_PIN,OUTPUT);
    pinMode(MOTOR_PWM_PIN,OUTPUT);
    pinMode(ENCODER_CHANNEL_1,INPUT);
    pinMode(ENCODER_CHANNEL_2,INPUT);
}

// Turn drive able to run
void Actuators::Drive::enable()
{
    this->available = true;
}

// Disable drive to run
void Actuators::Drive::disable()
{
    available = false;

    // Disable output pin
    change_direction();
}

// Brake 
void Actuators::Drive::brake()
{
    digitalWrite(MOTOR_OUT_1_PIN,true);
    digitalWrite(MOTOR_OUT_2_PIN,true);
    analogWrite(MOTOR_PWM_PIN,0);
    delay(100);
    change_direction();
}

// Start to control speed
bool Actuators::Drive::driving(uint8_t speed,bool direction)
{
    if (!available)
    {
        return false;
    }    
    else
    {
        analogWrite(MOTOR_PWM_PIN, speed);  
        update_direction(direction);      
        return true;
    }
}

// Interrupt function for calculation of Encoder's channel A
void Actuators::Drive::Endcoder_channel_A_ISR()
{
    if (digitalRead(ENCODER_CHANNEL_A) == digitalRead(ENCODER_CHANNEL_B)) {
        encoder_count++; // Increase encoder pulse accumulate
    } else {
        encoder_count--; // Decrease encoder pulse accumulate
    }

    // Calculate speed
    long tmr = millis(); // Take instance time
    if (encoder_calculation_flag) { // Check wheter this is the first pulse after robot stop 
    long dt = tmr - timer;
    if (dt > 0) {
      responded_speed = (float)ENCODER_RESOLUTION * 1000.0f / (float)dt;
    }
    } else {
        encoder_calculation_flag = true;
    }

    timer = tmr;
}

// Interrupt function for calculation of Encoder's channel B
void Actuators::Drive::Endcoder_channel_B_ISR()
{
    if (digitalRead(ENCODER_CHANNEL_A) != digitalRead(ENCODER_CHANNEL_B)) {
        encoder_count++; // Increase encoder pulse accumulate
    } else {
        encoder_count--; // Decrease encoder pulse accumulate
    }

    // Calculate speed
    long tmr = millis(); // Take instance time
    if (encoder_calculation_flag) // Check wheter this is the first pulse after robot stop
    {  
        long dt = tmr - timer;
        if (dt > 0) {
            responded_speed = (float)ENCODER_RESOLUTION * 1000.0f / (float)dt;
        }
        } else {
            encoder_calculation_flag = true;
        }

    timer = tmr;
}



/** =========================================================================================================
 *                                          Steer function
   =========================================================================================================*/

// =======================  =======================

// Init object
Actuators::Steer::Steer(uint8_t STEERING_PIN)
{
    this->STEERING_PIN = STEERING_PIN;
    servo.attach(STEERING_PIN);
}

int Actuators::Steer::saturation(int pulse)
{
    if (pulse < low_limit)
    {
        return low_limit;
    }
    else if (pulse > high_limit)
    {
        return high_limit;
    }
    return pulse;

}

// Turns
void Actuators::Steer::turn(int pulse)
{
    servo.writeMicroseconds(saturation(pulse));
}

// Enable
void Actuators::Steer::enable()
{
    servo.attach(STEERING_PIN);
}

// Disable
void Actuators::Steer::disable()
{
    servo.detach();
}