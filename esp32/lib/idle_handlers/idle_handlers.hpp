#pragma once
#include <Arduino.h>
#include "dispatch.hpp"

#include "protocol.hpp"
#include "state.hpp"
#include "actuators.hpp"
#include "../sensors/sensors.hpp"
#include "../../include/cfg.hpp"

/**
 *      ===== IDLE HANDLERS =====
 * 
 * Run directly when they are called
 */

 /**
 * -------- MOTOR CMD -------- 
 * CMD: 0xD_
*/

static void motor_enable(const Protocol::RxFrame& f)
{
    idle_params.motor_isWorking = true;
}

// Disable motor
static void motor_disable(const Protocol::RxFrame& f)
{
    idle_params.motor_isWorking = false;
}

// Run motor controlled by PID controller
static void motor_spd_fwd(const Protocol::RxFrame& f)
{
    idle_params.motor_mode = 1;
}

static void motor_spd_bwd(const Protocol::RxFrame& f)
{
    idle_params.motor_mode = 2;
}

// Run motor controlled by PWM
static void motor_bwm_fwd(const Protocol::RxFrame& f)
{
    idle_params.motor_mode = 3;
}

static void motor_bwm_bwd(const Protocol::RxFrame& f)
{
    idle_params.motor_mode = 4;
}

/**
 * -------- SERVOR CMD -------- 
 * CMD: 0xD_
*/

// Set servo flag 
static void servo_enable(const Protocol::RxFrame& f)
{
    idle_params.servo_isWorking = true;
}

static void servo_disable(const Protocol::RxFrame& f)
{
    idle_params.servo_isWorking = false;
}

static void servo_write(const Protocol::RxFrame& f)
{
    idle_params.servo_writeAngle = f.angle_i16;
}

static void servo_write_middle(const Protocol::RxFrame& f)
{
    idle_params.servo_writeAngle = SERVO_MID_DEG;
}


// Not available
static void servo_read(const Protocol::RxFrame& f)
{
    
}


/**
 * -------- SERVOR CMD -------- 
 * CMD: 0xC_
*/
static void sensor_read_line(const Protocol::RxFrame& f)
{
    
}

static void sensor_read_ultra(const Protocol::RxFrame& f)
{

}

// MPU not available yet
static void sensor_read_mpu(const Protocol::RxFrame& f)
{

}

/**
 * -------- SETTING CMD -------- 
 * CMD: 0xB_
*/
static void set_line_params(const Protocol::RxFrame& f)
{
    // uint8_t: number of sample
}

// Not available yet
static void set_ultra_params(const Protocol::RxFrame& f)
{

}

// Not available yet
static void set_mpu_params(const Protocol::RxFrame& f)
{

}

static void set_motor_params(const Protocol::RxFrame& f)
{

}

static void set_servo_params(const Protocol::RxFrame& f)
{

}

static void set_pid_params(const Protocol::RxFrame& f)
{

}

#pragma once
#include "protocol.hpp"

namespace IdleHandlers
{
  // MOTOR
  void motor_enable(const Protocol::RxFrame& f);
  void motor_disable(const Protocol::RxFrame& f);
  void motor_spd_fwd(const Protocol::RxFrame& f);
  void motor_spd_bwd(const Protocol::RxFrame& f);
  void motor_pwm_fwd(const Protocol::RxFrame& f);
  void motor_pwm_bwd(const Protocol::RxFrame& f);
  void motor_brake(const Protocol::RxFrame& f);

  // SERVO
  void servo_enable(const Protocol::RxFrame& f);
  void servo_disable(const Protocol::RxFrame& f);
  void servo_write(const Protocol::RxFrame& f);
  void servo_write_center(const Protocol::RxFrame& f);
  void servo_read(const Protocol::RxFrame& f);

  // SENSOR
  void sensor_read_line(const Protocol::RxFrame& f);
  void sensor_read_ultra(const Protocol::RxFrame& f);
  void sensor_read_mpu(const Protocol::RxFrame& f);

  // SET PARAMS (30B)
  void set_line_params(const Protocol::RxFrame& f);
  void set_ultra_params(const Protocol::RxFrame& f);
  void set_mpu_params(const Protocol::RxFrame& f);
  void set_motor_params(const Protocol::RxFrame& f);
  void set_servo_params(const Protocol::RxFrame& f);
  void set_pid_params(const Protocol::RxFrame& f);
}
