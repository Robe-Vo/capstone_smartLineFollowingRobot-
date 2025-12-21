#pragma once
#include <Arduino.h>

#include "../../include/cfg.hpp"
#include "bluetooth.hpp"
#include "controller.hpp"
#include "sensors.hpp"
#include "encoder.hpp"
#include "protocol.hpp"
#include "dispatch.hpp"

/**
 * ======================================================================
 *                                STATE
 * ======================================================================
 * 
 *  This library control state and states' statuses of entirer system
 * */

enum class Mode: uint8_t {IDLE, OPERATION};
extern Mode robotMode;

/**
 *          ===== IDLE PARAMS ===== 
 * */
struct Idle_values
{
    // Motor flag
    bool motor_isWorking = false;
    // 0: Disable 
    // 1: (PID) Run forward - 2: (PID) Run backward 
    // 1: Run forward - 2: Run backward
    uint8_t motor_mode = 0;   

    // Servo flag
    bool servo_isWorking;
    int16_t servo_writeAngle = SERVO_MID_DEG;

    // Line sensors
    uint8_t* line_signals;

    // Ultra sensors
    uint16_t ultra_signal;

    // MPU sensor
    uint16_t* mpu_signal;
};

extern Idle_values idle_params;

/**
 *          ===== OPERATION PARAMS ===== 
 * */
struct Op_RX_values
{
    // Run mode
    bool direction     = true;      // true: Forward | false: Backward
    bool useController = true;      // true: PID     | false: PWM
    bool isRunning     = true;     // Prevent controller to run while braking (block action)

    // Control params
    float speed = 0.0f;
    uint16_t pwm = 0;
};
extern Op_RX_values op_params;

struct Op_TX_values
{
    // Line sensors
    uint8_t* line_signals;

    // Ultra sensors
    uint16_t ultra_signal;

    // MPU sensor - now is invalid
    int16_t mpu_signal[5] = {0,0,0,0,0};

    // Encoder accumulate
    int count;
    float speed;
};
extern Op_TX_values op_signals;



/**=======================================
 *              STATE CONTROL
 * ======================================= 
 */ 
 // Init IDLE
// Call when setup esp32 and switching mode
void IDLE_init();

// Init OPERATION
// Call when setup esp32 and switching mode
void OPERATION_init();

// Repeat behavior
// Controller calculation + Control actuators+ Data get
void OPERATION_action();

// Return robot's current state
// Return 1 if IDLE, 2 if OPERATION
bool get_state();


/**=======================================
 *              ROBOT SYSTEM
 * ======================================= 
 */ 
// System declaration
extern Network net;
extern Controller::Cfg pid_params;   // <-- extern (no definition in header)
extern Controller::State controller_params;
    
// Init robot hardware
void robot_init();

// RTOS handles
static TaskHandle_t control_handle       = nullptr;
static TaskHandle_t communication_handle = nullptr;
static SemaphoreHandle_t g_stateMutex = nullptr;


// Communicate to pc
void robot_communication(void* pv);

// Take control action 
void robot_calculation(void* pv);