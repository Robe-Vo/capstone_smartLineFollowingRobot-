#pragma once
#include <Arduino.h>

enum Command : uint8_t
{
    // System command: Command highest level
    COMMAND_SYSTEM_EMERGENCY_STOP       = 0xFF,
    COMMAND_SYSTEM_CHANGE_MODE          = 0xFE,
    COMMAND_SYSTEM_PING_MODE            = 0xFD,
    COMMAND_SYSTEM_SEND_ALL_SIGNAL      = 0xFC,
    
    
    // Control command: Command to control motion of Robot
    // Van be use if Robot is in Operation mode 
    // The command has 1 byte for speed and 2 byte for steering angle follow up
    COMMAND_CONTROL_MOVE_FORWARD        = 0xEF,
    COMMAND_CONTROL_MOVE_BACKWARD       = 0xEE,
    COMMAND_CONTROL_BRAKE               = 0xED,


    // Drive command: Command to drive motor to do an action
    // The command has two byte follow up for the intensity
    COMMAND_DRIVE_DISABLE               = 0xDF,
    COMMAND_DRIVE_ENABLE                = 0xDE,
    COMMAND_DRIVE_ROTATE_FORWARD        = 0xDD,
    COMMAND_DRIVE_ROTATE_BACKWARD       = 0xDC,
    COMMAND_DRIVE_BRAKE                 = 0xDB,


    // Drive command: Command to steer servo to do an action
    // The command has two byte follow up for the intensity
    COMMAND_STEER_DISABLE               = 0xCF,
    COMMAND_STEER_ENABLE                = 0xCE,
    COMMAND_STEER_TURN                  = 0xCD,


    // Sensor command: Send signal to computer
    COMMAND_LINE_SENSOR                 = 0xBF,
    COMMAND_ULTRASONIC_SIGNAL           = 0xBE,
    COMMAND_MPU_ACCEL_SIGNAL            = 0xBD,
    COMMAND_MPU_GYRO_SIGNAL             = 0xBC
};


/** =========================================================================
 *                           SYSTEM HANDLERS DEFINITION
  =========================================================================*/

// ===== SYSTEM HANDLERS =====
void handleEmergencyStop(uint8_t* payload, uint8_t len);
void handleChangeMode(uint8_t* payload, uint8_t len);
void handlePingMode(uint8_t* payload, uint8_t len);
void handleSendAllSignal(uint8_t* payload, uint8_t len);

// ===== CONTROL HANDLERS =====
void handleMoveForward(uint8_t* payload, uint8_t len);
void handleMoveBackward(uint8_t* payload, uint8_t len);
void handleBrake(uint8_t* payload, uint8_t len);

// ===== DRIVE HANDLERS =====
void handleDriveDisable(uint8_t* payload, uint8_t len);
void handleDriveEnable(uint8_t* payload, uint8_t len);
void handleDriveRotateForward(uint8_t* payload, uint8_t len);
void handleDriveRotateBackward(uint8_t* payload, uint8_t len);
void handleDriveBrake(uint8_t* payload, uint8_t len);

// ===== STEER HANDLERS =====
void handleSteerDisable(uint8_t* payload, uint8_t len);
void handleSteerEnable(uint8_t* payload, uint8_t len);
void handleSteerTurn(uint8_t* payload, uint8_t len);

// ===== SENSOR HANDLERS =====
void handleLineSensor(uint8_t* payload, uint8_t len);
void handleUltrasonicSignal(uint8_t* payload, uint8_t len);
void handleMPUAccel(uint8_t* payload, uint8_t len);
void handleMPUGyro(uint8_t* payload, uint8_t len);

void handleUnknownCommand(uint8_t* payload, uint8_t len);

/** =========================================================================
 *                           SYSTEM HANDLERS JUMPIN
  =========================================================================*/

// Create jump in table
typedef void (*CommandHandler)(uint8_t* payload, uint8_t len);

// Declare, don't define, in header
extern CommandHandler PassiveCommandTable[256];
extern CommandHandler OperationCommandTable[256];
extern CommandHandler IdleCommandTable[256];

void initPassiveCommandTable();
void dispatchPassiveCommand(uint8_t cmd, uint8_t* payload, uint8_t len);

void initOperationCommandTable();
void dispatchOperationCommand(uint8_t cmd, uint8_t* payload, uint8_t len);

void initIdleCommandTable();
void dispatchIdleCommand(uint8_t cmd, uint8_t* payload, uint8_t len);
