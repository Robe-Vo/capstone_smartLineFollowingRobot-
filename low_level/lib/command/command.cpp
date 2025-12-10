#include "Command.hpp"

// Define both tables
CommandHandler PassiveCommandTable[256];
CommandHandler OperationCommandTable[256];
CommandHandler IdleCommandTable[256];


// ------- existing passive init (you already have something like this) -------
void initPassiveCommandTable()
{
    for (int i = 0; i < 256; ++i)
        PassiveCommandTable[i] = handleUnknownCommand;

    // System
    PassiveCommandTable[COMMAND_SYSTEM_EMERGENCY_STOP]   = handleEmergencyStop;
    PassiveCommandTable[COMMAND_SYSTEM_CHANGE_MODE]      = handleChangeMode;
    PassiveCommandTable[COMMAND_SYSTEM_PING_MODE]        = handlePingMode;
    PassiveCommandTable[COMMAND_SYSTEM_SEND_ALL_SIGNAL]  = handleSendAllSignal;

    // Control
    // PassiveCommandTable[COMMAND_CONTROL_MOVE_FORWARD]    = handleMoveForward;
    // PassiveCommandTable[COMMAND_CONTROL_MOVE_BACKWARD]   = handleMoveBackward;
    // PassiveCommandTable[COMMAND_CONTROL_BRAKE]           = handleBrake;

    // Drive
    PassiveCommandTable[COMMAND_DRIVE_DISABLE]           = handleDriveDisable;
    PassiveCommandTable[COMMAND_DRIVE_ENABLE]            = handleDriveEnable;
    PassiveCommandTable[COMMAND_DRIVE_ROTATE_FORWARD]    = handleDriveRotateForward;
    PassiveCommandTable[COMMAND_DRIVE_ROTATE_BACKWARD]   = handleDriveRotateBackward;
    PassiveCommandTable[COMMAND_DRIVE_BRAKE]             = handleDriveBrake;

    // Steer
    PassiveCommandTable[COMMAND_STEER_DISABLE]           = handleSteerDisable;
    PassiveCommandTable[COMMAND_STEER_ENABLE]            = handleSteerEnable;
    PassiveCommandTable[COMMAND_STEER_TURN]              = handleSteerTurn;

    // Sensor
    PassiveCommandTable[COMMAND_LINE_SENSOR]             = handleLineSensor;
    PassiveCommandTable[COMMAND_ULTRASONIC_SIGNAL]       = handleUltrasonicSignal;
    PassiveCommandTable[COMMAND_MPU_ACCEL_SIGNAL]        = handleMPUAccel;
    PassiveCommandTable[COMMAND_MPU_GYRO_SIGNAL]         = handleMPUGyro;
}

void dispatchPassiveCommand(uint8_t cmd, uint8_t* payload, uint8_t len)
{
    CommandHandler h = PassiveCommandTable[cmd];
    h(payload, len);
}

// ------- NEW: operation mode init -------

void initOperationCommandTable()
{
    for (int i = 0; i < 256; ++i)
        OperationCommandTable[i] = handleUnknownCommand;

    // In operation mode, you usually allow *all* motion-related commands.
    // Right now I mirror the same mapping as passive; you can change
    // which commands are enabled or map to different behaviour.

    // System
    OperationCommandTable[COMMAND_SYSTEM_EMERGENCY_STOP]   = handleEmergencyStop;
    OperationCommandTable[COMMAND_SYSTEM_CHANGE_MODE]      = handleChangeMode;
    OperationCommandTable[COMMAND_SYSTEM_PING_MODE]        = handlePingMode;
    OperationCommandTable[COMMAND_SYSTEM_SEND_ALL_SIGNAL]  = handleSendAllSignal;

    // Control
    OperationCommandTable[COMMAND_CONTROL_MOVE_FORWARD]    = handleMoveForward;
    OperationCommandTable[COMMAND_CONTROL_MOVE_BACKWARD]   = handleMoveBackward;
    OperationCommandTable[COMMAND_CONTROL_BRAKE]           = handleBrake;

    // // Drive
    // OperationCommandTable[COMMAND_DRIVE_DISABLE]           = handleDriveDisable;
    // OperationCommandTable[COMMAND_DRIVE_ENABLE]            = handleDriveEnable;
    // OperationCommandTable[COMMAND_DRIVE_ROTATE_FORWARD]    = handleDriveRotateForward;
    // OperationCommandTable[COMMAND_DRIVE_ROTATE_BACKWARD]   = handleDriveRotateBackward;
    // OperationCommandTable[COMMAND_DRIVE_BRAKE]             = handleDriveBrake;

    // // Steer
    // OperationCommandTable[COMMAND_STEER_DISABLE]           = handleSteerDisable;
    // OperationCommandTable[COMMAND_STEER_ENABLE]            = handleSteerEnable;
    // OperationCommandTable[COMMAND_STEER_TURN]              = handleSteerTurn;

    // // Sensor
    // OperationCommandTable[COMMAND_LINE_SENSOR]             = handleLineSensor;
    // OperationCommandTable[COMMAND_ULTRASONIC_SIGNAL]       = handleUltrasonicSignal;
    // OperationCommandTable[COMMAND_MPU_ACCEL_SIGNAL]        = handleMPUAccel;
    // OperationCommandTable[COMMAND_MPU_GYRO_SIGNAL]         = handleMPUGyro;
}

void dispatchOperationCommand(uint8_t cmd, uint8_t* payload, uint8_t len)
{
    CommandHandler h = OperationCommandTable[cmd];
    h(payload, len);
}

// ------- NEW: operation mode init -------

void initIdleCommandTable()
{
    for (int i = 0; i < 256; ++i)
        IdleCommandTable[i] = handleUnknownCommand;

    // In operation mode, you usually allow *all* motion-related commands.
    // Right now I mirror the same mapping as passive; you can change
    // which commands are enabled or map to different behaviour.

    // System
    IdleCommandTable[COMMAND_SYSTEM_EMERGENCY_STOP]   = handleEmergencyStop;
    IdleCommandTable[COMMAND_SYSTEM_CHANGE_MODE]      = handleChangeMode;
    IdleCommandTable[COMMAND_SYSTEM_PING_MODE]        = handlePingMode;
    IdleCommandTable[COMMAND_SYSTEM_SEND_ALL_SIGNAL]  = handleSendAllSignal;

    // // Control
    // IdleCommandTable[COMMAND_CONTROL_MOVE_FORWARD]    = handleMoveForward;
    // IdleCommandTable[COMMAND_CONTROL_MOVE_BACKWARD]   = handleMoveBackward;
    // IdleCommandTable[COMMAND_CONTROL_BRAKE]           = handleBrake;

    // // Drive
    // IdleCommandTable[COMMAND_DRIVE_DISABLE]           = handleDriveDisable;
    // IdleCommandTable[COMMAND_DRIVE_ENABLE]            = handleDriveEnable;
    // IdleCommandTable[COMMAND_DRIVE_ROTATE_FORWARD]    = handleDriveRotateForward;
    // IdleCommandTable[COMMAND_DRIVE_ROTATE_BACKWARD]   = handleDriveRotateBackward;
    // IdleCommandTable[COMMAND_DRIVE_BRAKE]             = handleDriveBrake;

    // // Steer
    // IdleCommandTable[COMMAND_STEER_DISABLE]           = handleSteerDisable;
    // IdleCommandTable[COMMAND_STEER_ENABLE]            = handleSteerEnable;
    // IdleCommandTable[COMMAND_STEER_TURN]              = handleSteerTurn;

    // // Sensor
    // IdleCommandTable[COMMAND_LINE_SENSOR]             = handleLineSensor;
    // IdleCommandTable[COMMAND_ULTRASONIC_SIGNAL]       = handleUltrasonicSignal;
    // IdleCommandTable[COMMAND_MPU_ACCEL_SIGNAL]        = handleMPUAccel;
    // IdleCommandTable[COMMAND_MPU_GYRO_SIGNAL]         = handleMPUGyro;
}

void dispatchIdleCommand(uint8_t cmd, uint8_t* payload, uint8_t len)
{
    CommandHandler h = IdleCommandTable[cmd];
    h(payload, len);
}
