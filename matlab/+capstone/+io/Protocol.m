% File: +capstone/+io/Protocol.m
classdef Protocol
    % Protocol constants (ESP32 CMD list aligned; MATLAB legacy aliases kept)
    methods (Static)
        function p = constants()
            p = struct();

            % ===================== MODES / GLOBAL =====================
            p.CMD_MODE_OP             = uint8(hex2dec('FF')); % CMD_GLOBAL_OPERATION
            p.CMD_MODE_IDLE           = uint8(hex2dec('FE')); % CMD_GLOBAL_IDLE
            p.CMD_GLOBAL_PING_MODE    = uint8(hex2dec('FD')); % CMD_GLOBAL_PING_MODE
            p.CMD_GLOBAL_EMERGENCY_STOP = uint8(hex2dec('F0')); % CMD_GLOBAL_EMRGENCY_STOP

            % ACK (IDLE command ping-back)
            p.ACK                     = uint8(hex2dec('20'));

            % ===================== OPERATION (ESP32) =====================
            % NOTE: OPERATION control frame you used earlier:
            %   [cmd][spd_L][spd_H][ang_L][ang_H]  (LE)
            % But ESP32 list shows separate OPERATION commands (PWM/Speed/Brake).
            % Keep both: frame-based CTRL/STOP (legacy) and granular OP commands.

            % Legacy OP frame commands (if firmware still supports them)
            p.CMD_OP_CTRL             = uint8(hex2dec('F1')); % legacy: 5B frame
            p.CMD_OP_STOP             = uint8(hex2dec('F0')); % legacy stop (conflicts with global emergency stop)

            % ESP32 OPERATION commands (granular)
            p.CMD_OP_PWM_FWD          = uint8(hex2dec('EF'));
            p.CMD_OP_PWM_BWD          = uint8(hex2dec('EE'));
            p.CMD_OP_SPD_FWD          = uint8(hex2dec('ED'));
            p.CMD_OP_SPD_BWD          = uint8(hex2dec('EC'));
            p.CMD_OP_BRAKE            = uint8(hex2dec('EB'));

            % ===================== IDLE SENSORS (ESP32) =====================
            p.CMD_IDLE_SENSOR_LINE_READ   = uint8(hex2dec('DF'));
            p.CMD_IDLE_SENSOR_ULTRA_READ  = uint8(hex2dec('DE'));
            p.CMD_IDLE_SENSOR_ULTRA_KICK  = uint8(hex2dec('DD'));
            p.CMD_IDLE_SENSOR_MPU_READ    = uint8(hex2dec('DC'));
            p.CMD_IDLE_ENCODER_ENABLE     = uint8(hex2dec('DB'));
            p.CMD_IDLE_ENCODER_DISABLE    = uint8(hex2dec('DA'));

            % ===================== IDLE ACTUATORS (ESP32) =====================
            p.CMD_IDLE_ACTUATOR_MOTOR_ENABLE      = uint8(hex2dec('CF'));
            p.CMD_IDLE_ACTUATOR_MOTOR_DISABLE     = uint8(hex2dec('CE'));
            p.CMD_IDLE_ACTUATOR_MOTOR_PWM_FWD     = uint8(hex2dec('CD')); % optional payload
            p.CMD_IDLE_ACTUATOR_MOTOR_PWM_BWD     = uint8(hex2dec('CC')); % optional payload
            p.CMD_IDLE_ACTUATOR_MOTOR_SPD_FWD     = uint8(hex2dec('CB')); % optional payload
            p.CMD_IDLE_ACTUATOR_MOTOR_SPD_BWD     = uint8(hex2dec('CA')); % optional payload
            p.CMD_IDLE_ACTUATOR_MOTOR_STOP        = uint8(hex2dec('C9'));

            p.CMD_IDLE_ACTUATOR_SERVO_ENABLE      = uint8(hex2dec('C8'));
            p.CMD_IDLE_ACTUATOR_SERVO_DISABLE     = uint8(hex2dec('C7'));
            p.CMD_IDLE_ACTUATOR_SERVO_WRITE       = uint8(hex2dec('C6')); % payload: angle_u16
            p.CMD_IDLE_ACTUATOR_SERVO_READ        = uint8(hex2dec('C5'));
            p.CMD_IDLE_ACTUATOR_SERVO_WRITE_CENTER= uint8(hex2dec('C4'));

            % ===================== IDLE SET PARAMS (payload fixed 30) =====================
            p.CMD_IDLE_SET_LINE_PARAMS   = uint8(hex2dec('BF'));
            p.CMD_IDLE_SET_MPU_PARAMS    = uint8(hex2dec('BE'));
            p.CMD_IDLE_SET_ULTRA_PARAMS  = uint8(hex2dec('BD'));
            p.CMD_IDLE_SET_MOTOR_PARAMS  = uint8(hex2dec('BC'));
            p.CMD_IDLE_SET_SERVO_PARAMS  = uint8(hex2dec('BB'));
            p.CMD_IDLE_SET_PID_PARAMS    = uint8(hex2dec('BA'));

            % ===================== LEGACY MATLAB NAMES (ALIASES) =====================
            % These existed in your old MATLAB file; remap them to ESP32 commands.
            % If any external MATLAB code calls these, it will still work.

            % Direct sensor flags (legacy) -> IDLE sensor read
            p.CMD_READ_LINE  = p.CMD_IDLE_SENSOR_LINE_READ;
            p.CMD_READ_ULTRA = p.CMD_IDLE_SENSOR_ULTRA_READ;
            p.CMD_READ_MPU   = p.CMD_IDLE_SENSOR_MPU_READ;

            % Direct drive (legacy DF/DE/DD/DB/DC) do not match ESP32 list.
            % Provide aliases to closest semantics:
            % - DRIVE_FWD/BWD: map to IDLE motor speed fwd/bwd (CB/CA) (optional payload in firmware)
            % - DRIVE_STOP: map to IDLE motor stop (C9)
            % - STEER_DISABLE: map to IDLE servo disable (C7)
            % - STEER_ANGLE: map to IDLE servo write (C6)
            p.CMD_DRIVE_FWD      = p.CMD_IDLE_ACTUATOR_MOTOR_SPD_FWD;
            p.CMD_DRIVE_BWD      = p.CMD_IDLE_ACTUATOR_MOTOR_SPD_BWD;
            p.CMD_DRIVE_STOP     = p.CMD_IDLE_ACTUATOR_MOTOR_STOP;
            p.CMD_STEER_DISABLE  = p.CMD_IDLE_ACTUATOR_SERVO_DISABLE;
            p.CMD_STEER_ANGLE    = p.CMD_IDLE_ACTUATOR_SERVO_WRITE;

            % ===================== FRAMES =====================
            p.SENSOR_FRAME_LEN = 22;

            % Endianness notes (for callers):
            % - Network OP control frame in your Network class is LE for speed/angle.
            % - IDLE servo write payload: angle_u16 (endianness must match firmware parse; use LE unless firmware says BE).
        end
    end
end
