classdef Protocol
    % Protocol constants (ESP32 CMD list aligned; MATLAB legacy aliases kept)
    methods (Static)
        function p = constants()
            p = struct();

            % ===================== MODES / GLOBAL (SYSTEM) =====================
            % Khớp với enum Cmd trong protocol.hpp:
            %   CMD_GLOBAL_OPERATION      = 0x01
            %   CMD_GLOBAL_IDLE           = 0x02
            %   CMD_GLOBAL_PING_MODE      = 0x03
            %   CMD_GLOBAL_EMRGENCY_STOP  = 0x04
            p.CMD_MODE_OP                = uint8(hex2dec('01')); % CMD_GLOBAL_OPERATION
            p.CMD_MODE_IDLE              = uint8(hex2dec('02')); % CMD_GLOBAL_IDLE
            p.CMD_GLOBAL_PING_MODE       = uint8(hex2dec('03')); % CMD_GLOBAL_PING_MODE
            p.CMD_GLOBAL_EMERGENCY_STOP  = uint8(hex2dec('04')); % CMD_GLOBAL_EMRGENCY_STOP

            % ACK byte từ ESP32 sau SYSTEM + IDLE
            p.ACK                        = uint8(hex2dec('20')); % 0x20

            % ===================== OPERATION (ESP32) =====================
            % Khớp với:
            %   CMD_OP_PWM_FWD        = 0x10
            %   CMD_OP_PWM_BWD        = 0x11
            %   CMD_OP_SPD_FWD        = 0x12
            %   CMD_OP_SPD_BWD        = 0x13
            %   CMD_OP_BRAKE          = 0x14
            %   CMD_OP_LEGACY_CTRL_5B = 0x15
            p.CMD_OP_PWM_FWD        = uint8(hex2dec('10')); % payload: u16 (11-bit duty) LE
            p.CMD_OP_PWM_BWD        = uint8(hex2dec('11')); % payload: u16 (11-bit duty) LE
            p.CMD_OP_SPD_FWD        = uint8(hex2dec('12')); % payload: float32 (speed) LE
            p.CMD_OP_SPD_BWD        = uint8(hex2dec('13')); % payload: float32 (speed) LE
            p.CMD_OP_BRAKE          = uint8(hex2dec('14')); % payload: none
            p.CMD_OP_LEGACY_CTRL_5B      = uint8(hex2dec('15')); % speed_u16 + angle_u16 (LE)

            % ===================== IDLE SENSORS (ESP32) =====================
            % Khớp với:
            %   CMD_IDLE_SENSOR_LINE_READ   = 0x20
            %   CMD_IDLE_SENSOR_ULTRA_READ  = 0x21
            %   CMD_IDLE_SENSOR_ULTRA_KICK  = 0x22
            %   CMD_IDLE_SENSOR_MPU_READ    = 0x23
            %   CMD_IDLE_ENCODER_ENABLE     = 0x24
            %   CMD_IDLE_ENCODER_DISABLE    = 0x25
            p.CMD_IDLE_SENSOR_LINE_READ   = uint8(hex2dec('20'));
            p.CMD_IDLE_SENSOR_ULTRA_READ  = uint8(hex2dec('21'));
            p.CMD_IDLE_SENSOR_ULTRA_KICK  = uint8(hex2dec('22'));
            p.CMD_IDLE_SENSOR_MPU_READ    = uint8(hex2dec('23'));
            p.CMD_IDLE_ENCODER_ENABLE     = uint8(hex2dec('24'));
            p.CMD_IDLE_ENCODER_DISABLE    = uint8(hex2dec('25'));

            % ===================== IDLE ACTUATORS (ESP32) =====================
            % Khớp với:
            %   CMD_IDLE_ACTUATOR_MOTOR_ENABLE   = 0x30
            %   CMD_IDLE_ACTUATOR_MOTOR_DISABLE  = 0x31
            %   CMD_IDLE_ACTUATOR_MOTOR_PWM_FWD  = 0x32
            %   CMD_IDLE_ACTUATOR_MOTOR_PWM_BWD  = 0x33
            %   CMD_IDLE_ACTUATOR_MOTOR_SPD_FWD  = 0x34
            %   CMD_IDLE_ACTUATOR_MOTOR_SPD_BWD  = 0x35
            %   CMD_IDLE_ACTUATOR_MOTOR_STOP     = 0x36
            %   CMD_IDLE_ACTUATOR_SERVO_ENABLE        = 0x40
            %   CMD_IDLE_ACTUATOR_SERVO_DISABLE       = 0x41
            %   CMD_IDLE_ACTUATOR_SERVO_WRITE         = 0x42
            %   CMD_IDLE_ACTUATOR_SERVO_READ          = 0x43
            %   CMD_IDLE_ACTUATOR_SERVO_WRITE_CENTER  = 0x44
            p.CMD_IDLE_ACTUATOR_MOTOR_ENABLE      = uint8(hex2dec('30'));
            p.CMD_IDLE_ACTUATOR_MOTOR_DISABLE     = uint8(hex2dec('31'));
            p.CMD_IDLE_ACTUATOR_MOTOR_PWM_FWD     = uint8(hex2dec('32')); % payload: u16 (LE)
            p.CMD_IDLE_ACTUATOR_MOTOR_PWM_BWD     = uint8(hex2dec('33')); % payload: u16 (LE)
            p.CMD_IDLE_ACTUATOR_MOTOR_SPD_FWD     = uint8(hex2dec('34')); % payload: u16 (LE)
            p.CMD_IDLE_ACTUATOR_MOTOR_SPD_BWD     = uint8(hex2dec('35')); % payload: u16 (LE)
            p.CMD_IDLE_ACTUATOR_MOTOR_STOP        = uint8(hex2dec('36'));

            p.CMD_IDLE_ACTUATOR_SERVO_ENABLE      = uint8(hex2dec('40'));
            p.CMD_IDLE_ACTUATOR_SERVO_DISABLE     = uint8(hex2dec('41'));
            p.CMD_IDLE_ACTUATOR_SERVO_WRITE       = uint8(hex2dec('42')); % payload: angle_u16 (LE)
            p.CMD_IDLE_ACTUATOR_SERVO_READ        = uint8(hex2dec('43'));
            p.CMD_IDLE_ACTUATOR_SERVO_WRITE_CENTER= uint8(hex2dec('44'));

            % ===================== IDLE SET PARAMS (payload fixed 30) =====================
            % Khớp với:
            %   CMD_IDLE_SET_LINE_PARAMS   = 0x50
            %   CMD_IDLE_SET_MPU_PARAMS    = 0x51
            %   CMD_IDLE_SET_ULTRA_PARAMS  = 0x52
            %   CMD_IDLE_SET_MOTOR_PARAMS  = 0x53
            %   CMD_IDLE_SET_SERVO_PARAMS  = 0x54
            %   CMD_IDLE_SET_PID_PARAMS    = 0x55
            p.CMD_IDLE_SET_LINE_PARAMS   = uint8(hex2dec('50'));
            p.CMD_IDLE_SET_MPU_PARAMS    = uint8(hex2dec('51'));
            p.CMD_IDLE_SET_ULTRA_PARAMS  = uint8(hex2dec('52'));
            p.CMD_IDLE_SET_MOTOR_PARAMS  = uint8(hex2dec('53'));
            p.CMD_IDLE_SET_SERVO_PARAMS  = uint8(hex2dec('54'));
            p.CMD_IDLE_SET_PID_PARAMS    = uint8(hex2dec('55'));

            % ===================== LEGACY MATLAB NAMES (ALIASES) =====================
            % Giữ alias cũ để không phải sửa toàn bộ code MATLAB:
            p.CMD_READ_LINE   = p.CMD_IDLE_SENSOR_LINE_READ;
            p.CMD_READ_ULTRA  = p.CMD_IDLE_SENSOR_ULTRA_READ;
            p.CMD_READ_MPU    = p.CMD_IDLE_SENSOR_MPU_READ;

            p.CMD_DRIVE_FWD      = p.CMD_IDLE_ACTUATOR_MOTOR_SPD_FWD;
            p.CMD_DRIVE_BWD      = p.CMD_IDLE_ACTUATOR_MOTOR_SPD_BWD;
            p.CMD_DRIVE_STOP     = p.CMD_IDLE_ACTUATOR_MOTOR_STOP;
            p.CMD_STEER_DISABLE  = p.CMD_IDLE_ACTUATOR_SERVO_DISABLE;
            p.CMD_STEER_ANGLE    = p.CMD_IDLE_ACTUATOR_SERVO_WRITE;

            % ===================== FRAMES =====================
            p.SENSOR_FRAME_LEN = uint16(22); % OPERATION telemetry frame length
        end
    end
end
