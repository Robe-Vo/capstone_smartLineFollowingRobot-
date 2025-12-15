% File: +capstone/+io/Protocol.m
classdef Protocol
    % Protocol constants (matches firmware switch-case + OP frame)
    methods (Static)
        function p = constants()
            p = struct();

            % ===== Modes =====
            p.CMD_MODE_OP   = uint8(hex2dec('FF')); % IDLE -> OPERATION
            p.CMD_MODE_IDLE = uint8(hex2dec('FE')); % OPERATION -> IDLE
            p.ACK           = uint8(hex2dec('20'));

            % ===== Direct sensor flags (optional) =====
            p.CMD_READ_LINE  = uint8(hex2dec('EF'));
            p.CMD_READ_ULTRA = uint8(hex2dec('EE'));
            p.CMD_READ_MPU   = uint8(hex2dec('ED'));

            % ===== Direct drive (2 bytes BE, value is PWM11 0..2047) =====
            p.CMD_DRIVE_FWD  = uint8(hex2dec('DF')); % + [hi][lo] (BE)
            p.CMD_DRIVE_BWD  = uint8(hex2dec('DE')); % + [hi][lo] (BE)
            p.CMD_DRIVE_STOP = uint8(hex2dec('DD')); % stop

            % ===== Direct steer (2 bytes BE) =====
            p.CMD_STEER_DISABLE = uint8(hex2dec('DC'));
            p.CMD_STEER_ANGLE   = uint8(hex2dec('DB')); % + [hi][lo] (BE)

            % ===== OPERATION control frame (5 bytes LE) =====
            % [cmd][spd_L][spd_H][ang_L][ang_H]
            p.CMD_OP_CTRL = uint8(hex2dec('F1'));
            p.CMD_OP_STOP = uint8(hex2dec('F0'));

            % ===== Sensor frame (keep your value if used elsewhere) =====
            p.SENSOR_FRAME_LEN = 22;
        end
    end
end
