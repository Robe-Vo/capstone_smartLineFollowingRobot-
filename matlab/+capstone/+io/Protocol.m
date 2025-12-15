% File: +capstone/+io/Protocol.m   (or wherever your Protocol.m is)
classdef Protocol
    methods (Static)
        function proto = constants()
            proto = struct();

            % Modes
            proto.CMD_MODE_OP   = uint8(hex2dec('FF')); % IDLE -> OPERATION
            proto.CMD_MODE_IDLE = uint8(hex2dec('FE')); % OPERATION -> IDLE
            proto.ACK           = uint8(hex2dec('20'));

            % Sensor read flags
            proto.CMD_READ_LINE  = uint8(hex2dec('EF'));
            proto.CMD_READ_ULTRA = uint8(hex2dec('EE'));
            proto.CMD_READ_MPU   = uint8(hex2dec('ED'));

            % Drive direct (NEW, per your firmware switch-case)
            proto.CMD_DRIVE_FWD  = uint8(hex2dec('DF')); % + speed_u16 BE
            proto.CMD_DRIVE_BWD  = uint8(hex2dec('DE')); % + speed_u16 BE
            proto.CMD_DRIVE_STOP = uint8(hex2dec('DD')); % stop motor

            % Steer direct
            proto.CMD_STEER_DISABLE = uint8(hex2dec('DC'));
            proto.CMD_STEER_ANGLE   = uint8(hex2dec('DB')); % + angle_u16 BE

            % Control frame (still available if you use OP frame)
            proto.CMD_CTRL = uint8(hex2dec('F1'));
            proto.CMD_STOP = uint8(hex2dec('F0'));

            % Your existing sensor frame length
            proto.SENSOR_FRAME_LEN = 22;
        end
    end
end
