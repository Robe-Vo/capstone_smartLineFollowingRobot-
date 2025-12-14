classdef Protocol
    methods (Static)
        function proto = constants()
            proto = struct();
            proto.CMD_MODE_OP   = uint8(hex2dec('FF'));
            proto.CMD_MODE_IDLE = uint8(hex2dec('FE'));
            proto.CMD_CTRL      = uint8(hex2dec('F1'));
            proto.CMD_STOP      = uint8(hex2dec('F0'));
            proto.ACK           = uint8(hex2dec('20'));
            proto.SENSOR_FRAME_LEN = 22;
        end
    end
end
