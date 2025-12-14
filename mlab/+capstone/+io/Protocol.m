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

        function [lineNorm, mask] = normalizeLine(lineRaw, offset, scale, threshold)
            r = double(lineRaw(:)).';
            lineNorm = (r - offset).*scale;
            lineNorm = max(0, min(1, lineNorm));
            mask = lineNorm > threshold;
        end

        function e = computeError(lineNorm, mask, positions)
            w = lineNorm;
            w(~mask) = 0;
            if all(w == 0)
                e = 0;
                return;
            end
            pos = sum(positions .* w) / sum(w);
            e = -pos;
        end
    end
end
