function [out, st] = onoffController(input, gain, st, cfg) %#ok<INUSD>
    if isempty(st), st = struct(); end

    if ~any(input.mask)
        out = struct("steerOffsetDeg",0.0,"speedScale",1.0);
        return;
    end

    if abs(input.error) < gain.threshold
        u = 0;
    elseif input.error > 0
        u = -gain.angleStep;
    else
        u = gain.angleStep;
    end

    out = struct("steerOffsetDeg", u, "speedScale", 1.0);
end
