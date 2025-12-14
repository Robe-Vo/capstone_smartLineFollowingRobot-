function [ctrl, roadRt] = handleTJunction(proc, roadParam, cfg, roadRt)

if roadParam.turnDir == "RIGHT"
    ctrl.steerDeg = cfg.steer.maxRight;
else
    ctrl.steerDeg = cfg.steer.maxLeft;
end

ctrl.speed = cfg.speed.slow;

% Điều kiện kết thúc:
if proc.mask(3)==1 && proc.quality=="OK"
    roadRt.specialDone = true;
end
end
