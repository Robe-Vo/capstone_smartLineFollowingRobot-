% Action of lost policy
function ctrl = applyLostPolicy(lastMask, cfg)
    
    ctrl = struct();
    
    if isequal(lastMask,[1 0 0 0 0])
        ctrl.steerDeg = cfg.steer.maxLeft;
        ctrl.speed    = cfg.speed.search;
    elseif isequal(lastMask,[0 0 0 0 1])
        ctrl.steerDeg = cfg.steer.maxRight;
        ctrl.speed    = cfg.speed.search;
    else
        ctrl.speed = 0;
        ctrl.steerDeg = cfg.steer.center;
    end
end
