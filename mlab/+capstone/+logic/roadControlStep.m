function [ctrlOut, roadRt] = roadControlStep(curRoad, appState, roadRt, proc, cfg)
% ctrlOut: struct(speed_u16, steerDeg, cmd)

ctrlOut = appState.ctrl; % copy base
ctrlOut.cmd = cfg.proto.CMD_CTRL;

% ----- STOP -----
if string(curRoad.type) == "STOP"
    ctrlOut.speed_u16 = uint16(0);
    ctrlOut.steerDeg  = uint16(cfg.steer.centerDeg);
    ctrlOut.cmd       = cfg.proto.CMD_STOP;
    return;
end

% ----- SPECIAL: OBSTACLE_AVOID -----
if string(curRoad.type) == "SPECIAL" && isfield(curRoad,"name") && curRoad.name == "OBSTACLE_AVOID"
    ctrlOut.speed_u16 = curRoad.param.avoidPWM16;
    ctrlOut.steerDeg  = uint16(curRoad.param.avoidSteerDeg);
    return;
end

% ----- NORMAL: DRIVE -----
if isfield(curRoad,"tune") && isfield(curRoad.tune,"drive")
    if string(curRoad.tune.drive.mode) == "CONST"
        ctrlOut.speed_u16 = curRoad.tune.drive.basePWM16;
    else
        ctrlOut.speed_u16 = curRoad.tune.drive.onPWM16; % tối thiểu
    end
end

% ----- NORMAL: STEER -----
ctype = string(curRoad.controller);
if ctype == "ONOFF"
    th = curRoad.tune.steer.onoff.threshold;
    if proc.error > th
        steer = curRoad.tune.steer.onoff.rightDeg;
    elseif proc.error < -th
        steer = curRoad.tune.steer.onoff.leftDeg;
    else
        steer = curRoad.tune.steer.onoff.centerDeg;
    end
    ctrlOut.steerDeg = uint16(steer);

elseif ctype == "PID"
    % placeholder: nếu bạn có hàm PID update riêng thì gọi ở đây
    ctrlOut.steerDeg = uint16(cfg.steer.centerDeg);

else
    % STOP hoặc unknown
    ctrlOut.speed_u16 = uint16(0);
    ctrlOut.steerDeg  = uint16(cfg.steer.centerDeg);
    ctrlOut.cmd       = cfg.proto.CMD_STOP;
end

end
