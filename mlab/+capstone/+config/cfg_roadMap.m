function roadMap = cfg_roadMap(cfg)
% cfg_roadMap: Road list + tune embedded per-road (NO TuneDB).
%
% Road IDs (numeric):
%   1 = STRAIGHT (test)
%   2 = CURVE (placeholder)
%   3 = OBSTACLE_AVOID (blind / special)
%   4 = STOP (end)
%
% Each road has:
%   .id, .name, .type, .controller, .lengthEnc, .tune, .transition

if nargin < 1, cfg = struct(); end

% ----- Common defaults (safe) -----
if ~isfield(cfg,"steer") || ~isfield(cfg.steer,"centerDeg")
    cfg.steer.centerDeg = 75;
end

% ========== ROAD 1: STRAIGHT ==========
roadMap(1).id   = 1;
roadMap(1).name = "STRAIGHT_1";
roadMap(1).type = "NORMAL";          % NORMAL / SPECIAL / STOP
roadMap(1).controller = "ONOFF";     % PID / ONOFF / STOP

% Length in encoder pulses (MATLAB-accumulated)
roadMap(1).lengthEnc = 500;          % <--- bạn chỉnh để test

% Tune embedded
roadMap(1).tune.drive = struct( ...
    "mode","CONST", ...              % CONST / ONOFF
    "basePWM16", uint16(40000), ...  % <--- speed test
    "onPWM16",  uint16(40000), ...
    "offPWM16", uint16(0));

roadMap(1).tune.steer = struct( ...
    "mode","ONOFF", ...              % PID / ONOFF
    "pid", struct("Kp",0,"Ki",0,"Kd",0,"satDeg",20), ...
    "onoff", struct( ...
        "threshold", 0.20, ...       % <--- ngưỡng error
        "leftDeg",   65, ...
        "rightDeg",  85, ...
        "centerDeg", double(cfg.steer.centerDeg)));

roadMap(1).transition = { ...
    struct("cond","ENC_REACHED","next",4) ... % đủ xung -> STOP
};

% ========== ROAD 2: CURVE (placeholder) ==========
roadMap(2).id   = 2;
roadMap(2).name = "CURVE_1";
roadMap(2).type = "NORMAL";
roadMap(2).controller = "ONOFF";
roadMap(2).lengthEnc = 300;

roadMap(2).tune = roadMap(1).tune;                 % reuse
roadMap(2).tune.drive.basePWM16 = uint16(32000);   % chậm hơn
roadMap(2).tune.steer.onoff.threshold = 0.15;

roadMap(2).transition = { ...
    struct("cond","ENC_REACHED","next",4) ...
};

% ========== ROAD 3: OBSTACLE_AVOID (special/blind) ==========
roadMap(3).id   = 3;
roadMap(3).name = "OBSTACLE_AVOID";
roadMap(3).type = "SPECIAL";
roadMap(3).controller = "SPECIAL";     % do handler riêng
roadMap(3).lengthEnc = 0;

roadMap(3).param = struct( ...
    "avoidSteerDeg", 90, ...           % <--- ví dụ: né sang phải
    "avoidPWM16",    uint16(25000), ...
    "timeoutS",      2.0);             % <--- né tối đa 2s

roadMap(3).transition = { ...
    struct("cond","LINE_FOUND","next","PREV"), ...
    struct("cond","TIMEOUT","next","PREV") ...
};

% ========== ROAD 4: STOP ==========
roadMap(4).id   = 4;
roadMap(4).name = "STOP";
roadMap(4).type = "STOP";
roadMap(4).controller = "STOP";
roadMap(4).lengthEnc = 0;
roadMap(4).tune = struct();
roadMap(4).transition = {};

end
