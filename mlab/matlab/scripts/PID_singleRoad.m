% PID_singleRoad.m
% Simple single-road PID line following using:
% - robot struct (limits, calib, PID)
% - Network class
% - One PID controller with anti-windup
% Press ENTER in the small control window to stop and send 0xFE (IDLE).

% Add others paths
thisDir = fileparts(mfilename('fullpath'));
addpath(fullfile(thisDir, "..", "classes"));
addpath(fullfile(thisDir, "..", "utils/line"));
addpath(fullfile(thisDir, "..", "utils/pid"));
addpath(fullfile(thisDir, "..", "config"));

robot_config;   % script tạo struct robot


%% Basic parameters (assume 'robot' already exists in workspace)
dt      = robot.Ts;           % sample time (s), e.g. 0.025
speedPWM = 150;               % constant speed command (0–255)

% Line calibration
lineWhite = robot.lineBlack;  % your "white" values (near line background)
lineBlack = robot.lineLight;  % your "black" values (on line)
lineOffset = lineWhite;
lineScale  = 1 ./ (lineBlack - lineWhite);
lineThresh = 0.5 * ones(1, numel(lineWhite));

% Sensor positions (left to right)
positions = -floor(numel(lineWhite)/2) : floor(numel(lineWhite)/2);

% PID settings
pid.Kp      = robot.Kp;
pid.Ki      = robot.Ki;
pid.Kd      = robot.Kd;
pid.intMax  = robot.Imax;
pid.outMax  = 15;            % max steering offset [deg], adjust if needed

pidState.intE  = 0.0;
pidState.prevE = 0.0;
pidState.first = true;

% Steering limits
minAngle = robot.MinAngle;
maxAngle = robot.MaxAngle;
baseAngle = robot.centerAngle;

% Logging
k           = 0;
timeLog     = [];
errorLog    = [];
angleLog    = [];
encLog      = [];
encSpeedLog = [];

%% Network connect
net = Network();
if robot.network == "bt"
    net.connectBluetooth(robot.networName, robot.networkPort);
elseif robot.network == "udp"
    % net.connectUDP(robot.ip, robot.port);   % nếu có cấu hình thêm
elseif robot.network == "tcp"
    % net.connectTCP(robot.ip, robot.port);
end

% Enter OPERATION mode: 0xFF, expect ACK 0x20
net.sendByte(hex2dec('FF'));
net.waitAck(2.0);

%% Stop control window (press ENTER to stop)
hCtrl = figure('Name','PID_singleRoad control', ...
               'NumberTitle','off', ...
               'MenuBar','none', ...
               'ToolBar','none');
setappdata(hCtrl, 'stop', false);
set(hCtrl, 'KeyPressFcn', @keyStopCallback);

%% Main loop
encCum = 0;
running = true;

while running
    % Check stop flag / window closed
    if ~ishandle(hCtrl)
        break;
    end
    stopFlag = getappdata(hCtrl, 'stop');
    if ~isempty(stopFlag) && stopFlag
        break;
    end

    % Receive sensor frame
    [ok, lineRaw, ultra, mpu, encCount, encSpeed] = net.recvSensorFrame(1.0); %#ok<NASGU>
    if ~ok
        continue;
    end

    % Line normalize and error
    [lineNorm, mask] = normalizeLine(lineRaw, lineOffset, lineScale, lineThresh);
    e = computeLineError(lineNorm, mask, positions);

    % PID update
    [uDeg, pidState] = pidSteeringUpdate(e, dt, pid, pidState, mask);

    % Build steering angle and clamp
    angleCmd = baseAngle + uDeg;
    angleCmd = max(minAngle, min(maxAngle, angleCmd));
    angleCmd_u16 = uint16(angleCmd);

    % Send command [0xF1 speed angle]
    net.sendControl(uint8(speedPWM), angleCmd_u16, hex2dec('F1'));

    % Logging
    k = k + 1;
    encCum = encCum + encCount;

    timeLog(k,1)     = k * dt;         %#ok<SAGROW>
    errorLog(k,1)    = e;              %#ok<SAGROW>
    angleLog(k,1)    = angleCmd;       %#ok<SAGROW>
    encLog(k,1)      = encCum;         %#ok<SAGROW>
    encSpeedLog(k,1) = encSpeed;       %#ok<SAGROW>
end

%% Stop robot and send IDLE (0xFE + wait ACK 0x20)
try
    % Stop motion
    net.sendControl(uint8(0), uint16(baseAngle), hex2dec('F0'));
    pause(0.1);

    % Send IDLE command 0xFE
    net.sendByte(hex2dec('FE'));
    ackOK = net.waitAck(2.0); %#ok<NASGU>
catch
end

net.disconnect();
if ishandle(hCtrl)
    close(hCtrl);
end

% timeLog, errorLog, angleLog, encLog, encSpeedLog are now in workspace.


%% Local helper functions
function keyStopCallback(src, evt)
    if strcmp(evt.Key, 'return')  % ENTER
        setappdata(src, 'stop', true);
    end
end