function PID_program_singleState()
% Single-state PID line following (no Network class)
% Control TX frame (5B, little-endian):
%   [cmd][speed_L][speed_H][angle_L][angle_H]
% speed uses 11-bit (0..2047) on MATLAB side.

    %% ===== 0) USER PARAMETERS =====
    btName    = "Behind the scream";
    btChannel = 1;

    % --- Robot / geometry ---
    wheelDimension = 0.072;      %#ok<NASGU>
    encoderPPR     = 44;         %#ok<NASGU>
    distPerPulse   = 0.25*pi*wheelDimension/encoderPPR; %#ok<NASGU>

    % --- Global control params ---
    params.minAngle    = 55;
    params.maxAngle    = 110;
    params.deadbandPos = 0.10;

    % Base speed (11-bit)
    params.baseSpeed   = uint16(600);   % 0..2047
    params.centerAngle = uint16(86);

    % --- PID steering parameters ---
    pid.Kp     = 50.0;
    pid.Ki     = 0.0;
    pid.Kd     = 0.2;
    pid.intMax = 10.0;
    pid.outMax = 45.0;

    % --- Calibration ---
    white = [  32.450   30.980   30.930   32.520   32.1600];
    black = [ 131.570  163.400  115.840   50.370   56.190];

    lineOffset    = white;
    lineScale     = 1 ./ (black - white);
    lineThreshold = 0.5 * ones(1,5);

    positions = [-2 -2 0 2 2];

    %% ===== 1) CONNECT BLUETOOTH =====
    conn = helper_connectBluetooth(btName, btChannel);
    fprintf("Connected (BT %s ch %d)\n", btName, btChannel);

    %% ===== 2) ENTER OPERATION MODE (0xFF + wait ACK 0x20) =====
    helper_sendBytes(conn, uint8(hex2dec('FF')));
    if ~helper_waitAck(conn, 2.0, uint8(hex2dec('20')))
        warning("No ACK (0x20) after entering OPERATION mode!");
    else
        fprintf("ACK (0x20) received. Robot is in OPERATION mode.\n");
    end

    %% ===== 3) PID STATE VARIABLES =====
    pidState.intE   = 0.0;
    pidState.prevE  = 0.0;
    pidState.first  = true;

    tPrev = tic;

    fprintf("Start SINGLE-STATE PID. Press Ctrl+C to stop.\n");

    %% ===== MAIN LOOP =====
    try
        while true
            dt = toc(tPrev);
            if dt <= 0, dt = 1e-3; end
            tPrev = tic;

            % --- Read sensor frame from robot ---
            % NOTE: You must adapt this to your actual on-wire sensor frame format.
            % Placeholder: expects you already have a function that returns:
            % [ok, lineRaw, ultra, mpu, encCount, encSpeed]
            [ok, lineRaw, ~, ~, ~, ~] = helper_recvSensorFrame_placeholder(conn, 1.0); %#ok<ASGLU>
            if ~ok
                fprintf("Timeout waiting frame...\n");
                continue;
            end

            % --- Normalize line ---
            [lineNorm, activeMask] = computeNormalize_calib( ...
                lineRaw, lineOffset, lineScale, lineThreshold);

            % --- Compute error ---
            e = computeError(lineNorm, activeMask, positions);

            % --- PID steering ---
            [steerOffsetDeg, pidState] = pidUpdateSteering( ...
                e, dt, pid, pidState, activeMask);

            % --- Command ---
            [cmd, speedCmd, angleCmd] = singleState_control( ...
                e, steerOffsetDeg, activeMask, params);

            % --- Send control (5B, LE, speed 11-bit) ---
            helper_sendControl5B(conn, speedCmd, angleCmd, cmd);
        end

    catch ME
        fprintf("ERROR: %s\n", ME.message);
    end

    %% ===== CLEAN UP =====
    try
        helper_sendControl5B(conn, uint16(0), params.centerAngle, uint8(hex2dec('F0'))); % stop
        helper_sendBytes(conn, uint8(hex2dec('FE')));                                    % back to IDLE
    catch
    end

    helper_disconnect(conn);
    fprintf("PID_program_singleState finished.\n");
end

%% ========================================================================
%% Helper I/O (NO CLASS)
%% ========================================================================

function conn = helper_connectBluetooth(deviceName, channelID)
    bt = bluetooth(deviceName, channelID);
    bt.Timeout = 5;
    conn = struct("type","bt","dev",bt);
end

function helper_disconnect(conn) %#ok<INUSD>
% For BT objects, letting it go is usually enough; keep explicit for symmetry.
end

function helper_sendBytes(conn, bytes_u8)
    if isempty(conn) || ~isfield(conn,"dev") || isempty(conn.dev), return; end
    b = uint8(bytes_u8(:)); % column
    if conn.type == "bt" || conn.type == "tcp"
        write(conn.dev, b, "uint8");
    elseif conn.type == "udp"
        write(conn.dev, b, "uint8", conn.ip, conn.port);
    end
end

function rx = helper_recvByte(conn)
    rx = [];
    if isempty(conn) || ~isfield(conn,"dev") || isempty(conn.dev), return; end

    if conn.type == "bt" || conn.type == "tcp"
        if conn.dev.NumBytesAvailable > 0
            rx = read(conn.dev, 1, "uint8");
        end
    elseif conn.type == "udp"
        u = conn.dev;
        if u.NumDatagramsAvailable > 0
            d = read(u, 1, "uint8");
            if isa(d,'udpport.datagram.Datagram')
                p = d.Data;
            else
                p = d;
            end
            if ~isempty(p)
                rx = p(1);
            end
        end
    end
end

function frame = helper_readFrameBlocking(conn, frameLen, timeout)
    if nargin < 3, timeout = 1.0; end
    frame = uint8([]);
    t0 = tic;
    while numel(frame) < frameLen
        if toc(t0) > timeout
            frame = [];
            return;
        end
        b = helper_recvByte(conn);
        if isempty(b)
            pause(0.001);
            continue;
        end
        frame = [frame; b]; %#ok<AGROW>
    end
end

function ok = helper_waitAck(conn, timeout, targetByte)
    if nargin < 2, timeout = 1.0; end
    if nargin < 3, targetByte = uint8(hex2dec('20')); end
    ok = false;

    t0 = tic;
    while toc(t0) < timeout
        b = helper_recvByte(conn);
        if isempty(b)
            pause(0.001);
            continue;
        end
        if uint8(b) == uint8(targetByte)
            ok = true;
            return;
        end
    end
end

function helper_sendControl5B(conn, speed_u16, angle_u16, cmd_u8)
% 5B control frame, LE:
% [cmd][spd_L][spd_H][ang_L][ang_H]
% speed forced to 11-bit (0..2047)

    if nargin < 4, cmd_u8 = uint8(hex2dec('F1')); end

    spd = uint16(speed_u16);
    spd = bitand(spd, uint16(2047)); % 11-bit

    ang = uint16(angle_u16);

    pkt = uint8(zeros(5,1));
    pkt(1) = uint8(cmd_u8);
    pkt(2) = uint8(bitand(spd, uint16(255)));
    pkt(3) = uint8(bitshift(spd, -8));
    pkt(4) = uint8(bitand(ang, uint16(255)));
    pkt(5) = uint8(bitshift(ang, -8));

    helper_sendBytes(conn, pkt);
end

%% ========================================================================
%% PID + Line Processing (unchanged logic)
%% ========================================================================

function [lineNorm, activeMask] = computeNormalize_calib(lineRaw, offset, scale, threshold)
    r = double(lineRaw(:)).';
    lineNorm = (r - offset) .* scale;
    lineNorm = max(0, min(1, lineNorm));
    activeMask = lineNorm > threshold;
end

function e = computeError(lineNorm, activeMask, positions)
    w = lineNorm;
    w(~activeMask) = 0;

    if all(w == 0)
        e = 0;
        return;
    end

    pos = sum(positions .* w) / sum(w);
    e   = -pos;
end

function [uDeg, pidState] = pidUpdateSteering(e, dt, pid, pidState, activeMask)
    if ~any(activeMask)
        uDeg = 0.0;
        return;
    end

    e_eff = e;

    if pidState.first
        pidState.first = false;
        pidState.prevE = e_eff;
    end

    pidState.intE = pidState.intE + e_eff * dt;
    pidState.intE = max(-pid.intMax, min(pid.intMax, pidState.intE));

    de = (e_eff - pidState.prevE) / dt;
    pidState.prevE = e_eff;

    uDeg = pid.Kp * e_eff + pid.Ki * pidState.intE + pid.Kd * de;
    uDeg = max(-pid.outMax, min(pid.outMax, uDeg));
end

function [cmd, speedCmd, angleCmd] = singleState_control(e, steerOffsetDeg, activeMask, params)
    cmd      = uint8(hex2dec('F1'));
    speedCmd = uint16(params.baseSpeed);      % 11-bit value
    angleCmd = uint16(params.centerAngle);

    minA = double(params.minAngle);
    maxA = double(params.maxAngle);

    if abs(e) < params.deadbandPos || ~any(activeMask)
        steerOffsetDeg = 0;
    end

    angleDouble = double(params.centerAngle) + steerOffsetDeg;
    angleDouble = max(minA, min(maxA, angleDouble));
    angleCmd    = uint16(angleDouble);
end

%% ========================================================================
%% Placeholder: you MUST replace this with your real sensor frame parser
%% ========================================================================

function [ok, lineRaw, ultra, mpu, encCount, encSpeed] = helper_recvSensorFrame_placeholder(conn, timeout) %#ok<INUSD>
% Không đủ dữ liệu để xác minh format frame sensor trong low_level/main.cpp hiện tại.
% Thay thế hàm này bằng parser đúng theo frame ESP32 đang gửi về.
    ok = false;
    lineRaw = zeros(1,5,'uint16');
    ultra = [];
    mpu = [];
    encCount = [];
    encSpeed = [];
    pause(timeout);
end
