function PID_multiRoad_singleSpeed()
%PID_multiRoad_singleSpeed
% Copy ~90% from PID_program_singleState, wrapped by season/road.
% - Steering: PID (same normalize->error->pid->angle pipeline)
% - Drive: single constant speed for all roads (11-bit, 0..2047)
% - Road switching condition:
%     (progress >= 90% of segment encoder_count_seg) AND (angleCmd hits a fixed switch angle)
% - Prints Windows console logs for debug.
%
% Stop:
%   - Ctrl+C
%   - or create stop flag:
%       type nul > "%TEMP%\capstone_stop.flag"

    %% ===== 0) CONFIG (season / roads / robot) =====
    robot = capstone.config.cfg_robot();
    roads = capstone.config.cfg_roads(robot);
    season = capstone.config.cfg_season(robot, roads, "Behind the scream");

    % Connection
    btName    = season.setup.comm.btName;
    btChannel = season.setup.comm.channel;
    if isnan(btChannel); btChannel = 1; end

    % Stop flag (Windows CMD)
    stopFlag = fullfile(getenv("TEMP"), "capstone_stop.flag");
    if exist(stopFlag,'file'); delete(stopFlag); end

    % -------- Single speed (11-bit) for entire run --------
    % Put this into season to avoid adding new structs elsewhere
    if ~isfield(season.setup,'drive'); season.setup.drive = struct(); end
    if ~isfield(season.setup.drive,'baseSpeed_u11')
        season.setup.drive.baseSpeed_u11 = uint16(800); % EDIT: 0..2047
    end

    % -------- Start road --------
    if ~isfield(season.runtime,'road_idx') || season.runtime.road_idx < 1
        season.runtime.road_idx = uint16(findRoadIdxById(season.setup.roads, "L1"));
    end
    season.runtime.road_id = season.setup.roads(season.runtime.road_idx).id;

    % -------- Line calibration from season.setup.calib.line --------
    % (fallbacks keep compatibility if fields not filled)
    calib = season.setup.calib.line;

    % Ensure these exist (no new structs)
    if ~isfield(calib,'weights');      calib.weights = [-2 -1 0 1 2]; end
    if ~isfield(calib,'min_u8');       calib.min_u8 = zeros(1,5); end
    if ~isfield(calib,'max_u8');       calib.max_u8 = 255*ones(1,5); end
    if ~isfield(calib,'th_detect');    calib.th_detect = NaN; end
    if ~isfield(calib,'err_clip');     calib.err_clip = 2.0; end
    if ~isfield(calib,'invert');       calib.invert = false; end

    % Convert "white/black" to offset/scale if you stored them that way
    % If you keep only min/max, you can still tune later.
    if isfield(calib,'white') && isfield(calib,'black')
        lineOffset = double(calib.white);
        lineScale  = 1 ./ (double(calib.black) - double(calib.white));
    else
        % Placeholder (not enough data to auto-derive real calibration)
        lineOffset = double(calib.min_u8);
        den = double(calib.max_u8) - double(calib.min_u8);
        den(den==0) = 1;
        lineScale = 1 ./ den;
    end

    if isfield(calib,'threshold')
        lineThreshold = double(calib.threshold);
    else
        lineThreshold = 0.5*ones(1,5);
    end

    positions = double(calib.weights); % same pipeline: weighted COM

    %% ===== 1) CONNECT NETWORK =====
    net = Network();
    net.connectBluetooth(btName, btChannel);
    fprintf("[HOST] Connected (BT %s ch %d)\n", btName, btChannel);

    %% ===== 2) ENTER OPERATION MODE (0xFF + wait ACK 0x20) =====
    net.sendByte(hex2dec('FF'));
    if ~net.waitAck(2.0)
        fprintf("[HOST] WARN: No ACK (0x20) after 0xFF\n");
    else
        fprintf("[HOST] ACK (0x20). Robot in OPERATION.\n");
    end

    %% ===== 3) PID STATE VARIABLES (kept same) =====
    pidState.intE   = 0.0;
    pidState.prevE  = 0.0;
    pidState.first  = true;

    tPrev = tic;

    % Road progress tracking (encoder-based)
    segStartCount = int32(0);
    segCountValid = false;

    fprintf("[HOST] Start PID_multiRoad_singleSpeed. Ctrl+C or stopFlag.\n");

    %% ===== MAIN LOOP =====
    try
        while true
            if exist(stopFlag,'file')
                fprintf("[HOST] stopFlag detected -> exit loop.\n");
                break;
            end

            % --- time step ---
            dt = toc(tPrev);
            if dt <= 0; dt = 1e-3; end
            tPrev = tic;

            % --- Receive sensor frame (same as old code call) ---
            [ok, lineRaw, ultra, mpu, encCount, encSpeed] = net.recvSensorFrame(1.0); %#ok<ASGLU>
            if ~ok
                fprintf("[HOST] Timeout waiting sensor frame...\n");
                continue;
            end

            encCount = int32(encCount);

            % Initialize segment start encoder at first valid frame of segment
            if ~segCountValid
                segStartCount = encCount;
                segCountValid = true;
                fprintf("[HOST] SEG START: road=%s, encStart=%d\n", season.runtime.road_id, segStartCount);
            end

            % --- 1) Normalize line & mask (same pipeline) ---
            [lineNorm, activeMask] = computeNormalize_calib(lineRaw, lineOffset, lineScale, lineThreshold, calib);

            % --- 2) Compute line error (same pipeline) ---
            e = computeError(lineNorm, activeMask, positions, calib);

            % --- 3) PID steering update (same pipeline) ---
            road = season.setup.roads(season.runtime.road_idx);
            pid  = road.tune.steer.pid; % expects fields Kp/Ki/Kd/intMax/outMax OR equivalent

            % Backward compatibility: if you store pid as Kp Ki Kd + I_min/I_max + outMax
            pid = pidCompat(pid);

            [steerOffsetDeg, pidState] = pidUpdateSteering(e, dt, pid, pidState, activeMask);

            % --- 4) Compute command (single-speed, per-road centerAngle/min/max) ---
            [cmd, speedCmd_u11, angleCmd_deg] = multiRoad_control(e, steerOffsetDeg, activeMask, season, road);

            % --- 5) Send command (11-bit speed) ---
            % Network class must be updated to send uint16 speed (11-bit used) + uint16 angle.
            % Here we call the same API name; you update inside Network.sendControl().
            net.sendControl(speedCmd_u11, uint16(angleCmd_deg), cmd);

            % --- Debug print ---
            prog = segmentProgress(encCount, segStartCount, road.motion.encoder_count_seg);
            fprintf("[DBG] road=%s e=%.3f u=%.2f ang=%.1f sp=%d enc=%d prog=%.2f mask=%s\n", ...
                season.runtime.road_id, e, steerOffsetDeg, angleCmd_deg, speedCmd_u11, encCount, prog, mat2str(activeMask));

            % --- Road switching: (>=90% distance) AND (angle hits fixed switch angle) ---
            [doSwitch, reason] = shouldSwitchRoad(encCount, segStartCount, angleCmd_deg, road);
            if doSwitch
                [season, segStartCount, segCountValid] = switchToNextRoad(season, encCount, reason);
                % reset PID state per road (optional; keep same behavior as single-state if you want)
                pidState.intE = 0.0;
                pidState.first = true;
            end
        end

    catch ME
        fprintf("[HOST] ERROR: %s\n", ME.message);
    end

    %% ===== CLEAN UP =====
    try
        % stop
        net.sendControl(uint16(0), uint16(robot.hw.steer.center_deg), uint8(hex2dec('F0')));
        % back to IDLE
        net.sendByte(hex2dec('FE'));
    catch
    end
    net.disconnect();
    fprintf("[HOST] Finished.\n");
end

%% =========================== HELPERS =====================================

function pid = pidCompat(pid)
% Ensure fields: Kp Ki Kd intMax outMax
    if ~isfield(pid,'Kp'); pid.Kp = 0; end
    if ~isfield(pid,'Ki'); pid.Ki = 0; end
    if ~isfield(pid,'Kd'); pid.Kd = 0; end

    if isfield(pid,'intMax')
        % ok
    elseif isfield(pid,'I_max')
        pid.intMax = abs(pid.I_max);
    else
        pid.intMax = 10.0;
    end

    if isfield(pid,'outMax')
        % ok
    else
        pid.outMax = 45.0;
    end
end

function idx = findRoadIdxById(roads, id)
    idx = find([roads.id] == string(id), 1, 'first');
    if isempty(idx); idx = 1; end
end

function [lineNorm, activeMask] = computeNormalize_calib(lineRaw, offset, scale, threshold, calib)
% Same behavior as your stable version, with calib clamp/invert support
    r = double(lineRaw(:)).';
    lineNorm = (r - offset) .* scale;
    lineNorm = max(0, min(1, lineNorm));

    if isfield(calib,'invert') && calib.invert
        lineNorm = 1 - lineNorm;
    end

    activeMask = lineNorm > threshold;
end

function e = computeError(lineNorm, activeMask, positions, calib)
% Same as your stable version; with optional error clip
    w = lineNorm;
    w(~activeMask) = 0;

    if all(w == 0)
        e = 0;
        return;
    end

    pos = sum(positions .* w) / sum(w);
    e = -pos;

    if isfield(calib,'err_clip') && ~isnan(calib.err_clip)
        e = max(-calib.err_clip, min(calib.err_clip, e));
    end
end

function [uDeg, pidState] = pidUpdateSteering(e, dt, pid, pidState, activeMask)
% Same as your stable version
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

function [cmd, speedCmd_u11, angleCmd_deg] = multiRoad_control(e, steerOffsetDeg, activeMask, season, road)
% Equivalent to singleState_control(), but:
% - speed is fixed from season.setup.drive.baseSpeed_u11
% - center angle can be per-road (road.tune.steer.center_deg), else fallback robot center

    robot = season.setup.robot;

    cmd = uint8(hex2dec('F1'));

    speedCmd_u11 = season.setup.drive.baseSpeed_u11;
    speedCmd_u11 = uint16(min(max(speedCmd_u11, robot.limits.drive.cmd_min), robot.limits.drive.cmd_max));

    % per-road center angle (deg)
    if isfield(road.tune,'steer') && isfield(road.tune.steer,'center_deg') && ~isnan(road.tune.steer.center_deg)
        center = double(road.tune.steer.center_deg);
    else
        center = double(robot.hw.steer.center_deg);
    end

    minA = double(robot.hw.steer.min_deg);
    maxA = double(robot.hw.steer.max_deg);

    % per-road deadband on position error (optional)
    db = NaN;
    if isfield(road.tune.steer,'deadbandPos'); db = road.tune.steer.deadbandPos; end
    if isnan(db) && isfield(road.tune.steer,'pid') && isfield(road.tune.steer.pid,'deadband_err')
        db = road.tune.steer.pid.deadband_err;
    end
    if isnan(db); db = 0.10; end

    if abs(e) < db || ~any(activeMask)
        steerOffsetDeg = 0;
    end

    angleDouble = center + steerOffsetDeg;
    angleDouble = max(minA, min(maxA, angleDouble));
    angleCmd_deg = angleDouble;
end

function p = segmentProgress(encCount, segStartCount, encTotal)
% Returns 0..1 if encTotal available, else NaN
    if isempty(encTotal) || isnan(encTotal) || encTotal <= 0
        p = NaN;
        return;
    end
    d = double(encCount - segStartCount);
    p = d / double(encTotal);
end

function [doSwitch, reason] = shouldSwitchRoad(encCount, segStartCount, angleCmd_deg, road)
% Condition: progress>=0.90 AND angle hits fixed switch angle
% road.tune.steer.switchAngle_deg must be set by you per road (fixed angle).
    doSwitch = false;
    reason = "";

    if ~isfield(road,'motion') || ~isfield(road.motion,'encoder_count_seg')
        return;
    end
    encTotal = road.motion.encoder_count_seg;
    if isnan(encTotal) || encTotal <= 0
        return;
    end

    prog = double(encCount - segStartCount) / double(encTotal);
    if prog < 0.90
        return;
    end

    % Fixed switch angle per road (deg)
    if isfield(road,'tune') && isfield(road.tune,'steer') && isfield(road.tune.steer,'switchAngle_deg')
        swA = road.tune.steer.switchAngle_deg;
    else
        swA = NaN;
    end
    if isnan(swA)
        return; % not configured => no switching by angle
    end

    % "hits" with tolerance (deg)
    tol = 0.5;
    if isfield(road.tune.steer,'switchAngle_tol_deg') && ~isnan(road.tune.steer.switchAngle_tol_deg)
        tol = road.tune.steer.switchAngle_tol_deg;
    end

    if abs(double(angleCmd_deg) - double(swA)) <= tol
        doSwitch = true;
        reason = sprintf("prog>=0.90 (%.2f) AND angle~switch (%.1f≈%.1f)", prog, angleCmd_deg, swA);
    end
end

function [season, segStartCount, segCountValid] = switchToNextRoad(season, encCount, reason)
% Switch to next_id_default; print debug
    roads = season.setup.roads;
    cur = roads(season.runtime.road_idx);

    nextId = "";
    if isfield(cur,'switch') && isfield(cur.switch,'next_id_default')
        nextId = cur.switch.next_id_default;
    end
    if strlength(nextId)==0
        fprintf("[HOST] SWITCH blocked: no next_id_default. reason=%s\n", reason);
        return;
    end

    nextIdx = findRoadIdxById(roads, nextId);
    prevId  = season.runtime.road_id;

    season.runtime.road_idx = uint16(nextIdx);
    season.runtime.road_id  = roads(nextIdx).id;

    fprintf("[HOST] SWITCH: %s -> %s | enc=%d | reason: %s\n", prevId, season.runtime.road_id, int32(encCount), reason);

    % reset segment tracking
    segStartCount = int32(encCount);
    segCountValid = true;
end
