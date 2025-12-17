% File: +capstone/+script/+test/test_speed_steps_idle_operation.m
function test_speed_steps_idle_operation()
%TEST_SPEED_STEPS_IDLE_OPERATION
% Test sequence:
%   1) IDLE mode:
%      - Speed steps: 30%, 60%, 100% (3 s each)
%      - Steering untouched
%   2) OPERATION mode:
%      - Speed steps: 30%, 60%, 100% (3 s each)
%      - Steering angles: 55°, 75°, 105° (cycled with speed steps)
%   3) Return to IDLE
%   4) Plot speed command (%) and encoder feedback vs time (OPERATION only)
%
% ASSUMPTIONS (must match your firmware / Network):
%   - Network.opDriveStepPercent(percent, timeout_s, ...) exists
%   - Network.opSendControl(speed_percent, angle_deg) exists
%   - Sensor frame contains encoder count and encoder speed
%     parsed by recvSensorFrame()
%
% NOTE
%   If your recvSensorFrame() signature differs, adjust parsing section only.

    %% ===== USER PARAMETERS =====
    btName    = "Behind the scream122";
    btChannel = 1;

    speedSteps = [30, 60, 100];   % percent
    stepTime   = 3.0;             % seconds

    steerSeq   = [55, 75, 105];   % degrees (OPERATION only)

    samplePause = 0.01;           % host polling period (s)

    %% ===== CONNECT =====
    net = capstone.io.Network();
    net.connectBluetooth(btName, btChannel);

    proto = capstone.io.Protocol.constants();

    %% ===================== IDLE TEST =====================
    fprintf("[TEST] IDLE speed steps\n");
    net.sendByte(proto.CMD_MODE_IDLE);
    pause(0.2);

    for i = 1:numel(speedSteps)
        fprintf("[IDLE] speed=%d%% for %.1fs\n", speedSteps(i), stepTime);
        % Direct drive step in IDLE (no steering)
        net.opDriveStepPercent(speedSteps(i), stepTime, ...
            "enterOp", false, "exitIdle", false);
        pause(0.5);
    end

    % Ensure stop
    net.sendByte(proto.CMD_DRIVE_STOP);
    pause(0.5);

    %% ===================== OPERATION TEST =====================
    fprintf("[TEST] OPERATION speed + steering steps\n");

    ok = net.opEnter(2.0);
    if ~ok
        warning("No ACK when entering OPERATION");
    end

    % ---- Data logging (OPERATION only) ----
    tLog   = [];
    spdCmd = [];
    encCnt = [];
    encSpd = [];

    t0 = tic;

    for i = 1:numel(speedSteps)
        sp = speedSteps(i);
        ang = steerSeq(i);

        fprintf("[OP] speed=%d%% angle=%d deg for %.1fs\n", sp, ang, stepTime);

        tStep = tic;
        while toc(tStep) < stepTime
            % Send OP control continuously (safe for ESP32)
            net.opSendControl(sp, ang);

            % ----- Receive sensor frame -----
            % EXPECTED signature (adjust if yours differs):
            % [ok, lineRaw, ultra, mpu, encCount, encSpeed]
            [rxOk, ~, ~, ~, encCount, encSpeed] = net.recvSensorFrame(0.05);

            if rxOk
                tLog(end+1)   = toc(t0);          %#ok<AGROW>
                spdCmd(end+1) = sp;               %#ok<AGROW>
                encCnt(end+1) = double(encCount); %#ok<AGROW>
                encSpd(end+1) = double(encSpeed); %#ok<AGROW>
            end

            pause(samplePause);
        end
    end

    %% ===== STOP & RETURN TO IDLE =====
    net.opSendStop();
    pause(0.2);
    net.opExitToIdle();

    net.disconnect();

    %% ===================== PLOT =====================
    if isempty(tLog)
        warning("No OPERATION data logged. Nothing to plot.");
        return;
    end

    figure("Name","Speed step test (OPERATION)");
    tiledlayout(2,1,"TileSpacing","compact");

    nexttile;
    plot(tLog, spdCmd, "LineWidth",1.5);
    grid on;
    ylabel("Speed command [%]");
    title("Commanded speed (OPERATION)");

    nexttile;
    yyaxis left;
    plot(tLog, encCnt, "LineWidth",1.2);
    ylabel("Encoder count");

    yyaxis right;
    plot(tLog, encSpd, "LineWidth",1.2);
    ylabel("Encoder speed");

    xlabel("Time [s]");
    grid on;
    title("Encoder feedback (OPERATION)");

    fprintf("[TEST] Finished. Plots generated.\n");
end
