function capstone_test_speed_settling_15s(deviceName, channelID, v_ref_hz)
% Test closed-loop speed for 15s and check steady-state convergence
% REQUIREMENT:
%  - ESP32 sends OPERATION telemetry frame = 22 bytes
%  - speed_i16 is LAST 2 BYTES (Hz * 100)

    if nargin < 1, deviceName = "Behind the scream"; end
    if nargin < 2, channelID  = 1; end
    if nargin < 3, v_ref_hz   = 5.0; end   % wheel Hz

    RUN_S = 15;
    SPEED_SCALE = 100.0;   % i16 = Hz*100

    proto = capstone.io.Protocol.constants();
    net   = capstone.io.Network();

    fprintf("=== CONNECT ===\n");
    net.connectBluetooth(deviceName, channelID);
    pause(1);

    %% IDLE + encoder
    net.sendByte(proto.CMD_MODE_IDLE);
    net.waitAck(1.0);
    net.sendByte(proto.CMD_IDLE_ENCODER_ENABLE);
    net.waitAck(1.0);
    pause(0.5);

    %% OPERATION
    net.sendByte(proto.CMD_MODE_OP);
    net.waitAck(1.0);
    pause(0.3);

    %% Send speed command
    fprintf("=== RUN CLOSED LOOP: %.2f Hz ===\n", v_ref_hz);
    if v_ref_hz >= 0
        net.opSendSpeedFwd(single(v_ref_hz));
    else
        net.opSendSpeedBwd(single(abs(v_ref_hz)));
    end

    %% DATA ACQUISITION
    t0 = tic;
    t  = [];
    vM = [];

    while toc(t0) < RUN_S
        try
            % BLOCKING read 22 bytes
            bytes = net.readBytesExact(22, 0.2);
            if isempty(bytes), continue; end

            bytes = uint8(bytes(:));
            speed_i16 = typecast(bytes(21:22), 'int16');
            speed_hz  = double(speed_i16) / SPEED_SCALE;

            t(end+1,1)  = toc(t0); %#ok<AGROW>
            vM(end+1,1) = speed_hz;

        catch
            % timeout → skip
        end
    end

    %% Stop
    net.opSendBrake();
    pause(0.3);
    net.sendByte(proto.CMD_MODE_IDLE);
    net.waitAck(1.0);

    %% CHECK DATA
    if isempty(t)
        error("NO TELEMETRY RECEIVED. Check ESP32 sendOpTelem22() frequency.");
    end

    %% PLOT
    vR = v_ref_hz * ones(size(t));

    figure('Name','Speed settling test (15s)');
    plot(t, vR, 'k--', 'LineWidth', 1.2); hold on;
    plot(t, vM, 'b',  'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)');
    ylabel('Wheel speed (Hz)');
    title(sprintf('Closed-loop speed response (Kp = 1), ref = %.2f Hz', v_ref_hz));
    legend('ref','meas','Location','best');

    %% STEADY-STATE ANALYSIS (last 20%)
    idx = t >= 0.8 * RUN_S;
    ss_mean = mean(vM(idx));
    ss_std  = std(vM(idx));
    ss_err  = ss_mean - v_ref_hz;

    fprintf("\n=== STEADY STATE (last 20%%) ===\n");
    fprintf("Mean speed : %.3f Hz\n", ss_mean);
    fprintf("Std  speed : %.3f Hz\n", ss_std);
    fprintf("Error      : %.3f Hz\n", ss_err);

    fprintf("Samples received: %d\n", numel(t));
    fprintf("Done.\n");
end
