function capstone_test_operation_speed_pwm(deviceName, channelID)
% Test OPERATION:
%  - PWM open-loop (u16 duty)
%  - SPEED closed-loop (float32 wheel frequency [Hz])
%
%   capstone_test_operation_speed_pwm()
%   capstone_test_operation_speed_pwm("Behind the scream", 1)

    if nargin < 1, deviceName = "Behind the scream"; end
    if nargin < 2, channelID = 1; end

    proto = capstone.io.Protocol.constants();
    net   = capstone.io.Network();

    fprintf("=== CONNECT ===\n");
    net.connectBluetooth(deviceName, channelID);
    pause(1.0);

    %% 0) IDLE + encoder enable
    fprintf("\n=== IDLE + ENCODER ENABLE ===\n");
    net.sendByte(proto.CMD_MODE_IDLE);
    net.waitAck(1.0);

    net.sendByte(proto.CMD_IDLE_ENCODER_ENABLE);
    net.waitAck(1.0);

    pause(0.5);

    %% 1) ENTER OPERATION
    fprintf("\n=== ENTER OPERATION ===\n");
    net.sendByte(proto.CMD_MODE_OP);
    net.waitAck(1.0);
    pause(0.5);

    %% 2) PWM OPEN-LOOP
    fprintf("\n=== TEST PWM OPEN-LOOP ===\n");
    duty = uint16(1500);  % 0..SPEED_MAX_INPUT, ví dụ 400

    fprintf("PWM_FWD, duty=%d\n", duty);
    sendU16LE(net, proto.CMD_OP_PWM_FWD, duty);
    pause(2.0);

    fprintf("PWM_BWD, duty=%d\n", duty);
    sendU16LE(net, proto.CMD_OP_PWM_BWD, duty);
    pause(2.0);

    fprintf("BRAKE\n");
    net.opSendBrake();
    pause(1.0);

    %% 3) SPEED CLOSED-LOOP (Hz, forward)
    fprintf("\n=== TEST SPEED CLOSED-LOOP FWD (Hz) ===\n");
    v_targets_hz = [50.0, 100.0, 150.0];  % ví dụ 2 Hz, 4 Hz, 6 Hz

    for k = 1:numel(v_targets_hz)
        v = v_targets_hz(k);
        fprintf("SPD_FWD = %.2f Hz\n", v);
        net.opSendSpeedFwd(v);
        pause(3.0);
    end

    fprintf("BRAKE\n");
    net.opSendBrake();
    pause(2.0);

    %% 4) SPEED CLOSED-LOOP (Hz, backward)
    fprintf("\n=== TEST SPEED CLOSED-LOOP BWD (Hz) ===\n");
    v_targets_hz = [50.0, 100.0];

    for k = 1:numel(v_targets_hz)
        v = v_targets_hz(k);
        fprintf("SPD_BWD = %.2f Hz\n", v);
        net.opSendSpeedBwd(v);
        pause(3.0);
    end

    fprintf("BRAKE\n");
    net.opSendBrake();
    pause(2.0);

    %% 5) EXIT TO IDLE
    fprintf("\n=== EXIT TO IDLE ===\n");
    net.sendByte(proto.CMD_MODE_IDLE);
    net.waitAck(1.0);

    fprintf("Done.\n");
end

% ===== helper: cmd + u16 LE =====
function sendU16LE(net, cmd_u8, value_u16)
    v  = uint16(value_u16);
    b  = typecast(v, 'uint8');  % little-endian
    net.sendBytes(uint8([uint8(cmd_u8); b(:)]));
end
