function capstone_test_idle_all(deviceName, channelID)
% CAPSTONE_TEST_IDLE_ALL  Test toàn bộ SYSTEM + IDLE CMD với robot ESP32.
%
%   capstone_test_idle_all()                      % dùng tên mặc định
%   capstone_test_idle_all("Behind the scream",1) % chỉ định rõ

    if nargin < 1, deviceName = "Behind the scream122"; end
    if nargin < 2, channelID = 1; end

    proto = capstone.io.Protocol.constants();
    net   = capstone.io.Network();

    fprintf("=== CONNECT ===\n");
    fprintf("Connecting to %s (channel %d)...\n", deviceName, channelID);
    net.connectBluetooth(deviceName, channelID);
    pause(1.0);

    %% 1) SYSTEM COMMANDS (MODE / PING / EMERGENCY)
    fprintf("\n=== 1) SYSTEM COMMANDS ===\n");

    % 1.1: Force IDLE
    fprintf("[SYS] CMD_MODE_IDLE (0x%02X)\n", proto.CMD_MODE_IDLE);
    net.sendByte(proto.CMD_MODE_IDLE);
    ok = net.waitAck(1.0);
    fprintf("    ACK(IDLE) = %d\n", ok);
    pause(0.2);

    % 1.2: Enter OPERATION
    fprintf("[SYS] CMD_MODE_OP (0x%02X)\n", proto.CMD_MODE_OP);
    net.sendByte(proto.CMD_MODE_OP);
    ok = net.waitAck(1.0);
    fprintf("    ACK(OP) = %d\n", ok);
    pause(0.2);

    % 1.3: PING MODE – ESP32 gửi back 1 byte mode, rồi ACK
    fprintf("[SYS] CMD_GLOBAL_PING_MODE (0x%02X)\n", proto.CMD_GLOBAL_PING_MODE);
    net.sendByte(proto.CMD_GLOBAL_PING_MODE);
    modeByte = net.readFrameBlocking(1, 1.0);  % đọc 1 byte mode
    if isempty(modeByte)
        fprintf("    MODE byte: <none>\n");
    else
        fprintf("    MODE byte: 0x%02X\n", modeByte(1));
    end
    ok = net.waitAck(1.0);
    fprintf("    ACK(PING_MODE) = %d\n", ok);
    pause(0.2);

    % 1.4: EMERGENCY STOP -> về IDLE
    fprintf("[SYS] CMD_GLOBAL_EMERGENCY_STOP (0x%02X)\n", proto.CMD_GLOBAL_EMERGENCY_STOP);
    net.sendByte(proto.CMD_GLOBAL_EMERGENCY_STOP);
    ok = net.waitAck(1.0);
    fprintf("    ACK(EMERGENCY_STOP) = %d\n", ok);

    % Đặt lại IDLE cho an toàn
    net.sendByte(proto.CMD_MODE_IDLE);
    net.waitAck(1.0);
    pause(0.5);


    %% 2) IDLE SENSORS
    fprintf("\n=== 2) IDLE SENSORS ===\n");

    % 2.1: READ LINE (5 byte + ACK)
    fprintf("[IDLE] CMD_IDLE_SENSOR_LINE_READ (0x%02X)\n", proto.CMD_IDLE_SENSOR_LINE_READ);
    net.sendByte(proto.CMD_IDLE_SENSOR_LINE_READ);
    lineFrame = net.readFrameBlocking(5, 1.0);
    fprintf("    LINE bytes: ");
    disp(lineFrame(:).');
    ok = net.waitAck(1.0);
    fprintf("    ACK(LINE_READ) = %d\n", ok);
    pause(0.2);

    % 2.2: ULTRA KICK (ACK-only)
    fprintf("[IDLE] CMD_IDLE_SENSOR_ULTRA_KICK (0x%02X)\n", proto.CMD_IDLE_SENSOR_ULTRA_KICK);
    net.sendByte(proto.CMD_IDLE_SENSOR_ULTRA_KICK);
    ok = net.waitAck(1.0);
    fprintf("    ACK(ULTRA_KICK) = %d\n", ok);
    pause(0.2);

    % 2.3: ULTRA READ (2 byte + ACK)
    fprintf("[IDLE] CMD_IDLE_SENSOR_ULTRA_READ (0x%02X)\n", proto.CMD_IDLE_SENSOR_ULTRA_READ);
    net.sendByte(proto.CMD_IDLE_SENSOR_ULTRA_READ);
    ultraFrame = net.readFrameBlocking(2, 1.0);
    fprintf("    ULTRA bytes: ");
    disp(ultraFrame(:).');
    ok = net.waitAck(1.0);
    fprintf("    ACK(ULTRA_READ) = %d\n", ok);
    pause(0.2);

    % 2.4: ENCODER ENABLE / DISABLE (ACK-only)
    fprintf("[IDLE] CMD_IDLE_ENCODER_ENABLE (0x%02X)\n", proto.CMD_IDLE_ENCODER_ENABLE);
    net.sendByte(proto.CMD_IDLE_ENCODER_ENABLE);
    ok = net.waitAck(1.0);
    fprintf("    ACK(ENC_ENABLE) = %d\n", ok);
    pause(0.2);

    fprintf("[IDLE] CMD_IDLE_ENCODER_DISABLE (0x%02X)\n", proto.CMD_IDLE_ENCODER_DISABLE);
    net.sendByte(proto.CMD_IDLE_ENCODER_DISABLE);
    ok = net.waitAck(1.0);
    fprintf("    ACK(ENC_DISABLE) = %d\n", ok);


    %% 3) IDLE ACTUATORS: MOTOR + SERVO
    fprintf("\n=== 3) IDLE ACTUATORS ===\n");

    % 3.1: MOTOR ENABLE / DISABLE
    fprintf("[IDLE] CMD_IDLE_ACTUATOR_MOTOR_ENABLE (0x%02X)\n", proto.CMD_IDLE_ACTUATOR_MOTOR_ENABLE);
    net.sendByte(proto.CMD_IDLE_ACTUATOR_MOTOR_ENABLE);
    ok = net.waitAck(1.0);
    fprintf("    ACK(MOTOR_ENABLE) = %d\n", ok);
    pause(0.2);

    fprintf("[IDLE] CMD_IDLE_ACTUATOR_MOTOR_DISABLE (0x%02X)\n", proto.CMD_IDLE_ACTUATOR_MOTOR_DISABLE);
    net.sendByte(proto.CMD_IDLE_ACTUATOR_MOTOR_DISABLE);
    ok = net.waitAck(1.0);
    fprintf("    ACK(MOTOR_DISABLE) = %d\n", ok);
    pause(0.2);

    % 3.2: MOTOR PWM FWD/BWD (u16 LE) – debug
    dutyTest = uint16(200); % khoảng 10% PWM11
    fprintf("[IDLE] CMD_IDLE_ACTUATOR_MOTOR_PWM_FWD (0x%02X), duty=%d\n", proto.CMD_IDLE_ACTUATOR_MOTOR_PWM_FWD, dutyTest);
    sendU16LE(net, proto.CMD_IDLE_ACTUATOR_MOTOR_PWM_FWD, dutyTest);
    ok = net.waitAck(1.0);
    fprintf("    ACK(MOTOR_PWM_FWD) = %d\n", ok);
    pause(0.5);

    fprintf("[IDLE] CMD_IDLE_ACTUATOR_MOTOR_PWM_BWD (0x%02X), duty=%d\n", proto.CMD_IDLE_ACTUATOR_MOTOR_PWM_BWD, dutyTest);
    sendU16LE(net, proto.CMD_IDLE_ACTUATOR_MOTOR_PWM_BWD, dutyTest);
    ok = net.waitAck(1.0);
    fprintf("    ACK(MOTOR_PWM_BWD) = %d\n", ok);
    pause(0.5);

    % 3.3: MOTOR SPD FWD/BWD (u16 LE) – dùng như speed_cmd
    spdCmd = uint16(500);
    fprintf("[IDLE] CMD_IDLE_ACTUATOR_MOTOR_SPD_FWD (0x%02X), spd=%d\n", proto.CMD_IDLE_ACTUATOR_MOTOR_SPD_FWD, spdCmd);
    sendU16LE(net, proto.CMD_IDLE_ACTUATOR_MOTOR_SPD_FWD, spdCmd);
    ok = net.waitAck(1.0);
    fprintf("    ACK(MOTOR_SPD_FWD) = %d\n", ok);
    pause(0.5);

    fprintf("[IDLE] CMD_IDLE_ACTUATOR_MOTOR_SPD_BWD (0x%02X), spd=%d\n", proto.CMD_IDLE_ACTUATOR_MOTOR_SPD_BWD, spdCmd);
    sendU16LE(net, proto.CMD_IDLE_ACTUATOR_MOTOR_SPD_BWD, spdCmd);
    ok = net.waitAck(1.0);
    fprintf("    ACK(MOTOR_SPD_BWD) = %d\n", ok);
    pause(0.5);

    % 3.4: MOTOR STOP
    fprintf("[IDLE] CMD_IDLE_ACTUATOR_MOTOR_STOP (0x%02X)\n", proto.CMD_IDLE_ACTUATOR_MOTOR_STOP);
    net.sendByte(proto.CMD_IDLE_ACTUATOR_MOTOR_STOP);
    ok = net.waitAck(1.0);
    fprintf("    ACK(MOTOR_STOP) = %d\n", ok);

    % 3.5: SERVO ENABLE / CENTER / WRITE / READ / DISABLE
    fprintf("[IDLE] CMD_IDLE_ACTUATOR_SERVO_ENABLE (0x%02X)\n", proto.CMD_IDLE_ACTUATOR_SERVO_ENABLE);
    net.sendByte(proto.CMD_IDLE_ACTUATOR_SERVO_ENABLE);
    ok = net.waitAck(1.0);
    fprintf("    ACK(SERVO_ENABLE) = %d\n", ok);
    pause(0.2);

    fprintf("[IDLE] CMD_IDLE_ACTUATOR_SERVO_WRITE_CENTER (0x%02X)\n", proto.CMD_IDLE_ACTUATOR_SERVO_WRITE_CENTER);
    net.sendByte(proto.CMD_IDLE_ACTUATOR_SERVO_WRITE_CENTER);
    ok = net.waitAck(1.0);
    fprintf("    ACK(SERVO_CENTER) = %d\n", ok);
    pause(0.5);

    testAngle = uint16(75); % trong khoảng 55–105 deg
    fprintf("[IDLE] CMD_IDLE_ACTUATOR_SERVO_WRITE (0x%02X), angle=%d\n", proto.CMD_IDLE_ACTUATOR_SERVO_WRITE, testAngle);
    sendU16LE(net, proto.CMD_IDLE_ACTUATOR_SERVO_WRITE, testAngle);
    ok = net.waitAck(1.0);
    fprintf("    ACK(SERVO_WRITE) = %d\n", ok);
    pause(0.5);

    fprintf("[IDLE] CMD_IDLE_ACTUATOR_SERVO_READ (0x%02X)\n", proto.CMD_IDLE_ACTUATOR_SERVO_READ);
    net.sendByte(proto.CMD_IDLE_ACTUATOR_SERVO_READ);
    % hiện tại firmware chưa gửi data, chỉ ACK → không gọi readFrameBlocking
    ok = net.waitAck(1.0);
    fprintf("    ACK(SERVO_READ) = %d\n", ok);

    fprintf("[IDLE] CMD_IDLE_ACTUATOR_SERVO_DISABLE (0x%02X)\n", proto.CMD_IDLE_ACTUATOR_SERVO_DISABLE);
    net.sendByte(proto.CMD_IDLE_ACTUATOR_SERVO_DISABLE);
    ok = net.waitAck(1.0);
    fprintf("    ACK(SERVO_DISABLE) = %d\n", ok);


    %% 4) IDLE SET PARAMS (payload 30 bytes)
    fprintf("\n=== 4) IDLE SET PARAMS (30 bytes) ===\n");

    % 4.1: LINE PARAMS
    % byte 0 : uint8 lineSample
    buf = zeros(30,1,'uint8');
    buf(1) = uint8(4);
    fprintf("[IDLE] CMD_IDLE_SET_LINE_PARAMS (0x%02X), lineSample=%d\n", proto.CMD_IDLE_SET_LINE_PARAMS, buf(1));
    sendBytes30(net, proto.CMD_IDLE_SET_LINE_PARAMS, buf);
    ok = net.waitAck(1.0);
    fprintf("    ACK(SET_LINE_PARAMS) = %d\n", ok);

    % 4.2: MPU PARAMS (offsets = 0)
    buf = zeros(30,1,'uint8');
    fprintf("[IDLE] CMD_IDLE_SET_MPU_PARAMS (0x%02X)\n", proto.CMD_IDLE_SET_MPU_PARAMS);
    sendBytes30(net, proto.CMD_IDLE_SET_MPU_PARAMS, buf);
    ok = net.waitAck(1.0);
    fprintf("    ACK(SET_MPU_PARAMS) = %d\n", ok);

    % 4.3: ULTRA PARAMS (trigger period 60 ms)
    trigMs = uint16(60);
    buf = zeros(30,1,'uint8');
    buf(1:2) = typecast(trigMs,'uint8');
    fprintf("[IDLE] CMD_IDLE_SET_ULTRA_PARAMS (0x%02X), trigMs=%d\n", proto.CMD_IDLE_SET_ULTRA_PARAMS, trigMs);
    sendBytes30(net, proto.CMD_IDLE_SET_ULTRA_PARAMS, buf);
    ok = net.waitAck(1.0);
    fprintf("    ACK(SET_ULTRA_PARAMS) = %d\n", ok);

    % 4.4: MOTOR PARAMS (deadband 80)
    deadband = uint16(80);
    buf = zeros(30,1,'uint8');
    buf(1:2) = typecast(deadband,'uint8');
    fprintf("[IDLE] CMD_IDLE_SET_MOTOR_PARAMS (0x%02X), deadband=%d\n", proto.CMD_IDLE_SET_MOTOR_PARAMS, deadband);
    sendBytes30(net, proto.CMD_IDLE_SET_MOTOR_PARAMS, buf);
    ok = net.waitAck(1.0);
    fprintf("    ACK(SET_MOTOR_PARAMS) = %d\n", ok);

    % 4.5: SERVO PARAMS (giá trị demo, firmware hiện tại chưa dùng runtime)
    buf = zeros(30,1,'uint8');
    buf(1:2) = typecast(uint16(55),'uint8'); % min
    buf(3:4) = typecast(uint16(105),'uint8');% max
    buf(5:6) = typecast(uint16(75),'uint8'); % mid
    fprintf("[IDLE] CMD_IDLE_SET_SERVO_PARAMS (0x%02X)\n", proto.CMD_IDLE_SET_SERVO_PARAMS);
    sendBytes30(net, proto.CMD_IDLE_SET_SERVO_PARAMS, buf);
    ok = net.waitAck(1.0);
    fprintf("    ACK(SET_SERVO_PARAMS) = %d\n", ok);

    % 4.6: PID PARAMS (demo)
    Kp      = single(10.0);
    Ki      = single(2.0);
    Kd      = single(0.0);
    windup  = single(1000.0);
    dutyMin = uint16(0);
    dutyMax = uint16(2047);
    slew    = single(500.0);
    dbPID   = uint16(80);

    buf = zeros(30,1,'uint8');
    buf(1:4)   = typecast(Kp,'uint8');
    buf(5:8)   = typecast(Ki,'uint8');
    buf(9:12)  = typecast(Kd,'uint8');
    buf(13:16) = typecast(windup,'uint8');
    buf(17:18) = typecast(dutyMin,'uint8');
    buf(19:20) = typecast(dutyMax,'uint8');
    buf(21:24) = typecast(slew,'uint8');
    buf(25:26) = typecast(dbPID,'uint8');

    fprintf("[IDLE] CMD_IDLE_SET_PID_PARAMS (0x%02X)\n", proto.CMD_IDLE_SET_PID_PARAMS);
    sendBytes30(net, proto.CMD_IDLE_SET_PID_PARAMS, buf);
    ok = net.waitAck(1.0);
    fprintf("    ACK(SET_PID_PARAMS) = %d\n", ok);


    %% 5) FINAL STOP
    fprintf("\n=== 5) FINAL STOP ===\n");
    net.sendByte(proto.CMD_IDLE_ACTUATOR_MOTOR_STOP);
    net.waitAck(1.0);
    net.sendByte(proto.CMD_MODE_IDLE);
    net.waitAck(1.0);
    fprintf("Done.\n");
end

% ===== HELPER: gửi cmd + u16 (LE) =====
function sendU16LE(net, cmd_u8, value_u16)
    v  = uint16(value_u16);
    b  = typecast(v,'uint8');  % little-endian trên MATLAB
    net.sendBytes(uint8([uint8(cmd_u8); b(:)]));
end

% ===== HELPER: gửi cmd + 30 bytes =====
function sendBytes30(net, cmd_u8, payload30_u8)
    b = uint8(payload30_u8(:));
    if numel(b) ~= 30
        error("payload30_u8 must have 30 elements");
    end
    net.sendBytes(uint8([uint8(cmd_u8); b]));
end
