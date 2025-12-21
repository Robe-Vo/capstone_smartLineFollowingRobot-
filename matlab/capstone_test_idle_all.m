function capstone_test_idle_all(deviceName, channelID)
% CAPSTONE_TEST_IDLE_ALL  Test toàn bộ SYSTEM + IDLE CMD với robot ESP32.
%
%   capstone_test_idle_all()                      % dùng tên "Behind the scream", kênh 1
%   capstone_test_idle_all("Behind the scream",1) % chỉ định rõ

    if nargin < 1, deviceName = "Behind the scream122"; end
    if nargin < 2, channelID = 1; end

    proto = capstone.io.Protocol.constants();
    net   = capstone.io.Network();

    fprintf("=== CONNECT ===\n");
    fprintf("Connecting to %s (channel %d)...\n", deviceName, channelID);
    net.connectBluetooth(deviceName, channelID);
    pause(1.0);

    %% 1) SYSTEM COMMANDS
    fprintf("\n=== 1) SYSTEM COMMANDS ===\n");

    % 1.1: Force IDLE, chờ ACK
    fprintf("[SYS] CMD_MODE_IDLE (0x%02X)\n", proto.CMD_MODE_IDLE);
    net.sendByte(proto.CMD_MODE_IDLE);
    ok = net.waitAck(1.0);
    fprintf("    ACK(IDLE) = %d\n", ok);

    pause(0.2);

    % 1.2: Enter OPERATION, chờ ACK
    fprintf("[SYS] CMD_MODE_OP (0x%02X)\n", proto.CMD_MODE_OP);
    net.sendByte(proto.CMD_MODE_OP);
    ok = net.waitAck(1.0);
    fprintf("    ACK(OP) = %d\n", ok);

    pause(0.2);

    % 1.3: PING MODE, đọc 1 byte mode + ACK
    fprintf("[SYS] CMD_GLOBAL_PING_MODE (0x%02X)\n", proto.CMD_GLOBAL_PING_MODE);
    net.sendByte(proto.CMD_GLOBAL_PING_MODE);

    modeByte = net.readFrameBlocking(1, 1.0);
    if isempty(modeByte)
        fprintf("    MODE byte: <none>\n");
    else
        fprintf("    MODE byte: 0x%02X\n", modeByte(1));
    end
    ok = net.waitAck(1.0);
    fprintf("    ACK(PING_MODE) = %d\n", ok);

    pause(0.2);

    % 1.4: EMERGENCY STOP
    fprintf("[SYS] CMD_GLOBAL_EMERGENCY_STOP (0x%02X)\n", proto.CMD_GLOBAL_EMERGENCY_STOP);
    net.sendByte(proto.CMD_GLOBAL_EMERGENCY_STOP);
    ok = net.waitAck(1.0);
    fprintf("    ACK(EMERGENCY_STOP) = %d\n", ok);

    % Quay lại IDLE cho an toàn
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

    % 2.2: ULTRA KICK (không cần data, chỉ ACK)
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

    % 2.4: ENCODER ENABLE / DISABLE
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

    % 3.2: MOTOR PWM FWD/BWD (payload u16 LE), giá trị test nhỏ
    dutyTest = uint16(200); % ~10% PWM11 nếu 0..2047
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

    % 3.3: MOTOR SPD FWD/BWD (payload u16 LE – dùng như test speed_cmd)
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

    % 3.5: SERVO ENABLE / DISABLE / CENTER / WRITE
    fprintf("[IDLE] CMD_IDLE_ACTUATOR_SERVO_ENABLE (0x%02X)\n", proto.CMD_IDLE_ACTUATOR_SERVO_ENABLE);
    net.sendByte(proto.CMD_IDLE_ACTUATOR_SERVO_ENABLE);
    ok = net.waitAck(1.0);
    fprintf("    ACK(SERVO_ENABLE) = %d\n", ok);

    pause(0.2);

    % servo center
    fprintf("[IDLE] CMD_IDLE_ACTUATOR_SERVO_WRITE_CENTER (0x%02X)\n", proto.CMD_IDLE_ACTUATOR_SERVO_WRITE_CENTER);
    net.sendByte(proto.CMD_IDLE_ACTUATOR_SERVO_WRITE_CENTER);
    ok = net.waitAck(1.0);
    fprintf("    ACK(SERVO_CENTER) = %d\n", ok);
    pause(0.5);

    % servo write một góc test (ví dụ 75 deg, phù hợp với firmware)
    testAngle = uint16(75);
    fprintf("[IDLE] CMD_IDLE_ACTUATOR_SERVO_WRITE (0x%02X), angle=%d\n", proto.CMD_IDLE_ACTUATOR_SERVO_WRITE, testAngle);
    sendU16LE(net, proto.CMD_IDLE_ACTUATOR_SERVO_WRITE, testAngle);
    ok = net.waitAck(1.0);
    fprintf("    ACK(SERVO_WRITE) = %d\n", ok);
    pause(0.5);

    fprintf("[IDLE] CMD_IDLE_ACTUATOR_SERVO_DISABLE (0x%02X)\n", proto.CMD_IDLE_ACTUATOR_SERVO_DISABLE);
    net.sendByte(proto.CMD_IDLE_ACTUATOR_SERVO_DISABLE);
    ok = net.waitAck(1.0);
    fprintf("    ACK(SERVO_DISABLE) = %d\n", ok);


    %% 4) IDLE SET PARAMS (payload 30 byte)
    fprintf("\n=== 4) IDLE SET PARAMS (30 bytes) ===\n");

    % 4.1: LINE PARAMS
    % Mapping đề xuất:
    %   byte 0 : uint8 lineSample (số mẫu trung bình)
    %   byte 1..29 : reserved
    lineSample = uint8(4);
    buf = zeros(30,1,'uint8');
    buf(1) = lineSample;
    fprintf("[IDLE] CMD_IDLE_SET_LINE_PARAMS (0x%02X), lineSample=%d\n", proto.CMD_IDLE_SET_LINE_PARAMS, lineSample);
    sendBytes30(net, proto.CMD_IDLE_SET_LINE_PARAMS, buf);
    ok = net.waitAck(1.0);
    fprintf("    ACK(SET_LINE_PARAMS) = %d\n", ok);

    % 4.2: MPU PARAMS
    % Mapping đề xuất:
    %   byte  0..1 : int16 accelOffsetX
    %   byte  2..3 : int16 accelOffsetY
    %   byte  4..5 : int16 accelOffsetZ
    %   byte  6..7 : int16 gyroOffsetX
    %   byte  8..9 : int16 gyroOffsetY
    %   byte 10..11: int16 gyroOffsetZ
    %   byte 12..29: reserved
    ax_off = int16(0);
    ay_off = int16(0);
    az_off = int16(0);
    gx_off = int16(0);
    gy_off = int16(0);
    gz_off = int16(0);

    buf = zeros(30,1,'uint8');
    buf(1:2)   = typecast(ax_off,'uint8');
    buf(3:4)   = typecast(ay_off,'uint8');
    buf(5:6)   = typecast(az_off,'uint8');
    buf(7:8)   = typecast(gx_off,'uint8');
    buf(9:10)  = typecast(gy_off,'uint8');
    buf(11:12) = typecast(gz_off,'uint8');

    fprintf("[IDLE] CMD_IDLE_SET_MPU_PARAMS (0x%02X)\n", proto.CMD_IDLE_SET_MPU_PARAMS);
    sendBytes30(net, proto.CMD_IDLE_SET_MPU_PARAMS, buf);
    ok = net.waitAck(1.0);
    fprintf("    ACK(SET_MPU_PARAMS) = %d\n", ok);

    % 4.3: ULTRA PARAMS
    % Mapping đề xuất:
    %   byte 0..1 : uint16 trigPeriodMs
    %   byte 2..29: reserved
    trigMs = uint16(60);
    buf = zeros(30,1,'uint8');
    buf(1:2) = typecast(trigMs,'uint8');
    fprintf("[IDLE] CMD_IDLE_SET_ULTRA_PARAMS (0x%02X), trigMs=%d\n", proto.CMD_IDLE_SET_ULTRA_PARAMS, trigMs);
    sendBytes30(net, proto.CMD_IDLE_SET_ULTRA_PARAMS, buf);
    ok = net.waitAck(1.0);
    fprintf("    ACK(SET_ULTRA_PARAMS) = %d\n", ok);

    % 4.4: MOTOR PARAMS
    % Mapping đề xuất:
    %   byte 0..1 : uint16 motorDeadbandDuty (0..2047), dùng trong PID
    %   byte 2..3 : uint16 encTwMin_us   (optional)
    %   byte 4..5 : uint16 encTwMax_us   (optional)
    %   byte 6..7 : uint16 encF_low_x10  (optional)
    %   byte 8..9 : uint16 encF_high_x10 (optional)
    %   byte 10..29: reserved
    deadband = uint16(80);
    buf = zeros(30,1,'uint8');
    buf(1:2) = typecast(deadband,'uint8');
    fprintf("[IDLE] CMD_IDLE_SET_MOTOR_PARAMS (0x%02X), deadband=%d\n", proto.CMD_IDLE_SET_MOTOR_PARAMS, deadband);
    sendBytes30(net, proto.CMD_IDLE_SET_MOTOR_PARAMS, buf);
    ok = net.waitAck(1.0);
    fprintf("    ACK(SET_MOTOR_PARAMS) = %d\n", ok);

    % 4.5: SERVO PARAMS
    % Mapping đề xuất:
    %   byte 0..1 : uint16 servoMinDeg
    %   byte 2..3 : uint16 servoMaxDeg
    %   byte 4..5 : uint16 servoMidDeg
    %   byte 6..29: reserved
    servoMin = uint16(55);
    servoMax = uint16(105);
    servoMid = uint16(75);

    buf = zeros(30,1,'uint8');
    buf(1:2) = typecast(servoMin,'uint8');
    buf(3:4) = typecast(servoMax,'uint8');
    buf(5:6) = typecast(servoMid,'uint8');

    fprintf("[IDLE] CMD_IDLE_SET_SERVO_PARAMS (0x%02X)\n", proto.CMD_IDLE_SET_SERVO_PARAMS);
    sendBytes30(net, proto.CMD_IDLE_SET_SERVO_PARAMS, buf);
    ok = net.waitAck(1.0);
    fprintf("    ACK(SET_SERVO_PARAMS) = %d\n", ok);

    % 4.6: PID PARAMS
    % Mapping đề xuất:
    %   byte  0..3 : float Kp
    %   byte  4..7 : float Ki
    %   byte  8..11: float Kd
    %   byte 12..15: float windup_limit
    %   byte 16..17: uint16 dutyMin
    %   byte 18..19: uint16 dutyMax
    %   byte 20..23: float dutySlew_per_s
    %   byte 24..25: uint16 deadbandDuty
    %   byte 26..29: reserved
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


    %% 5) KẾT THÚC: STOP + IDLE
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
    lo = uint8(bitand(v, 255));
    hi = uint8(bitshift(v, -8));
    net.sendBytes(uint8([uint8(cmd_u8); lo; hi]));
end

% ===== HELPER: gửi cmd + 30 bytes =====
function sendBytes30(net, cmd_u8, payload30_u8)
    b = uint8(payload30_u8(:));
    if numel(b) ~= 30
        error("payload30_u8 must have 30 elements");
    end
    net.sendBytes(uint8([uint8(cmd_u8); b]));
end
