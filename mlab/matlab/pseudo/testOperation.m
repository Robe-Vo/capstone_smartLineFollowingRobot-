%% OPERATION MODE TEST – 5 s RUN, VARY STEERING, LOG I/O, PLOT RESULTS

clear; clc; close all;

%% 1. Parameters and objects

btName    = "Behind the scream";
btChannel = 1;
bt        = bluetooth(btName, btChannel);    % adjust if needed
bt.Timeout = 1.0;                            % 1 s timeout

writeBytes = @(bytes) write(bt, uint8(bytes), "uint8");
readBytes  = @(n)     read(bt, n, "uint8");

% --- Flush any leftover data in buffer ---
if bt.NumBytesAvailable > 0
    read(bt, bt.NumBytesAvailable, "uint8");
end

% --- Test configuration ---
T_total   = 5.0;        % total test time [s]
Ts_frame  = 0.05;       % TIME_SEND_SIGNAL = 50 ms (must match firmware)
N         = round(T_total / Ts_frame);

speed_cmd = 140;        % constant speed command
angle_1   = 90;         % first steering angle
angle_2   = 110;        % second steering angle

% --- Log buffers ---
t_log   = zeros(N, 1);
u_speed = zeros(N, 1);
u_angle = zeros(N, 1);

y_line   = zeros(N, 5, 'uint8');   % line sensors
y_ultra  = zeros(N, 1, 'uint16');  % ultrasonic distance
y_pulses = zeros(N, 1, 'uint8');   % encoder pulses/frame
y_rpm    = zeros(N, 1, 'uint16');  % encoder speed (rpm)

%% 2. Go to IDLE and flush ACKs

% FE: go to IDLE
writeBytes(hex2dec('FE'));
pause(0.05);
ack = readBytes(1);  %#ok<NASGU>  % should be 0x20

% (optional) clear any extra bytes
if bt.NumBytesAvailable > 0
    read(bt, bt.NumBytesAvailable, "uint8");
end

%% 3. Enter OPERATION mode

% FF: IDLE -> OPERATION
writeBytes(hex2dec('FF'));
pause(0.05);
ack = readBytes(1);  %#ok<NASGU>  % should be 0x20

%% 4. 5-second loop: send commands + receive 22-byte frames

t_start = tic;

for k = 1:N
    t_log(k) = toc(t_start);

    % --- Choose steering angle (change halfway through the test) ---
    if t_log(k) < T_total/2
        angle_cmd = angle_1;
    else
        angle_cmd = angle_2;
    end

    % --- Build control frame [cmd, speed, angle_hi, angle_lo] ---
    cmd    = hex2dec('F1');     % forward + steering
    ang16  = uint16(angle_cmd);
    ang_hi = bitshift(ang16, -8);
    ang_lo = bitand(ang16, 255);

    ctrl_frame = uint8([cmd, speed_cmd, ang_hi, ang_lo]);

    % --- Send command ---
    writeBytes(ctrl_frame);

    % --- Read feedback frame (22 bytes) ---
    frame_len = 22;
    frame     = readBytes(frame_len);   % uint8[22]

    if numel(frame) ~= frame_len
        warning('Frame %d: received %d bytes instead of %d.', ...
                k, numel(frame), frame_len);
        break;
    end

    % --- Decode frame ---

    % Line sensors (5 x uint8)
    y_line(k, :) = frame(1:5);

    % Ultrasonic (uint16, big-endian)
    ultra_hi   = uint16(frame(6));
    ultra_lo   = uint16(frame(7));
    y_ultra(k) = bitor(bitshift(ultra_hi, 8), ultra_lo);

    % Encoder pulses per frame (uint8)
    y_pulses(k) = frame(20);   % C index 19 -> MATLAB index 20

    % Encoder speed rpm (uint16, big-endian)
    rpm_hi    = uint16(frame(21));   % C index 20
    rpm_lo    = uint16(frame(22));   % C index 21
    y_rpm(k)  = bitor(bitshift(rpm_hi, 8), rpm_lo);

    % --- Store inputs ---
    u_speed(k) = speed_cmd;
    u_angle(k) = angle_cmd;

    % --- Simple time alignment to Ts_frame ---
    elapsed = toc(t_start) - t_log(k);
    wait_t  = Ts_frame - elapsed;
    if wait_t > 0
        pause(wait_t);
    end
end

%% 5. Stop and go back to IDLE

% Stop in OPERATION: [0xF0, 0, 0, 0]
writeBytes(uint8([hex2dec('F0'), 0, 0, 0]));
pause(0.1);

% Back to IDLE from OPERATION: [0xFE]
writeBytes(hex2dec('FE'));
pause(0.05);
ack = readBytes(1);  %#ok<NASGU>  % ACK 0x20

%% 6. Print summary information

% Convert to double for analysis
rpm_d     = double(y_rpm);
pulses_d  = double(y_pulses);
ultra_d   = double(y_ultra);
speed_d   = double(u_speed);
angle_d   = double(u_angle);

fprintf('\n=== OPERATION MODE TEST SUMMARY ===\n');
fprintf('Total frames (logged): %d\n', find(t_log>0,1,'last'));
fprintf('Total time            : %.3f s\n', t_log(find(t_log>0,1,'last')));

fprintf('\nEncoder pulses/frame:\n');
fprintf('  min: %.1f, max: %.1f, mean: %.2f\n', ...
    min(pulses_d), max(pulses_d), mean(pulses_d));

fprintf('\nEncoder speed [rpm]:\n');
fprintf('  min: %.1f, max: %.1f, mean: %.2f\n', ...
    min(rpm_d), max(rpm_d), mean(rpm_d));

fprintf('\nUltrasonic signal (raw units):\n');
fprintf('  min: %.1f, max: %.1f, mean: %.2f\n', ...
    min(ultra_d), max(ultra_d), mean(ultra_d));

fprintf('\nSteering angle command [deg]:\n');
fprintf('  unique values: ');
fprintf('%.1f ', unique(angle_d));
fprintf('\n');

fprintf('\nTest finished.\n');

%% 7. Plot some markable signals

valid_idx = t_log > 0;
t_s       = t_log(valid_idx);

rpm_d_v     = rpm_d(valid_idx);
pulses_d_v  = pulses_d(valid_idx);
angle_d_v   = angle_d(valid_idx);

figure('Name','Operation Mode Test','NumberTitle','off');
tiledlayout(3,1);

% --- Plot encoder rpm ---
nexttile;
plot(t_s, rpm_d_v, '-o');
grid on;
xlabel('Time [s]');
ylabel('Speed [rpm]');
title('Encoder speed (rpm)');

% --- Plot encoder pulses per frame ---
nexttile;
stem(t_s, pulses_d_v, 'filled');
grid on;
xlabel('Time [s]');
ylabel('Pulses / frame');
title('Encoder pulses per frame');

% --- Plot steering angle command ---
nexttile;
plot(t_s, angle_d_v, '-o');
grid on;
xlabel('Time [s]');
ylabel('Angle command [deg]');
title('Steering angle command');
ylim([min(angle_d_v)-5, max(angle_d_v)+5]);

clear bt;
