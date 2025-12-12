%% rampCharacteristicsExperiment.m
% 1) Kết nối ESP32 qua Bluetooth
% 2) Chuyển sang OPERATION_MODE
% 3) Chờ 1 s (trong thời gian này gửi speed = 0, angle = 76)
% 4) Ramp tốc độ: 0 -> 255 trong 10 s, góc servo giữ 76 deg
% 5) Ramp tốc độ: 255 -> 0 trong 10 s (tổng ramp 20 s)
% 6) Mỗi frame: gửi [0xF1, speed, angle_hi, angle_lo], nhận 22 byte, log toàn bộ
% 7) Chuyển về IDLE, tính encoder count, dead-band start/stop, in baseline, lưu struct và vẽ

clear; clc; close all;

%% 1. Thông số kết nối và thử nghiệm

btName    = "Behind the scream";
btChannel = 1;

N = 1;

T_wait    = 1.0;                  % thời gian chờ trong OPERATION, speed = 0
T_up      = 5.0;                 % ramp-up duration [s]
T_down    = 5.0;                 % ramp-down duration [s]
T_total   = T_up + T_down;        % 20 s tổng thời gian ramp
T_all     = T_wait + T_total;     % tổng thời gian (chờ + ramp)

Ts_frame  = 0.05;                 % 50 ms (TIME_SEND_SIGNAL)
nSteps    = floor(T_all / Ts_frame);

CMD_FWD   = uint8(hex2dec('F1')); % 0xF1: forward + steering
CMD_STOP  = uint8(hex2dec('F0')); % 0xF0: stop
angleDeg  = uint16(75);           % góc servo cố định = 76 deg

FRAME_LEN = 22;                   % độ dài frame phản hồi từ main.cpp

%% 2. Mở Bluetooth

bt = bluetooth(btName, btChannel);
bt.Timeout = 1.0;      % [s]

% Xóa buffer cũ nếu có

write(bt,[0xEC N],"uint8")

if bt.NumBytesAvailable > 0
    read(bt, bt.NumBytesAvailable, "uint8");
end

%% 3. Đưa về IDLE, sau đó sang OPERATION_MODE

% Đảm bảo về IDLE (0xFE từ IDLE hay OPERATION đều hợp lệ)
write(bt, uint8(hex2dec('FE')), "uint8");
pause(0.1);
if bt.NumBytesAvailable > 0
    read(bt, bt.NumBytesAvailable, "uint8");
end

% Gửi 0xFF để chuyển sang OPERATION_MODE
write(bt, uint8(hex2dec('FF')), "uint8");

% Đọc ACK 0x20 (nếu có), tối đa 1 s giống baseline
ack    = [];
tStart = tic;
while toc(tStart) < 1.0 && isempty(ack)
    if bt.NumBytesAvailable > 0
        ack = read(bt, bt.NumBytesAvailable, "uint8");
    else
        pause(0.01);
    end
end

%% 4. Khởi tạo các biến log

t_log        = zeros(nSteps, 1);
line_log     = zeros(nSteps, 5, 'uint8');   % 5 IR
ultra_log    = zeros(nSteps, 1, 'uint16');  % ultrasonic
pulses_log   = zeros(nSteps, 1, 'uint8');   % encoder pulses/frame
rpm_log      = zeros(nSteps, 1, 'double');  % encoder rpm (giải mã uint16)
ctrl_speed   = zeros(nSteps, 1, 'uint8');   % PWM command
ctrl_angle   = zeros(nSteps, 1, 'uint16');  % góc command

%% 5. Vòng lặp: 1 s chờ (speed=0) + 20 s ramp

t0 = tic;

for k = 1:nSteps

    t_now     = toc(t0);
    t_log(k)  = t_now;

    % ----- 5.1. Tính speedCmd theo thời gian -----
    if t_now <= T_wait
        % Thời gian chờ: speed = 0
        speedCmd = uint8(0);
    elseif t_now <= (T_wait + T_up)
        % Ramp up 0 -> 255 trong 10 s (sau khi chờ xong)
        tau = t_now - T_wait;
        s   = tau / T_up;          % 0 -> 1
        s   = max(0,min(1,s));
        speedCmd = uint8(round(255 * s));
    elseif t_now <= (T_wait + T_total)
        % Ramp down 255 -> 0 trong 10 s
        tau = t_now - (T_wait + T_up);
        s   = tau / T_down;        % 0 -> 1
        s   = max(0,min(1,s));
        speedCmd = uint8(round(255 * (1 - s)));
    else
        speedCmd = uint8(0);
    end

    ctrl_speed(k) = speedCmd;
    ctrl_angle(k) = angleDeg;

    % ----- 5.2. Tạo 4 byte control [F1, speed, angle_hi, angle_lo] -----
    angle_hi = uint8(bitshift(angleDeg, -8));
    angle_lo = uint8(bitand(angleDeg, 255));

    ctrl = [CMD_FWD; speedCmd; angle_hi; angle_lo];

    % Gửi control
    write(bt, ctrl, "uint8");

    % ----- 5.3. Đọc frame 22 byte theo format main.cpp -----
    frame = readFrame22(bt, FRAME_LEN, 1.0);   % timeout 1 s

    if isempty(frame)
        warning("Không nhận đủ 22 byte tại bước %d, dừng chương trình.", k);
        nSteps     = k - 1;
        t_log      = t_log(1:nSteps);
        line_log   = line_log(1:nSteps, :);
        ultra_log  = ultra_log(1:nSteps);
        pulses_log = pulses_log(1:nSteps);
        rpm_log    = rpm_log(1:nSteps);
        ctrl_speed = ctrl_speed(1:nSteps);
        ctrl_angle = ctrl_angle(1:nSteps);
        break;
    end

    % ----- 5.4. Giải mã frame 22 byte (OPERATION_MODE trong main.cpp) -----
    % frame[0..4]  = IR
    % frame[5]     = ultra_hi
    % frame[6]     = ultra_lo
    % frame[19]    = pulses (0..255)
    % frame[20]    = speed_hi (rpm)
    % frame[21]    = speed_lo (rpm)

    % IR 5 kênh
    line_log(k, :) = frame(1:5);

    % Ultrasonic
    ultra_hi       = frame(6);
    ultra_lo       = frame(7);
    ultra_val      = uint16(ultra_hi)*256 + uint16(ultra_lo);
    ultra_log(k)   = ultra_val;

    % Encoder pulses per frame
    pulses_val     = frame(20);      % C index 19 -> MATLAB 20
    pulses_log(k)  = pulses_val;

    % Encoder speed rpm
    rpm_hi         = frame(21);      % C index 20 -> MATLAB 21
    rpm_lo         = frame(22);      % C index 21 -> MATLAB 22
    rpm_u16        = uint16(rpm_hi)*256 + uint16(rpm_lo);
    rpm_log(k)     = double(rpm_u16);

    % ----- 5.5. Đồng bộ thời gian ~50 ms -----
    t_target = k * Ts_frame;
    while toc(t0) < t_target
        pause(0.001);
    end
end

%% 6. Gửi STOP và quay về IDLE

% STOP trong OPERATION_MODE: [F0, 0, angle_hi, angle_lo]
angle_hi = uint8(bitshift(angleDeg, -8));
angle_lo = uint8(bitand(angleDeg, 255));
ctrlStop = [CMD_STOP; uint8(0); angle_hi; angle_lo];
write(bt, ctrlStop, "uint8");
pause(0.1);

% Quay về IDLE_MODE bằng 0xFE
write(bt, uint8(hex2dec('FE')), "uint8");
pause(0.1);

%% 7. Đóng Bluetooth

clear bt;

%% 8. Hậu xử lý: encoder count, dead-band, baseline start/stop

t_vec    = t_log(:);
rpm_d    = rpm_log(:);
pulses_d = double(pulses_log(:));
u_pwm    = double(ctrl_speed(:));

% Encoder count tích lũy
enc_count = cumsum(pulses_d);

% Dead-band START: frame đầu tiên trong ramp-up (t từ T_wait tới T_wait+T_up) có rpm > 0
idx_db_start = find((t_vec >= T_wait) & (t_vec <= T_wait + T_up) & (rpm_d > 0), 1, 'first');
if isempty(idx_db_start)
    deadband_start_time = NaN;
    deadband_start_pwm  = NaN;
else
    deadband_start_time = t_vec(idx_db_start);
    deadband_start_pwm  = u_pwm(idx_db_start);
end

% Dead-band STOP: frame cuối cùng trong toàn bộ profile có rpm > 0
idx_db_stop = find(rpm_d > 0, 1, 'last');
if isempty(idx_db_stop)
    deadband_stop_time = NaN;
    deadband_stop_pwm  = NaN;
else
    deadband_stop_time = t_vec(idx_db_stop);
    deadband_stop_pwm  = u_pwm(idx_db_stop);
end

% Motor START/STOP (baseline) – log ra màn hình MATLAB
motorStart.time = deadband_start_time;
motorStart.pwm  = deadband_start_pwm;

motorStop.time  = deadband_stop_time;
motorStop.pwm   = deadband_stop_pwm;

fprintf('\n=== MOTOR BASELINE (START / STOP) ===\n');
if isnan(motorStart.time)
    fprintf('Motor START : không phát hiện rpm > 0 trong đoạn ramp-up.\n');
else
    fprintf('Motor START : t = %.3f s, PWM = %.1f\n', ...
        motorStart.time, motorStart.pwm);
end

if isnan(motorStop.time)
    fprintf('Motor STOP  : không phát hiện rpm > 0 trong toàn bộ thử nghiệm.\n');
else
    fprintf('Motor STOP  : t = %.3f s, PWM = %.1f\n', ...
        motorStop.time, motorStop.pwm);
end

%% 9. Lưu struct kết quả

rampResult = struct();
rampResult.t              = t_vec;
rampResult.speed_cmd      = u_pwm;
rampResult.encoder_rpm    = rpm_d;
rampResult.encoder_pulses = pulses_d;
rampResult.encoder_count  = enc_count;
rampResult.line_raw       = line_log;
rampResult.ultra_raw      = double(ultra_log);

rampResult.deadband = struct( ...
    'start_time', deadband_start_time, ...
    'start_pwm',  deadband_start_pwm, ...
    'stop_time',  deadband_stop_time, ...
    'stop_pwm',   deadband_stop_pwm ...
    );

rampResult.motorStart = motorStart;
rampResult.motorStop  = motorStop;

save('rampCharacteristicsResult.mat', 'rampResult');

%% 10. Plot encoder count và lồng tốc độ encoder vào command speed

figure;

% 10.1 Encoder count theo thời gian
subplot(2,1,1);
plot(rampResult.t, rampResult.encoder_count, 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('Encoder count');
title('Encoder count over time');

% 10.2 Lồng encoder rpm vào command speed (chung 1 figure, 2 trục Y)
subplot(2,1,2);
yyaxis left;
plot(rampResult.t, rampResult.encoder_rpm, 'LineWidth', 1.2);
ylabel('Speed [rpm]');
grid on;

yyaxis right;
plot(rampResult.t, rampResult.speed_cmd, '--', 'LineWidth', 1.2);
ylabel('PWM command');

xlabel('Time [s]');
legend('Angular velocity','Command speed')
title('Encoder speed (rpm) and command speed (PWM)');

%% ===== Helper: đọc đủ 22 byte từ ESP32 trong OPERATION_MODE =====
function frame = readFrame22(bt, frameLen, timeout_s)
    if nargin < 3
        timeout_s = 1.0;
    end

    buffer = uint8([]);
    tStart = tic;

    while numel(buffer) < frameLen && toc(tStart) < timeout_s
        nAvail = bt.NumBytesAvailable;
        if nAvail > 0
            chunk  = read(bt, nAvail, "uint8");
            buffer = [buffer; chunk]; %#ok<AGROW>
        else
            pause(0.001);
        end
    end

    if numel(buffer) < frameLen
        frame = [];
    else
        frame = buffer(1:frameLen);
    end
end
