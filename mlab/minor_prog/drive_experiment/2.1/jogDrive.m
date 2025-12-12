%% stepFilterExperiment_ema.m
% Test encoder noise với step input nhiều bậc, mỗi bậc 3 s, nghỉ 1 s
% Lọc nhiễu encoder bậc 1 (EMA), quét alpha = [0.4 0.3 0.2 0.1], beta = 0
% Có nhập tên file, tên figure, tự động thống kê mean/std theo alpha

clear; clc; close all;

%% ===== Nhập tên file và tên figure =====
saveName = input("Nhập tên tập tin lưu (không có .mat): ", "s");
figName  = input("Nhập tên figure: ", "s");

if isempty(saveName)
    saveName = "stepFilterEMAResults";
end
if isempty(figName)
    figName = "Step Filter EMA Results";
end

matFile = sprintf("%s.mat", saveName);

%% ===== 1. Thông số chung =====
btName    = "Behind the scream";
btChannel = 1;

Ts_frame  = 0.05;          % 50 ms
T_wait    = 1.0;
T_stepOn  = 3.0;
T_stepOff = 1.0;
stepPWM   = [30 60 140 200 255];
nStepLvl  = numel(stepPWM);
T_profile = nStepLvl * (T_stepOn + T_stepOff);
T_after   = 1.0;
T_all     = T_wait + T_profile + T_after;

nFrames = floor(T_all / Ts_frame);

CMD_FWD   = uint8(hex2dec('F1'));
CMD_STOP  = uint8(hex2dec('F0'));
CMD_IDLE  = uint8(hex2dec('FE'));
CMD_CONF  = uint8(hex2dec('EC'));
CMD_OP    = uint8(hex2dec('FF'));
FRAME_LEN = 22;

angleDeg  = uint16(74);
angle_hi  = uint8(bitshift(angleDeg,-8));
angle_lo  = uint8(bitand(angleDeg,255));

% EMA 1st order: mode m = 0, alpha list, beta = 0
alphaList = [0.4 0.3 0.2 0.1];
beta      = 0.0;           % cố định = 0

%% ===== 2. Mở Bluetooth =====
bt = bluetooth(btName, btChannel);
bt.Timeout = 1.0;

% Xóa buffer cũ
if bt.NumBytesAvailable > 0
    read(bt, bt.NumBytesAvailable, "uint8");
end

stepResults = struct([]);

%% ===== 3. Lặp qua từng alpha =====
for iA = 1:numel(alphaList)
    alpha = alphaList(iA);

    % scale alpha,beta sang uint16 với hệ số 1/1000 (phía ESP32)
    alpha_raw = uint16(round(alpha * 1000));
    beta_raw  = uint16(round(beta  * 1000));

    % tách thành hi/lo
    a_hi = uint8(bitshift(alpha_raw, -8));
    a_lo = uint8(bitand(alpha_raw, 255));
    b_hi = uint8(bitshift(beta_raw,  -8));
    b_lo = uint8(bitand(beta_raw,  255));

    % mode = 0: EMA bậc 1
    mode = uint8(0);

    % ---- Về IDLE ----
    write(bt, CMD_IDLE, "uint8");
    pause(0.1);
    if bt.NumBytesAvailable > 0
        read(bt, bt.NumBytesAvailable, "uint8");
    end

    % ---- Gửi cấu hình filter (0xEC m a_hi a_lo b_hi b_lo) ----
    cfgFrame = [CMD_CONF; mode; a_hi; a_lo; b_hi; b_lo];
    write(bt, cfgFrame, "uint8");
    pause(0.1);
    if bt.NumBytesAvailable > 0
        read(bt, bt.NumBytesAvailable, "uint8");
    end

    % ---- Sang OPERATION ----
    write(bt, CMD_OP, "uint8");
    pause(0.1);
    if bt.NumBytesAvailable > 0
        read(bt, bt.NumBytesAvailable, "uint8");
    end

    % ---- Khởi tạo log ----
    t_log      = zeros(nFrames,1);
    rpm_log    = zeros(nFrames,1);
    pulses_log = zeros(nFrames,1,'uint8');
    speed_log  = zeros(nFrames,1,'uint8');

    %% ===== 4. Chạy profile step =====
    t0 = tic;
    for k = 1:nFrames
        t_now       = toc(t0);
        t_log(k)    = t_now;

        % ---- profile step ----
        if t_now <= T_wait
            speedCmd = uint8(0);
        elseif t_now <= T_wait + T_profile
            tau  = t_now - T_wait;
            cycT = T_stepOn + T_stepOff;
            cyc  = floor(tau / cycT);

            if cyc >= nStepLvl
                speedCmd = uint8(0);
            else
                tau_in = tau - cyc*cycT;
                if tau_in <= T_stepOn
                    speedCmd = uint8(stepPWM(cyc+1));
                else
                    speedCmd = uint8(0);
                end
            end
        else
            speedCmd = uint8(0);
        end

        speed_log(k) = speedCmd;

        % ---- gửi control ----
        ctrl = [CMD_FWD; speedCmd; angle_hi; angle_lo];
        write(bt, ctrl, "uint8");

        % ---- đọc frame 22 byte ----
        frame = readFrame22(bt, FRAME_LEN, 1.0);
        if isempty(frame)
            nFrames   = k-1;
            t_log     = t_log(1:nFrames);
            rpm_log   = rpm_log(1:nFrames);
            pulses_log= pulses_log(1:nFrames);
            speed_log = speed_log(1:nFrames);
            break;
        end

        pulses_log(k) = frame(20);
        rpm_hi        = frame(21);
        rpm_lo        = frame(22);
        rpm_log(k)    = double(uint16(rpm_hi)*256 + uint16(rpm_lo));

        % ---- đồng bộ thời gian ----
        t_target = k*Ts_frame;
        while toc(t0) < t_target
            pause(0.001);
        end
    end

    %% ===== 5. STOP và về IDLE =====
    ctrlStop = [CMD_STOP; uint8(0); angle_hi; angle_lo];
    write(bt, ctrlStop, "uint8");
    pause(0.1);

    write(bt, CMD_IDLE, "uint8");
    pause(0.1);
    if bt.NumBytesAvailable > 0
        read(bt, bt.NumBytesAvailable, "uint8");
    end

    %% ===== 6. Lưu kết quả =====
    stepResults(iA).alpha     = alpha;
    stepResults(iA).alpha_raw = double(alpha_raw);
    stepResults(iA).beta      = beta;
    stepResults(iA).beta_raw  = double(beta_raw);
    stepResults(iA).mode      = double(mode);

    stepResults(iA).t        = t_log;
    stepResults(iA).rpm      = rpm_log;
    stepResults(iA).pulses   = double(pulses_log);
    stepResults(iA).speedCmd = double(speed_log);
end

clear bt;

save(matFile,'stepResults');
fprintf("Đã lưu file: %s\n", matFile);

%% ===== 7. Thống kê mean/std theo alpha (chỉ lấy đoạn có PWM > 0) =====
nA    = numel(stepResults);
stats = struct('alpha',[],'rpm_mean',[],'rpm_std',[],'nSamples',[]);

for iA = 1:nA
    alpha = stepResults(iA).alpha;
    t     = stepResults(iA).t;
    rpm   = stepResults(iA).rpm;
    u     = stepResults(iA).speedCmd;

    idx_valid = (u > 0);

    if any(idx_valid)
        rpm_sel              = rpm(idx_valid);
        stats(iA).alpha      = alpha;
        stats(iA).rpm_mean   = mean(rpm_sel);
        stats(iA).rpm_std    = std(rpm_sel);
        stats(iA).nSamples   = numel(rpm_sel);
    else
        stats(iA).alpha      = alpha;
        stats(iA).rpm_mean   = NaN;
        stats(iA).rpm_std    = NaN;
        stats(iA).nSamples   = 0;
    end
end

save(matFile,'stepResults','stats','-append');

fprintf("\nThống kê theo alpha (chỉ tính mẫu có PWM>0):\n");
for iA = 1:nA
    fprintf("alpha = %.2f: mean = %8.2f rpm, std = %8.2f rpm, n = %4d\n", ...
        stats(iA).alpha, stats(iA).rpm_mean, stats(iA).rpm_std, stats(iA).nSamples);
end

%% ===== 8. Plot tín hiệu theo thời gian cho từng alpha =====
figure('Name',figName,'NumberTitle','off');
sgtitle(figName);

for iA = 1:nA
    subplot(nA,1,iA);

    yyaxis left;
    plot(stepResults(iA).t, stepResults(iA).rpm, 'LineWidth', 1.0);
    ylabel('rpm');

    yyaxis right;
    stairs(stepResults(iA).t, stepResults(iA).speedCmd, '--', 'LineWidth', 1.0);
    ylabel('PWM');

    grid on;
    title(sprintf('alpha = %.2f', stepResults(iA).alpha));

    if iA == nA
        xlabel('Time [s]');
    end
end

%% ===== 9. Plot độ lệch chuẩn rpm theo alpha =====
figure('Name',[figName ' - Std'],'NumberTitle','off');
alpha_vals = [stats.alpha];
std_vals   = [stats.rpm_std];

bar(alpha_vals, std_vals);
grid on;
xlabel('alpha (EMA)');
ylabel('Std of rpm (PWM>0)');
title([figName ' - Std(rpm) vs alpha']);

%% ===== Helper: đọc đủ 22 byte =====
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
