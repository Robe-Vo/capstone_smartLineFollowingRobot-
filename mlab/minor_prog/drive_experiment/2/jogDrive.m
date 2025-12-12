%% stepFilterExperiment.m
% Test encoder noise với step input 4 bậc, mỗi bậc 3 s, nghỉ 1 s
% Có nhập tên file, tên figure, tự động thống kê mean/std theo N

clear; clc; close all;

%% ===== Nhập tên file và tên figure =====
saveName = input("Nhập tên tập tin lưu (không có .mat): ", "s");
figName  = input("Nhập tên figure: ", "s");

if isempty(saveName)
    saveName = "stepFilterResults";
end
if isempty(figName)
    figName = "Step Filter Results";
end

matFile = sprintf("%s.mat", saveName);

%% ===== 1. Thông số chung =====
btName    = "Behind the scream";
btChannel = 1;

Ts_frame  = 0.05;          % 50 ms
T_wait    = 1.0;
T_stepOn  = 3.0;
T_stepOff = 1.0;
stepPWM   = [20 60 140 200 255];
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

winList = [1 3 5 7 9];

%% ===== 2. Mở Bluetooth =====
bt = bluetooth(btName, btChannel);
bt.Timeout = 1.0;

% Xóa buffer cũ
if bt.NumBytesAvailable > 0
    read(bt, bt.NumBytesAvailable, "uint8");
end

stepResults = struct([]);

%% ===== 3. Lặp qua từng cửa sổ N =====
for iWin = 1:numel(winList)
    winN = uint8(winList(iWin));

    % ---- Về IDLE ----
    write(bt, CMD_IDLE, "uint8");
    pause(0.1);
    if bt.NumBytesAvailable > 0, read(bt, bt.NumBytesAvailable, "uint8"); end

    % ---- Gửi cấu hình cửa sổ N ----
    write(bt, [CMD_CONF winN], "uint8");
    pause(0.1);
    if bt.NumBytesAvailable > 0, read(bt, bt.NumBytesAvailable, "uint8"); end

    % ---- Sang OPERATION ----
    write(bt, CMD_OP, "uint8");
    pause(0.1);
    if bt.NumBytesAvailable > 0, read(bt, bt.NumBytesAvailable, "uint8"); end

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
    stepResults(iWin).N        = double(winN);
    stepResults(iWin).t        = t_log;
    stepResults(iWin).rpm      = rpm_log;
    stepResults(iWin).pulses   = double(pulses_log);
    stepResults(iWin).speedCmd = double(speed_log);
end

clear bt;

save(matFile,'stepResults');
fprintf("Đã lưu file: %s\n", matFile);

%% ===== 7. Thống kê mean/std theo N (chỉ lấy đoạn có PWM > 0) =====
nWin = numel(stepResults);
stats = struct('N',[],'rpm_mean',[],'rpm_std',[],'nSamples',[]);

for iWin = 1:nWin
    Nval = stepResults(iWin).N;
    t    = stepResults(iWin).t;
    rpm  = stepResults(iWin).rpm;
    u    = stepResults(iWin).speedCmd;

    idx_valid = (u > 0);
    % Có thể bỏ bớt biên độ quá ngắn bằng cách lọc thêm theo thời gian nếu cần

    if any(idx_valid)
        rpm_sel = rpm(idx_valid);
        stats(iWin).N        = Nval;
        stats(iWin).rpm_mean = mean(rpm_sel);
        stats(iWin).rpm_std  = std(rpm_sel);
        stats(iWin).nSamples = numel(rpm_sel);
    else
        stats(iWin).N        = Nval;
        stats(iWin).rpm_mean = NaN;
        stats(iWin).rpm_std  = NaN;
        stats(iWin).nSamples = 0;
    end
end

save(matFile,'stepResults','stats','-append');

fprintf("\nThống kê theo N (chỉ tính mẫu có PWM>0):\n");
for iWin = 1:nWin
    fprintf("N = %2d: mean = %8.2f rpm, std = %8.2f rpm, n = %4d\n", ...
        stats(iWin).N, stats(iWin).rpm_mean, stats(iWin).rpm_std, stats(iWin).nSamples);
end

%% ===== 8. Plot tín hiệu theo thời gian cho từng N =====
figure('Name',figName,'NumberTitle','off');
sgtitle(figName);

for iWin = 1:nWin
    subplot(nWin,1,iWin);

    yyaxis left;
    plot(stepResults(iWin).t, stepResults(iWin).rpm, 'LineWidth', 1.0);
    ylabel('rpm');

    yyaxis right;
    stairs(stepResults(iWin).t, stepResults(iWin).speedCmd, '--', 'LineWidth', 1.0);
    ylabel('PWM');

    grid on;
    title(sprintf('N = %d', stepResults(iWin).N));

    if iWin == nWin
        xlabel('Time [s]');
    end
end

%% ===== 9. Plot độ lệch chuẩn rpm theo N =====
figure('Name',[figName ' - Std'],'NumberTitle','off');
N_vals   = [stats.N];
std_vals = [stats.rpm_std];

bar(N_vals, std_vals);
grid on;
xlabel('Filter window N');
ylabel('Std of rpm (PWM>0)');
title([figName ' - Std(rpm) vs N']);

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
