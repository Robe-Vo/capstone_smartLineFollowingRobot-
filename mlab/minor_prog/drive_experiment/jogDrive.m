%% stepFilterExperiment.m
% Test encoder noise với step input 4 bậc, mỗi bậc 3 s, nghỉ 1 s
% Thử lần lượt N = 1,3,5,7

clear; clc; close all;

%% 1. Thông số chung
btName    = "Behind the scream";
btChannel = 1;

Ts_frame  = 0.05;          % 50 ms
T_wait    = 1.0;           % chờ trước profile
T_stepOn  = 3.0;           % mỗi bậc bật 3 s
T_stepOff = 1.0;           % giữa 2 bậc tắt 1 s
stepPWM   = [80 140 200 255];  % 4 bậc step
nStepLvl  = numel(stepPWM);
T_profile = nStepLvl*(T_stepOn + T_stepOff); % tổng thời gian profile
T_after   = 1.0;           % chờ sau profile
T_all     = T_wait + T_profile + T_after;

nFrames   = floor(T_all / Ts_frame);

CMD_FWD   = uint8(hex2dec('F1'));
CMD_STOP  = uint8(hex2dec('F0'));
CMD_IDLE  = uint8(hex2dec('FE'));
CMD_CONF  = uint8(hex2dec('EC'));
CMD_OP    = uint8(hex2dec('FF'));
FRAME_LEN = 22;

angleDeg  = uint16(74);
angle_hi  = uint8(bitshift(angleDeg,-8));
angle_lo  = uint8(bitand(angleDeg,255));

winList   = [1 3 5 7];

%% 2. Mở Bluetooth
bt = bluetooth(btName, btChannel);
bt.Timeout = 1.0;

% Xóa buffer cũ
if bt.NumBytesAvailable > 0
    read(bt, bt.NumBytesAvailable, "uint8");
end

stepResults = struct([]);

for iWin = 1:numel(winList)
    winN = uint8(winList(iWin));

    %% 3. Về IDLE, cấu hình N, chuyển sang OPERATION
    write(bt, CMD_IDLE, "uint8");
    pause(0.1);
    if bt.NumBytesAvailable > 0
        read(bt, bt.NumBytesAvailable, "uint8");
    end

    write(bt, [CMD_CONF winN], "uint8");
    pause(0.1);
    if bt.NumBytesAvailable > 0
        read(bt, bt.NumBytesAvailable, "uint8");
    end

    write(bt, CMD_OP, "uint8");
    pause(0.1);
    if bt.NumBytesAvailable > 0
        read(bt, bt.NumBytesAvailable, "uint8");
    end

    %% 4. Khởi tạo log
    t_log      = zeros(nFrames,1);
    rpm_log    = zeros(nFrames,1);
    pulses_log = zeros(nFrames,1,'uint8');
    speed_log  = zeros(nFrames,1,'uint8');

    %% 5. Vòng lặp step
    t0 = tic;
    for k = 1:nFrames
        t_now       = toc(t0);
        t_log(k)    = t_now;

        % ---- profile tốc độ ----
        if t_now <= T_wait
            speedCmd = uint8(0);
        elseif t_now <= T_wait + T_profile
            tau  = t_now - T_wait;
            cycT = T_stepOn + T_stepOff;
            cyc  = floor(tau / cycT);      % 0..nStepLvl-1
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

        % ---- đồng bộ 50 ms ----
        t_target = k*Ts_frame;
        while toc(t0) < t_target
            pause(0.001);
        end
    end

    %% 6. STOP, về IDLE
    ctrlStop = [CMD_STOP; uint8(0); angle_hi; angle_lo];
    write(bt, ctrlStop, "uint8");
    pause(0.1);
    write(bt, CMD_IDLE, "uint8");
    pause(0.1);
    if bt.NumBytesAvailable > 0
        read(bt, bt.NumBytesAvailable, "uint8");
    end

    %% 7. Lưu kết quả cho N hiện tại
    stepResults(iWin).N        = double(winN);
    stepResults(iWin).t        = t_log;
    stepResults(iWin).rpm      = rpm_log;
    stepResults(iWin).pulses   = double(pulses_log);
    stepResults(iWin).speedCmd = double(speed_log);
end

clear bt;

save('stepFilterResults.mat','stepResults');

%% 8. Plot ví dụ cho từng N
figure;
for iWin = 1:numel(stepResults)
    subplot(numel(stepResults),1,iWin);
    yyaxis left;
    plot(stepResults(iWin).t, stepResults(iWin).rpm, 'LineWidth', 1.0);
    ylabel('rpm');
    yyaxis right;
    stairs(stepResults(iWin).t, stepResults(iWin).speedCmd, '--', 'LineWidth', 1.0);
    ylabel('PWM');
    grid on;
    title(sprintf('N = %d', stepResults(iWin).N));
    if iWin == numel(stepResults)
        xlabel('Time [s]');
    end
end

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
