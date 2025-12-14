clear; clc;

COM  = "COM9";
BAUD = 921600;
CPR_QUAD = 44;

PWM_SEQ = [25 60 110 220];
HOLD_MS = 6000;
GAP_MS  = 1000;
TS_EXPECT = 0.010;

T_RUN = numel(PWM_SEQ) * (HOLD_MS+GAP_MS)/1000 + 2.0; % margin

sp = serialport(COM, BAUD);
sp.Timeout = 0.05;
flush(sp);

% storage
ev_t = zeros(500000,1,'uint32'); ev_ab = zeros(500000,1,'uint8'); ev_info = zeros(500000,1,'uint8'); ev_n = 0;
sum_tms=zeros(100000,1,'uint32'); sum_dt=zeros(100000,1,'uint16'); sum_dq=zeros(100000,1,'int32'); sum_pus=zeros(100000,1,'uint32'); sum_nAr=zeros(100000,1,'uint8'); sum_n=0;

buf = uint8([]);
t0 = tic;

while toc(t0) < T_RUN
    nAvail = sp.NumBytesAvailable;
    if nAvail > 0
        newBytes = read(sp, nAvail, "uint8");
        buf = [buf; newBytes(:)]; %#ok<AGROW>
    else
        pause(0.001);
    end

    k = 1;
    while k <= numel(buf)
        typ = buf(k);
        if typ == hex2dec('E1')
            if (numel(buf)-k+1) < 7, break; end
            t_us = typecast(uint8(buf(k+1:k+4)), 'uint32');
            ab   = buf(k+5);
            info = buf(k+6);

            ev_n = ev_n + 1;
            if ev_n > numel(ev_t)
                ev_t(end+200000) = uint32(0);
                ev_ab(end+200000) = uint8(0);
                ev_info(end+200000) = uint8(0);
            end
            ev_t(ev_n)=t_us; ev_ab(ev_n)=ab; ev_info(ev_n)=info;
            k = k + 7;

        elseif typ == hex2dec('E2')
            if (numel(buf)-k+1) < 16, break; end
            t_ms = typecast(uint8(buf(k+1:k+4)), 'uint32');
            dtms = typecast(uint8(buf(k+5:k+6)), 'uint16');
            dq   = typecast(uint8(buf(k+7:k+10)), 'int32');
            pus  = typecast(uint8(buf(k+11:k+14)), 'uint32');
            nAr  = buf(k+15);

            sum_n = sum_n + 1;
            if sum_n > numel(sum_tms)
                sum_tms(end+20000)=uint32(0); sum_dt(end+20000)=uint16(0);
                sum_dq(end+20000)=int32(0);   sum_pus(end+20000)=uint32(0);
                sum_nAr(end+20000)=uint8(0);
            end
            sum_tms(sum_n)=t_ms; sum_dt(sum_n)=dtms; sum_dq(sum_n)=dq; sum_pus(sum_n)=pus; sum_nAr(sum_n)=nAr;
            k = k + 16;
        else
            k = k + 1;
        end
    end
    buf = buf(k:end);
end

clear sp;

% trim
ev_t_s = double(ev_t(1:ev_n))*1e-6;
ev_ab  = ev_ab(1:ev_n);
ev_info= ev_info(1:ev_n);

sum_t_s  = double(sum_tms(1:sum_n))*1e-3;
sum_dt_s = double(sum_dt(1:sum_n))*1e-3;
sum_dq_d = double(sum_dq(1:sum_n));
sum_pus_d= double(sum_pus(1:sum_n));
sum_nAr_d= double(sum_nAr(1:sum_n));

% derived rpm
rev_acc = sum_dq_d ./ CPR_QUAD;
rpm_acc = (rev_acc ./ sum_dt_s) * 60;

Ar_per_rev = CPR_QUAD/4;
rpm_per = nan(size(sum_pus_d));
valid = (sum_pus_d>0) & (Ar_per_rev>0);
rpm_per(valid) = ((1 ./ (sum_pus_d(valid)*1e-6)) ./ Ar_per_rev) * 60;

% segment labeling by schedule (relative to first summary time)
t_ref = sum_t_s(1);
t_rel = sum_t_s - t_ref;

seg = zeros(size(sum_t_s));
seg_pwm = zeros(size(sum_t_s));
for i = 1:numel(PWM_SEQ)
    t0s = (i-1)*(HOLD_MS+GAP_MS)/1000 + GAP_MS/1000; % after gap
    t1s = t0s + HOLD_MS/1000;
    mask = (t_rel >= t0s) & (t_rel < t1s);
    seg(mask) = i;
    seg_pwm(mask) = PWM_SEQ(i);
end

% save
out = struct();
out.cfg = struct("COM",COM,"BAUD",BAUD,"CPR_QUAD",CPR_QUAD,"PWM_SEQ",PWM_SEQ,"HOLD_MS",HOLD_MS,"GAP_MS",GAP_MS,"TS_EXPECT",TS_EXPECT,"T_RUN",T_RUN);

out.ev = struct("t_s",ev_t_s,"ab",ev_ab,"info",ev_info);
out.sum = struct("t_s",sum_t_s,"dt_s",sum_dt_s,"dq",sum_dq_d,"period_us",sum_pus_d,"nA_rise",sum_nAr_d);
out.derived = struct("rpm_acc",rpm_acc,"rpm_per",rpm_per);
out.segment = struct("index",seg,"pwm",seg_pwm);

save("comparePeriod2Accu.mat","out");
disp("Saved: comparePeriod2Accu.mat");

% ===================== plot_comparePeriod2Accu.m =====================
% Load comparePeriod2Accu.mat (saved as struct "out") and plot:
% 1) A/B waveform (raw)
% 2) RPM comparison (period vs accumulation) + segment PWM
% 3) dq, period_us, nA_rise
% 4) Histograms + quality metrics
% 5) Per-step (25/60/110/220) stats table in Command Window

clear; clc;

MATFILE = "comparePeriod2Accu.mat";
S = load(MATFILE);

if isfield(S,"out")
    out = S.out;
else
    error("File does not contain variable 'out'.");
end

% --- unpack ---
ev_t_s   = out.ev.t_s;
ev_ab    = out.ev.ab;
ev_info  = out.ev.info;

sum_t_s  = out.sum.t_s;
sum_dt_s = out.sum.dt_s;
sum_dq   = out.sum.dq;
sum_pus  = out.sum.period_us;
sum_nAr  = out.sum.nA_rise;

rpm_acc  = out.derived.rpm_acc;
rpm_per  = out.derived.rpm_per;

hasSeg = isfield(out,"segment") && isfield(out.segment,"pwm") && ~isempty(out.segment.pwm);
if hasSeg
    seg_pwm = out.segment.pwm;
    seg_idx = out.segment.index;
else
    seg_pwm = zeros(size(sum_t_s));
    seg_idx = zeros(size(sum_t_s));
end

CPR_QUAD = out.cfg.CPR_QUAD;
PWM_SEQ  = out.cfg.PWM_SEQ;

% --- decode A/B levels at event instants ---
A = bitand(ev_ab, 1) > 0;
B = bitand(bitshift(ev_ab, -1), 1) > 0;

% sort events by time
[ev_t_s, id] = sort(ev_t_s);
A = A(id); B = B(id); ev_ab = ev_ab(id); ev_info = ev_info(id);

t_end = max([ev_t_s; sum_t_s]);

% ===================== PLOT 1: Raw waveforms =====================
figure('Name','Encoder A/B waveform (raw)');
subplot(2,1,1);
stairs(ev_t_s, A, 'LineWidth', 1); grid on;
ylabel('A'); xlim([0 t_end]);
title('Channel A (raw level at each event)');
subplot(2,1,2);
stairs(ev_t_s, B, 'LineWidth', 1); grid on;
ylabel('B'); xlabel('Time (s)'); xlim([0 t_end]);
title('Channel B (raw level at each event)');

% ===================== PLOT 2: RPM compare + PWM steps =====================
figure('Name','RPM compare (period vs accumulation) + PWM steps');
plot(sum_t_s, rpm_acc, 'LineWidth', 1); hold on;
plot(sum_t_s, rpm_per, 'LineWidth', 1);
grid on; xlabel('Time (s)'); ylabel('RPM');
legend('rpm\_accum (dq/dt)', 'rpm\_period (A-rise period)','Location','best');

if hasSeg
    % plot PWM steps on right axis (scaled)
    yyaxis right;
    plot(sum_t_s, seg_pwm, 'LineWidth', 1);
    ylabel('PWM (command)');
    ylim([0 max(PWM_SEQ)*1.2]);
    yyaxis left;
end

% ===================== PLOT 3: Raw metrics =====================
figure('Name','Raw metrics (dq / period_us / nA_rise)');
subplot(3,1,1);
plot(sum_t_s, sum_dq, 'LineWidth', 1); grid on;
ylabel('dq / frame'); xlim([0 t_end]);

subplot(3,1,2);
plot(sum_t_s, sum_pus, 'LineWidth', 1); grid on;
ylabel('period\_us'); xlim([0 t_end]);

subplot(3,1,3);
plot(sum_t_s, sum_nAr, 'LineWidth', 1); grid on;
ylabel('nA\_rise/frame'); xlabel('Time (s)'); xlim([0 t_end]);

% ===================== PLOT 4: Histograms =====================
validP = sum_pus(sum_pus > 0);
figure('Name','Histograms');
subplot(2,2,1);
histogram(sum_dq, 40); grid on; title('dq histogram'); xlabel('dq'); ylabel('count');

subplot(2,2,2);
if ~isempty(validP)
    histogram(validP, 40); grid on; title('period\_us histogram (valid)'); xlabel('us'); ylabel('count');
else
    text(0.1,0.5,'No valid period_us','Units','normalized'); axis off;
end

subplot(2,2,3);
mask = isfinite(rpm_acc);
histogram(rpm_acc(mask), 50); grid on; title('rpm\_acc histogram'); xlabel('rpm'); ylabel('count');

subplot(2,2,4);
mask = isfinite(rpm_per);
histogram(rpm_per(mask), 50); grid on; title('rpm\_per histogram'); xlabel('rpm'); ylabel('count');

% ===================== PLOT 5: Event timing + A rising intervals =====================
% Extract A rising timestamps from event frames
isAevent = bitand(ev_info,1)==0;
isArise  = bitand(ev_info,2)~=0;
tAr = ev_t_s(isAevent & isArise);

figure('Name','A-rising timing');
subplot(2,1,1);
if numel(tAr) >= 2
    dT = diff(tAr);
    plot(tAr(2:end), dT*1e3, 'LineWidth', 1); grid on;
    ylabel('\Delta t (ms)'); xlabel('Time (s)');
    title('A-rising interval over time');
else
    text(0.1,0.5,'Not enough A-rising events','Units','normalized'); axis off;
end

subplot(2,1,2);
if numel(tAr) >= 2
    histogram(dT*1e3, 50); grid on;
    xlabel('\Delta t (ms)'); ylabel('count');
    title('Histogram of A-rising interval');
else
    text(0.1,0.5,'Not enough A-rising events','Units','normalized'); axis off;
end

% ===================== COMMAND WINDOW SUMMARY =====================
fprintf("\n==================== SUMMARY (FROM MAT FILE) ====================\n");
fprintf("Events (E1): %d\n", numel(ev_t_s));
fprintf("Summary frames (E2): %d\n", numel(sum_t_s));

dt_mean = mean(sum_dt_s);
dt_std  = std(sum_dt_s);
fprintf("dt_s: mean=%.6f, std=%.6f\n", dt_mean, dt_std);

fprintf("valid period_us: %d / %d\n", nnz(sum_pus>0), numel(sum_pus));
if ~isempty(validP)
    fprintf("period_us: mean=%.2f, std=%.2f, min=%.2f, max=%.2f\n", mean(validP), std(validP), min(validP), max(validP));
end

mask = isfinite(rpm_acc) & isfinite(rpm_per);
fprintf("rpm compare samples: %d\n", nnz(mask));
if nnz(mask) > 5
    d = rpm_per(mask) - rpm_acc(mask);
    rmse = sqrt(mean(d.^2));
    mae  = mean(abs(d));
    fprintf("rpm_acc: mean=%.3f, std=%.3f\n", mean(rpm_acc(mask)), std(rpm_acc(mask)));
    fprintf("rpm_per: mean=%.3f, std=%.3f\n", mean(rpm_per(mask)), std(rpm_per(mask)));
    fprintf("diff(per-acc): mean=%.3f, std=%.3f\n", mean(d), std(d));
    fprintf("RMSE=%.3f rpm, MAE=%.3f rpm\n", rmse, mae);
end

% Quadrature heuristic: both bits changed simultaneously
ab2 = bitand(ev_ab,3);
if numel(ab2) >= 2
    dab = bitxor(ab2(2:end), ab2(1:end-1));
    bothChanged = (dab == 3);
    fprintf("both-bits-changed events: %d\n", nnz(bothChanged));
end

fprintf("A-rising events: %d\n", numel(tAr));
if numel(tAr) >= 2
    dT = diff(tAr);
    fprintf("A-rise interval (ms): mean=%.3f, std=%.3f, min=%.3f, max=%.3f\n", ...
        mean(dT)*1e3, std(dT)*1e3, min(dT)*1e3, max(dT)*1e3);
end

% ===================== Per-step stats table =====================
if hasSeg
    fprintf("\n==================== PER-PWM STEP STATS ====================\n");
    fprintf("%6s  %8s  %10s  %10s  %10s  %10s  %10s\n", ...
        "PWM","N","acc_mean","acc_std","per_mean","per_std","RMSE");
    for i = 1:numel(PWM_SEQ)
        pwm = PWM_SEQ(i);
        m = (seg_pwm == pwm) & isfinite(rpm_acc) & isfinite(rpm_per);
        N = nnz(m);
        if N < 5
            fprintf("%6d  %8d  %10s  %10s  %10s  %10s  %10s\n", pwm, N, "-", "-", "-", "-", "-");
        else
            d = rpm_per(m) - rpm_acc(m);
            rmse = sqrt(mean(d.^2));
            fprintf("%6d  %8d  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f\n", ...
                pwm, N, mean(rpm_acc(m)), std(rpm_acc(m)), mean(rpm_per(m)), std(rpm_per(m)), rmse);
        end
    end
    fprintf("============================================================\n");
end
% ===== Re-evaluate after sign fix hypothesis =====
rpm_acc_fix = -rpm_acc;   % giả sử accumulation bị đảo dấu

% Filter: only frames where we actually have pulses
mask_good = (out.sum.nA_rise > 0) & isfinite(rpm_per) & isfinite(rpm_acc_fix);

d_fix = rpm_per(mask_good) - rpm_acc_fix(mask_good);
rmse_fix = sqrt(mean(d_fix.^2));
mae_fix  = mean(abs(d_fix));

fprintf("\n===== AFTER SIGN FIX (rpm_acc_fix = -rpm_acc) =====\n");
fprintf("Samples: %d\n", nnz(mask_good));
fprintf("RMSE=%.3f rpm, MAE=%.3f rpm\n", rmse_fix, mae_fix);

% Per-step table after fix
if isfield(out,"segment")
    fprintf("\n===== PER-PWM STEP (AFTER SIGN FIX) =====\n");
    fprintf("%6s  %8s  %10s  %10s  %10s  %10s  %10s\n", ...
        "PWM","N","acc_mean","acc_std","per_mean","per_std","RMSE");
    for i = 1:numel(out.cfg.PWM_SEQ)
        pwm = out.cfg.PWM_SEQ(i);
        m = (out.segment.pwm == pwm) & mask_good;
        N = nnz(m);
        if N < 5
            fprintf("%6d  %8d  %10s  %10s  %10s  %10s  %10s\n", pwm, N, "-", "-", "-", "-", "-");
        else
            dd = rpm_per(m) - rpm_acc_fix(m);
            fprintf("%6d  %8d  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f\n", ...
                pwm, N, mean(rpm_acc_fix(m)), std(rpm_acc_fix(m)), mean(rpm_per(m)), std(rpm_per(m)), sqrt(mean(dd.^2)));
        end
    end
    fprintf("========================================\n");
end
