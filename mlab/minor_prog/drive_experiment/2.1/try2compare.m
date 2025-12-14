% ===================== compare_filters_tunable.m =====================
% Load comparePeriod2Accu.mat (variable: out)
% Apply: gating -> median(W) -> IIR1(alpha) and IIR2(cascade, alpha)
% Compare period vs accumulation and plot results
%
% REQUIRE: comparePeriod2Accu.mat saved with struct "out"

clear; clc; close all;

%% ===================== TUNABLE PARAMETERS =====================
MATFILE       = "comparePeriod2Accu.mat";

% --- Encoder / math ---
ACC_SIGN_FIX  = true;        % true if dq sign is inverted vs desired direction
CPR_QUAD      = [];          % leave [] to use out.cfg.CPR_QUAD

% --- Sampling / gating ---
TS_EXPECT_S   = 0.010;       % expected dt (s) for reference only
PERIOD_MAX_US = 200000;      % reject period_us larger than this (gap/stop spikes)
REQ_PULSES    = 1;           % require >= this many A-rises per frame to accept accumulation (1 is minimal)

% --- Filtering ---
MED_W         = 5;           % moving median window length
ALPHA         = 0.2;         % IIR smoothing factor
USE_MEDIAN    = true;        % enable/disable median stage

% --- Optional: accumulation over sliding window K (reduce quantization) ---
USE_ACC_K     = false;       % true to compute rpm from dq summed over K frames
ACC_K         = 5;           % K-frame window (effective window ~K*Ts)

% --- Plot controls ---
SHOW_PWM_AXIS = true;        % if out.segment exists, plot PWM on right axis
PLOT_RANGE_S  = [];          % [] for full, or [t0 t1] seconds
%% ===============================================================


%% ===================== LOAD =====================
S = load(MATFILE);
if ~isfield(S,"out"), error("MAT-file must contain variable 'out'."); end
out = S.out;

t   = out.sum.t_s(:);
dt  = out.sum.dt_s(:);
dq  = out.sum.dq(:);
pus = out.sum.period_us(:);
nAr = out.sum.nA_rise(:);

if isempty(CPR_QUAD)
    if isfield(out,"cfg") && isfield(out.cfg,"CPR_QUAD")
        CPR_QUAD = out.cfg.CPR_QUAD;
    else
        error("CPR_QUAD not found. Set CPR_QUAD in tunables.");
    end
end

% PWM segment (optional)
hasSeg = isfield(out,"segment") && isfield(out.segment,"pwm") && ~isempty(out.segment.pwm);
if hasSeg
    pwm_cmd = out.segment.pwm(:);
    pwm_seq = out.cfg.PWM_SEQ(:);
else
    pwm_cmd = zeros(size(t));
    pwm_seq = [];
end

% set plot window
if isempty(PLOT_RANGE_S)
    t0 = min(t); t1 = max(t);
else
    t0 = PLOT_RANGE_S(1); t1 = PLOT_RANGE_S(2);
end
inPlot = (t >= t0) & (t <= t1);

%% ===================== RAW RPM =====================
% Accumulation: dq per frame
rpm_acc = (dq ./ CPR_QUAD) ./ dt * 60;

% Period-based: use A-rising frequency (A-rises per rev = CPR_QUAD/4)
Ar_per_rev = CPR_QUAD/4;
rpm_per = nan(size(pus));
mP = (pus > 0);
rpm_per(mP) = ( (1./(pus(mP)*1e-6)) ./ Ar_per_rev ) * 60;

% Optional sign fix
if ACC_SIGN_FIX
    rpm_acc = -rpm_acc;
end

%% ===================== OPTIONAL: ACCUMULATION K-FRAME =====================
if USE_ACC_K
    dqK = movsum(dq, [ACC_K-1 0], 'omitnan');
    dtK = movsum(dt, [ACC_K-1 0], 'omitnan');
    nArK = movsum(nAr, [ACC_K-1 0], 'omitnan');

    rpm_accK = (dqK ./ CPR_QUAD) ./ dtK * 60;
    if ACC_SIGN_FIX
        rpm_accK = -rpm_accK; % keep consistent if you applied sign fix above (still ok)
    end
else
    rpm_accK = rpm_acc;
    nArK = nAr;
end

%% ===================== GATING =====================
% Acc gating: require pulses
mAcc = (nArK >= REQ_PULSES) & isfinite(rpm_accK);

% Period gating: reject zero/NaN and large gaps
mPer = (pus > 0) & (pus < PERIOD_MAX_US) & isfinite(rpm_per);

acc_g = rpm_accK; acc_g(~mAcc) = NaN;
per_g = rpm_per;  per_g(~mPer) = NaN;

%% ===================== MEDIAN + IIR1 + IIR2 =====================
if USE_MEDIAN
    acc_med = movmedian(acc_g, MED_W, 'omitnan');
    per_med = movmedian(per_g, MED_W, 'omitnan');
else
    acc_med = acc_g;
    per_med = per_g;
end

acc_iir1 = iir1_nan(acc_med, ALPHA);
per_iir1 = iir1_nan(per_med, ALPHA);

acc_iir2 = iir2_cascade_nan(acc_med, ALPHA);
per_iir2 = iir2_cascade_nan(per_med, ALPHA);

%% ===================== METRICS =====================
% Compare after filtering
mask1 = isfinite(acc_iir1) & isfinite(per_iir1);
d1 = per_iir1(mask1) - acc_iir1(mask1);

mask2 = isfinite(acc_iir2) & isfinite(per_iir2);
d2 = per_iir2(mask2) - acc_iir2(mask2);

rmse1 = sqrt(mean(d1.^2)); mae1 = mean(abs(d1));
rmse2 = sqrt(mean(d2.^2)); mae2 = mean(abs(d2));

fprintf("\n===== FILTERED COMPARISON (median=%d, alpha=%.3f) =====\n", MED_W, ALPHA);
fprintf("IIR1: Samples=%d | RMSE=%.3f rpm | MAE=%.3f rpm\n", nnz(mask1), rmse1, mae1);
fprintf("IIR2(cascade): Samples=%d | RMSE=%.3f rpm | MAE=%.3f rpm\n", nnz(mask2), rmse2, mae2);

% Per-PWM stats (if segment exists)
if hasSeg
    fprintf("\n===== PER-PWM (FILTERED) =====\n");
    fprintf("%6s  %8s  %12s  %12s  %12s  %12s\n", "PWM","N","acc_iir1","per_iir1","acc_iir2","per_iir2");
    for i=1:numel(pwm_seq)
        pwm = pwm_seq(i);
        mm = (pwm_cmd == pwm);

        m1 = mm & isfinite(acc_iir1) & isfinite(per_iir1);
        m2 = mm & isfinite(acc_iir2) & isfinite(per_iir2);

        if nnz(m1) >= 5
            fprintf("%6d  %8d  %12.2f  %12.2f  ", pwm, nnz(m1), mean(acc_iir1(m1)), mean(per_iir1(m1)));
        else
            fprintf("%6d  %8d  %12s  %12s  ", pwm, nnz(m1), "-", "-");
        end

        if nnz(m2) >= 5
            fprintf("%12.2f  %12.2f\n", mean(acc_iir2(m2)), mean(per_iir2(m2)));
        else
            fprintf("%12s  %12s\n", "-", "-");
        end
    end
    fprintf("=============================================\n");
end

%% ===================== PLOTS =====================
% Plot 1: Period (raw/gated/median/IIR1/IIR2)
figure('Name', sprintf('Period filters (W=%d, alpha=%.2f)', MED_W, ALPHA));
plot(t(inPlot), per_g(inPlot),   'LineWidth', 1); hold on;
plot(t(inPlot), per_med(inPlot), 'LineWidth', 1);
plot(t(inPlot), per_iir1(inPlot),'LineWidth', 1);
plot(t(inPlot), per_iir2(inPlot),'LineWidth', 1);
grid on; xlabel('Time (s)'); ylabel('RPM');
title('Period-based RPM');
legend('per gated','per median','per IIR1','per IIR2','Location','best');

% Plot 2: Accumulation (raw/gated/median/IIR1/IIR2)
figure('Name', sprintf('Accum filters (W=%d, alpha=%.2f)', MED_W, ALPHA));
plot(t(inPlot), acc_g(inPlot),   'LineWidth', 1); hold on;
plot(t(inPlot), acc_med(inPlot), 'LineWidth', 1);
plot(t(inPlot), acc_iir1(inPlot),'LineWidth', 1);
plot(t(inPlot), acc_iir2(inPlot),'LineWidth', 1);
grid on; xlabel('Time (s)'); ylabel('RPM');
title('Accumulation-based RPM');
legend('acc gated','acc median','acc IIR1','acc IIR2','Location','best');

% Plot 3: Compare period vs accum after IIR1 and IIR2, optional PWM axis
figure('Name', 'Compare: period vs accum (IIR1 vs IIR2)');
plot(t(inPlot), per_iir1(inPlot), 'LineWidth', 1); hold on;
plot(t(inPlot), acc_iir1(inPlot), 'LineWidth', 1);
plot(t(inPlot), per_iir2(inPlot), 'LineWidth', 1);
plot(t(inPlot), acc_iir2(inPlot), 'LineWidth', 1);
grid on; xlabel('Time (s)'); ylabel('RPM');
title('Filtered comparison');
legend('per IIR1','acc IIR1','per IIR2','acc IIR2','Location','best');

if SHOW_PWM_AXIS && hasSeg
    yyaxis right;
    plot(t(inPlot), pwm_cmd(inPlot), 'LineWidth', 1);
    ylabel('PWM command');
    yyaxis left;
end

%% ===================== HELPERS =====================
function y = iir2_cascade_nan(x, alpha)
% 2nd-order IIR as cascade of two 1st-order IIRs (same alpha), NaN-safe.
    y1 = iir1_nan(x, alpha);
    y  = iir1_nan(y1, alpha);
end

function y = iir1_nan(x, alpha)
% 1st-order IIR: y[k] = (1-a)*y[k-1] + a*x[k], NaN-safe.
    y = nan(size(x));
    idx = find(isfinite(x), 1, 'first');
    if isempty(idx), return; end
    y(idx) = x(idx);
    for k = idx+1:numel(x)
        if isfinite(x(k))
            if ~isfinite(y(k-1))
                y(k) = x(k);
            else
                y(k) = (1-alpha)*y(k-1) + alpha*x(k);
            end
        else
            y(k) = NaN;
        end
    end
end
