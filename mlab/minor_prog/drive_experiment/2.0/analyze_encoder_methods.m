% analyze_encoder_methods.m  (FULL FIXED VERSION)
% Phân tích dữ liệu để chọn:
%  (1) phương pháp đo (accumulate / period / fixed-dt count)
%  (2) kênh (A/B)
%  (3) lọc (none / median / IIR1 / IIR2)
% theo tiêu chí định lượng: corr(PWM), noise, jerk, outlier, lag.
%
% INPUT: encoderStream_continuous.mat
%   encoderStream.allEvents: struct(t_us, flags)
%   encoderStream.cmdLog:    struct(t_pc, pwm)
%
% OUTPUT:
%   results (table) + figure top-N
%   analysis_results_methods.mat

clear; clc;

%% ===== Load =====
S = load("encoderStream_continuous.mat");
encoderStream = S.encoderStream;

events = encoderStream.allEvents;
cmdLog = encoderStream.cmdLog;

assert(~isempty(events), "No encoder events.");
assert(~isempty(cmdLog), "No cmdLog (PWM).");

%% ===== User knobs =====
opt.dtGrid      = 0.010;    % uniform grid (s)
opt.maxLag_s    = 1.0;      % max lag for alignment (s)
opt.Tw_list     = [0.005 0.01 0.02];      % accumulate window (s)
opt.dtCount_list= [0.01 0.02 0.05];       % fixed-dt count window (s)
opt.medN_list   = [1 5 11 21];            % 1 = none
opt.a1_list     = [0.02 0.05 0.10];
opt.a2_list     = [0.02 0.05 0.10];
opt.b2_list     = [0.00 0.10 0.20];

opt.topN_plot   = 6;
opt.clipPrct    = 0.5;      % winsorize %
opt.minSamples  = 200;      % min finite samples on grid to score

%% ===== Decode events -> tA, tB (FORCE COLUMN) =====
[evt_t_s, evt_ch] = decodeEvents(events);     % seconds (device time), ch 0=A 1=B
evt_t_s = evt_t_s(:);
evt_ch  = evt_ch(:);

tA = evt_t_s(evt_ch==0);  tA = tA(:);
tB = evt_t_s(evt_ch==1);  tB = tB(:);

tAll = [tA; tB];
assert(~isempty(tAll), "No events on both channels.");

t0dev = min(tAll);
tA = tA - t0dev;
tB = tB - t0dev;

%% ===== PWM time series (PC time) =====
tPWM = [cmdLog.t_pc]; tPWM = tPWM(:);
uPWM = double([cmdLog.pwm]); uPWM = uPWM(:);

tPWM = tPWM - tPWM(1);

% overlap duration: conservative
tMaxDev = max([tA; tB]);               % device span
tMaxPWM = max(tPWM);                    % pc span
tMax = min(tMaxPWM, tMaxDev);
assert(isfinite(tMax) && tMax > 0, "Invalid overlap duration.");

tGrid = (0:opt.dtGrid:tMax).';
uGrid = interp1(tPWM, uPWM, tGrid, "previous", "extrap");
uGrid = fillmissing(uGrid,"previous");
uN = zscore(uGrid);

%% ===== Enumerate methods =====
rows = {};
channels = ["A","B"];
measureModes = ["accumulate","period","count_dt"];

for ch = channels
    if ch=="A"
        tEvt = tA;
    else
        tEvt = tB;
    end

    tEvt = sort(tEvt(:));
    if numel(tEvt) < 10
        continue;
    end

    for mode = measureModes
        switch mode
            case "accumulate"
                for Tw = opt.Tw_list
                    [tM, yRaw] = measure_accumulate(tEvt, Tw);
                    rows = [rows; runAllFilters(ch, mode, struct("Tw",Tw), tM, yRaw, tGrid, uN, opt)]; %#ok<AGROW>
                end

            case "period"
                [tM, yRaw] = measure_period(tEvt);
                rows = [rows; runAllFilters(ch, mode, struct(), tM, yRaw, tGrid, uN, opt)]; %#ok<AGROW>

            case "count_dt"
                for dtC = opt.dtCount_list
                    [tM, yRaw] = measure_count_dt(tEvt, dtC);
                    rows = [rows; runAllFilters(ch, mode, struct("dtC",dtC), tM, yRaw, tGrid, uN, opt)]; %#ok<AGROW>
                end
        end
    end
end

if isempty(rows)
    error("No valid methods evaluated (check data length / minSamples).");
end

results = vertcat(rows{:});

%% ===== Rank =====
results = sortrows(results, "score_total", "ascend");
disp(results(1:min(height(results),20), :));

%% ===== Plot top-N =====
N = min(opt.topN_plot, height(results));
figure("Name","Top methods (aligned to PWM)");
tiledlayout(N,1,"Padding","compact","TileSpacing","compact");

for i = 1:N
    r = results(i,:);
    nexttile;
    plot(tGrid, uN, "-"); hold on; grid on;
    plot(tGrid, r.y_aligned{1}, "-");
    legend("PWM (zscore)","speed_est (zscore)","Location","northeast");
    title(sprintf("#%d | %s | %s | %s | %s | lag=%.3fs | score=%.3f", ...
        i, r.channel, r.measure, r.filter, r.cfg_json, r.lag_s, r.score_total));
    xlabel("t (s)"); ylabel("z");
end

%% ===== Save =====
save("analysis_results_methods.mat","results","opt");

%% ===================== Helper: evaluate filters for one raw series =====================
function rows = runAllFilters(ch, mode, cfg, tM, yRaw, tGrid, uN, opt)
rows = {};

[tM, yRaw] = ensure_col(tM, yRaw);
[yGrid, ok] = resample_to_grid(tM, yRaw, tGrid);
if ~ok || nnz(isfinite(yGrid)) < opt.minSamples
    return;
end

yGrid = winsorize(yGrid, opt.clipPrct);
yGrid = fillmissing(yGrid,"linear","EndValues","nearest");
yZ = zscore(yGrid);

[lag_s, yAligned, corrMax] = align_by_xcorr(yZ, uN, opt.dtGrid, opt.maxLag_s);

base = score_metrics(yAligned, uN, corrMax);

cfgStr = jsonencode(cfg);

% none
rows{end+1} = pack_row(ch, mode, cfgStr, "none", lag_s, base, yAligned); %#ok<AGROW>

% median only
for medN = opt.medN_list
    if medN <= 1, continue; end
    yF = medfilt1(yAligned, makeOdd(medN), "truncate");
    sc = score_metrics(yF, uN, corrcoef_safe(yF,uN));
    rows{end+1} = pack_row(ch, mode, cfgStr, sprintf("median(N=%d)",makeOdd(medN)), lag_s, sc, yF); %#ok<AGROW>
end

% IIR1
for a1 = opt.a1_list
    yF = iir1(yAligned, a1);
    sc = score_metrics(yF, uN, corrcoef_safe(yF,uN));
    rows{end+1} = pack_row(ch, mode, cfgStr, sprintf("iir1(a=%.3f)",a1), lag_s, sc, yF); %#ok<AGROW>
end

% IIR2 alpha-beta
for a2 = opt.a2_list
    for b2 = opt.b2_list
        yF = iir2_alphaBeta(yAligned, a2, b2);
        sc = score_metrics(yF, uN, corrcoef_safe(yF,uN));
        rows{end+1} = pack_row(ch, mode, cfgStr, sprintf("iir2(a=%.3f,b=%.3f)",a2,b2), lag_s, sc, yF); %#ok<AGROW>
    end
end
end

function r = pack_row(ch, mode, cfgStr, filtName, lag_s, sc, yAligned)
% score_total: weighted sum (tunable)
w.corr = 1.0;   % want HIGH corr -> use (1-corr)
w.noise= 0.7;
w.jerk = 0.7;
w.out  = 0.6;
w.lag  = 0.2;

score_total = w.corr*(1-sc.corr) + w.noise*sc.noise + w.jerk*sc.jerk + w.out*sc.outlier + w.lag*abs(lag_s);

r = table( ...
    string(ch), string(mode), string(filtName), string(cfgStr), ...
    lag_s, sc.corr, sc.noise, sc.jerk, sc.outlier, score_total, ...
    {yAligned}, ...
    'VariableNames', {'channel','measure','filter','cfg_json','lag_s','corr','noise','jerk','outlier','score_total','y_aligned'} );
end

%% ===================== Metrics =====================
function sc = score_metrics(y, uN, corrVal)
y = fillmissing(y,"linear","EndValues","nearest");
y = zscore(y);

dy = diff(y);
noise = robustStd(dy);

d2 = diff(dy);
jerk = robustStd(d2);

thr = 4*robustStd(y);
outlier = mean(abs(y) > thr);

if ~isfinite(corrVal)
    corrVal = corrcoef_safe(y,uN);
end

sc = struct("corr",corrVal, "noise",noise, "jerk",jerk, "outlier",outlier);
end

function s = robustStd(x)
x = x(isfinite(x));
if numel(x) < 10
    s = inf;
    return;
end
s = 1.4826 * median(abs(x - median(x)));
if s == 0
    s = std(x);
end
end

function c = corrcoef_safe(a,b)
a = a(:); b = b(:);
m = isfinite(a) & isfinite(b);
if nnz(m) < 20
    c = NaN;
    return;
end
aa = a(m); bb = b(m);
aa = aa - mean(aa); bb = bb - mean(bb);
c = (aa.'*bb) / sqrt((aa.'*aa)*(bb.'*bb) + eps);
end

%% ===================== Alignment =====================
function [lag_s, yAligned, corrMax] = align_by_xcorr(y, u, dt, maxLag_s)
L = round(maxLag_s/dt);

y0 = y - mean(y,"omitnan");
u0 = u - mean(u,"omitnan");

[c,lags] = xcorr(y0, u0, L, "coeff");
[~,ix] = max(c);
lag = lags(ix);
lag_s = lag * dt;

yAligned = shift_series(y, -lag);
corrMax = c(ix);
end

function y2 = shift_series(y, k)
y2 = nan(size(y));
n = numel(y);

if k == 0
    y2 = y;
elseif k > 0
    y2(1+k:n) = y(1:n-k);
else
    kk = -k;
    y2(1:n-kk) = y(1+kk:n);
end

y2 = fillmissing(y2,"linear","EndValues","nearest");
y2 = zscore(y2);
end

%% ===================== Resample + Winsorize =====================
function [yGrid, ok] = resample_to_grid(t, y, tGrid)
t = t(:); y = y(:);
m = isfinite(t) & isfinite(y);
t = t(m); y = y(m);

ok = false;
yGrid = nan(size(tGrid));
if numel(t) < 5
    return;
end

[t, idx] = sort(t);
y = y(idx);
[t, uniqIdx] = unique(t,"stable");
y = y(uniqIdx);

if numel(t) < 5
    return;
end

yGrid = interp1(t, y, tGrid, "linear", NaN);
yGrid = fillmissing(yGrid,"linear","EndValues","nearest");
ok = true;
end

function y = winsorize(y, prct)
if prct <= 0
    return;
end
a = prctile(y, prct);
b = prctile(y, 100-prct);
y = min(max(y, a), b);
end

function n = makeOdd(n)
n = round(n);
if n < 1, n = 1; end
if mod(n,2)==0, n = n + 1; end
end

function [t, y] = ensure_col(t, y)
t = t(:);
y = y(:);
end

%% ===================== Measurement methods =====================
function [t_mid, f_win] = measure_accumulate(t_evt, Tw)
t_evt = t_evt(:);
edges = t_evt(1):Tw:(t_evt(end)+Tw);
if numel(edges) < 2
    t_mid = t_evt(1);
    f_win = 0;
    return;
end
c = histcounts(t_evt, edges);
t_mid = edges(1:end-1) + Tw/2;
f_win = c(:) / Tw;
end

function [tP, fP] = measure_period(t_evt)
t_evt = t_evt(:);
dt = diff(t_evt);
dt = max(dt, eps);
tP = t_evt(2:end);
fP = 1 ./ dt;
end

function [t_mid, f_dt] = measure_count_dt(t_evt, dtC)
[t_mid, f_dt] = measure_accumulate(t_evt, dtC);
end

%% ===================== Decode events =====================
function [evt_t_s, evt_ch] = decodeEvents(events)
evt_t_us = double([events.t_us]); evt_t_us = evt_t_us(:);
evt_t_s  = evt_t_us * 1e-6;

evt_f  = uint8([events.flags]); evt_f = evt_f(:);
evt_ch = bitand(evt_f, 1); % 0=A 1=B
evt_ch = double(evt_ch(:));
end

%% ===================== Filters =====================
function y = iir1(x, alpha)
y = zeros(size(x));
y(1) = x(1);
for k = 2:numel(x)
    y(k) = y(k-1) + alpha*(x(k) - y(k-1));
end
end

function y = iir2_alphaBeta(x, alpha, beta)
y = zeros(size(x));
y(1) = x(1);
if numel(x) >= 2
    y(2) = (1-alpha)*y(1) + alpha*x(2);
end
for k = 3:numel(x)
    y(k) = (1-alpha)*y(k-1) + alpha*x(k) + beta*(y(k-1) - y(k-2));
end
end
