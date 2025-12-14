% encoder_compare_filters_continuous.m
% Compare RAW/MEDIAN/IIR1/IIR2 on continuous encoder stream
% Loads: encoderStream_continuous.mat

clear; clc;

S = load("encoderStream_continuous.mat");
encoderStream = S.encoderStream;

events = encoderStream.allEvents;
cmdLog = encoderStream.cmdLog;

assert(~isempty(events), "No encoder events.");

%% ===== User params =====
useChannel = "A";   % "A" or "B" (không dùng BOTH để tránh trộn period)
modeFreq   = "accumulate"; % "accumulate" or "period"

Tw   = 0.01;    % window for accumulate (s)
medN = 11;      % odd
a1   = 0.05;    % IIR1 alpha
a2   = 0.05;    % IIR2 alpha
b2   = 0.00;    % IIR2 beta

t0_view = 0;    % seconds (relative to first event)
t1_view = inf;

%% ===== Decode event arrays =====
evt_t  = double([events.t_us]) * 1e-6; % seconds (device time base)
evt_f  = uint8([events.flags]);

evt_ch = bitand(evt_f, 1);                 % 0:A, 1:B
tA = evt_t(evt_ch==0);
tB = evt_t(evt_ch==1);

switch upper(useChannel)
    case "A", t_evt = tA(:);
    case "B", t_evt = tB(:);
    otherwise, error("useChannel must be 'A' or 'B'.");
end

t_evt = sort(t_evt);

% Use time relative to first event for plotting
t_ref = t_evt(1);
t_evt = t_evt - t_ref;

%% ===== Compute frequency estimate =====
if strcmpi(modeFreq, "accumulate")
    [tRaw, fRaw] = windowFreq(t_evt, Tw);
elseif strcmpi(modeFreq, "period")
    dt = diff(t_evt); dt = max(dt, eps);
    tRaw = t_evt(2:end);
    fRaw = 1 ./ dt;
else
    error("modeFreq must be 'accumulate' or 'period'.");
end

%% ===== Filters =====
fMed  = medfilt1(fRaw, makeOdd(medN), "truncate");
fIIR1 = iir1(fRaw, a1);
fIIR2 = iir2_alphaBeta(fRaw, a2, b2);

%% ===== Plot =====
figure("Name","Encoder Frequency Compare (continuous)");
hold on; grid on;

plot(tRaw, fRaw,  "-", "LineWidth", 1);
plot(tRaw, fMed,  "-", "LineWidth", 1);
plot(tRaw, fIIR1, "-", "LineWidth", 1);
plot(tRaw, fIIR2, "-", "LineWidth", 1);

xlabel("t (s, relative to first event)");
ylabel("event-rate (Hz)");
title(sprintf("Channel %s | Mode %s | Tw=%.3g | medN=%d | a1=%.3f | a2=%.3f | b2=%.3f", ...
    upper(useChannel), modeFreq, Tw, medN, a1, a2, b2));

legend("RAW","MEDIAN","IIR1","IIR2","Location","northeast");

if isfinite(t1_view)
    xlim([t0_view t1_view]);
else
    xlim([t0_view max(tRaw)]);
end

%% ===== Overlay PWM command (PC time base) =====
% cmdLog uses PC time; it is not aligned to device t_us.
% Plot it on a secondary axis only for qualitative comparison.
if ~isempty(cmdLog)
    yyaxis right;
    tCmd = [cmdLog.t_pc];     % seconds since start of script
    uCmd = double([cmdLog.pwm]);

    plot(tCmd, uCmd, "-", "LineWidth", 1);
    ylabel("PWM command (0..255)");
    legend("RAW","MEDIAN","IIR1","IIR2","PWM cmd","Location","northeast");
end

%% ===== Local functions =====
function n = makeOdd(n)
    n = round(n);
    if n < 1, n = 1; end
    if mod(n,2)==0, n = n + 1; end
end

function [t_mid, f_win] = windowFreq(t_evt, Tw)
    t_evt = t_evt(:);
    if numel(t_evt) < 2
        t_mid = t_evt;
        f_win = zeros(size(t_evt));
        return;
    end
    edges = t_evt(1):Tw:(t_evt(end)+Tw);
    if numel(edges) < 2
        t_mid = t_evt(1);
        f_win = 0;
        return;
    end
    c = histcounts(t_evt, edges);
    t_mid = edges(1:end-1) + Tw/2;
    f_win = c(:)' / Tw;
end

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
