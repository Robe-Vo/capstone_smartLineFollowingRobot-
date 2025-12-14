% motor_profile_acq_continuous.m
% Control motor + continuous encoder acquisition
% Profile:
%  (1) Step: 25->255 step=5, hold 5s each, then stop 1s
%  (2) Ramp: 0->255 in 5s, then 255->0 in 5s
%
% Assumes ESP32 protocol:
%   'C' clear buffer
%   ['S', pwm] set PWM
%   'P' stop (PWM=0)
% Stream frames:
%   0xE0 [t_us uint32 little-endian] [pwm uint8]   (optional marker)
%   0xE1 [t_us uint32 little-endian] [flags uint8] (event)

clear; clc;

%% ===== User config =====
portName = "COM9";
baud     = 2000000;

dtCmd    = 0.050;   % send/update PWM every 50 ms (continuous control)
rxPause  = 0.001;   % small pause when no bytes

% Step profile
pwmStepStart = 25;
pwmStepEnd   = 255;
pwmStepDelta = 5;
tHoldStep    = 5.0;
tHoldStop    = 1.0;

% Ramp profile
tRampUp   = 5.0;
tRampDown = 5.0;

%% ===== Open serial =====
s = serialport(portName, baud, "Timeout", 0.2);
flush(s);

write(s, uint8('C'), "uint8");
pause(0.05);
flush(s);

%% ===== Accumulators =====
raw = uint8([]);  % row buffer
allMarkers = struct("t_us", {}, "pwm", {});
allEvents  = struct("t_us", {}, "flags", {});

% Command log (PC time base)
cmdLog = struct("t_pc", {}, "pwm", {});

tGlobal = tic;

%% ===== Helpers =====
sendPWM(s, uint8(0)); % ensure known state

%% ===== 1) STEP sequence =====
pwms = uint8(pwmStepStart:pwmStepDelta:pwmStepEnd);

for k = 1:numel(pwms)
    pwm = pwms(k);

    % hold pwm for tHoldStep
    [raw, m, e, cmdLog] = runSegment(s, raw, tHoldStep, dtCmd, ...
        @(t) pwm, tGlobal, cmdLog, rxPause);
    allMarkers = [allMarkers, m]; %#ok<AGROW>
    allEvents  = [allEvents,  e]; %#ok<AGROW>

    % stop for tHoldStop
    [raw, m, e, cmdLog] = runSegment(s, raw, tHoldStop, dtCmd, ...
        @(t) uint8(0), tGlobal, cmdLog, rxPause);
    allMarkers = [allMarkers, m]; %#ok<AGROW>
    allEvents  = [allEvents,  e]; %#ok<AGROW>
end

%% ===== 2) RAMP up then down =====
% ramp up: 0 -> 255
[raw, m, e, cmdLog] = runSegment(s, raw, tRampUp, dtCmd, ...
    @(t) uint8(round(255 * min(max(t/tRampUp,0),1))), tGlobal, cmdLog, rxPause);
allMarkers = [allMarkers, m]; %#ok<AGROW>
allEvents  = [allEvents,  e]; %#ok<AGROW>

% ramp down: 255 -> 0
[raw, m, e, cmdLog] = runSegment(s, raw, tRampDown, dtCmd, ...
    @(t) uint8(round(255 * (1 - min(max(t/tRampDown,0),1)))), tGlobal, cmdLog, rxPause);
allMarkers = [allMarkers, m]; %#ok<AGROW>
allEvents  = [allEvents,  e]; %#ok<AGROW>

%% ===== Stop & close =====
write(s, uint8('P'), "uint8");
pause(0.1);
flush(s);
clear s;

%% ===== Save =====
encoderStream = struct();
encoderStream.allMarkers = allMarkers;
encoderStream.allEvents  = allEvents;
encoderStream.cmdLog     = cmdLog;
encoderStream.meta = struct( ...
    "dtCmd", dtCmd, ...
    "step",  struct("start",pwmStepStart,"end",pwmStepEnd,"delta",pwmStepDelta,"hold_s",tHoldStep,"stop_s",tHoldStop), ...
    "ramp",  struct("up_s",tRampUp,"down_s",tRampDown) );

save("encoderStream_continuous.mat","encoderStream");

disp("Saved: encoderStream_continuous.mat");

%% ========================= Local functions =========================
function sendPWM(s, pwm)
    if pwm == 0
        write(s, uint8('P'), "uint8");
    else
        write(s, uint8(['S', uint8(pwm)]), "uint8");
    end
end

function [rawNew, markersAll, eventsAll, cmdLog] = runSegment( ...
        s, rawCur, duration_s, dtCmd, pwmFcn, tGlobal, cmdLog, rxPause)

    markersAll = struct("t_us", {}, "pwm", {});
    eventsAll  = struct("t_us", {}, "flags", {});
    rawNew = rawCur;

    tSeg = tic;
    tNextCmd = 0;

    lastPwm = uint8(255); % dummy init to force first send

    while toc(tSeg) < duration_s
        % ---- send PWM at dtCmd ----
        tNow = toc(tSeg);
        if tNow >= tNextCmd
            pwm = pwmFcn(tNow);
            pwm = uint8(max(0, min(255, double(pwm))));

            if pwm ~= lastPwm
                sendPWM(s, pwm);
                lastPwm = pwm;

                cmdLog(end+1).t_pc = toc(tGlobal); %#ok<AGROW>
                cmdLog(end).pwm    = pwm;
            end
            tNextCmd = tNextCmd + dtCmd;
        end

        % ---- read stream ----
        n = s.NumBytesAvailable;
        if n > 0
            bytes = read(s, n, "uint8");
            bytes = bytes(:).'; % row
            rawNew = [rawNew, bytes]; %#ok<AGROW>

            [m, e, rawNew] = parseStream(rawNew);

            if ~isempty(m), markersAll = [markersAll, m]; end %#ok<AGROW>
            if ~isempty(e), eventsAll  = [eventsAll,  e]; end %#ok<AGROW>
        else
            pause(rxPause);
        end
    end
end

function [markers, events, rawOut] = parseStream(rawIn)
    rawOut  = rawIn;
    markers = struct("t_us", {}, "pwm", {});
    events  = struct("t_us", {}, "flags", {});
    i = 1;

    while i <= numel(rawOut)
        hdr = rawOut(i);

        if hdr == hex2dec('E0')
            if i + 5 <= numel(rawOut)
                t = typecast(uint8(rawOut(i+1:i+4)), "uint32");
                p = rawOut(i+5);
                markers(end+1).t_us = double(t); %#ok<AGROW>
                markers(end).pwm    = uint8(p);
                i = i + 6;
            else
                break;
            end

        elseif hdr == hex2dec('E1')
            if i + 5 <= numel(rawOut)
                t = typecast(uint8(rawOut(i+1:i+4)), "uint32");
                f = rawOut(i+5);
                events(end+1).t_us = double(t); %#ok<AGROW>
                events(end).flags  = uint8(f);
                i = i + 6;
            else
                break;
            end
        else
            i = i + 1;
        end
    end

    rawOut = rawOut(i:end);
end
