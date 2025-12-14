% =========================
% encoder_acquisition.m  (SCRIPT)
% FIX: ensure raw is always a ROW vector (1xN) to avoid vertcat mismatch.
% =========================

clear; clc;

% ---- User config ----
portName = "COM9";        % change
baud     = 2000000;       % must match ESP32
tRun     = 4.0;           % seconds run per step
tRest    = 1.0;           % seconds rest per step
pwms     = uint8(25:10:255);

% ---- Serial open ----
s = serialport(portName, baud, "Timeout", 0.2);
flush(s);

% Clear ESP buffer
write(s, uint8('C'), "uint8");
pause(0.05);
flush(s);

% ---- Accumulators (RAW IS ROW) ----
raw = uint8([]); % 1xN
allMarkers = struct("t_us", {}, "pwm", {});
allEvents  = struct("t_us", {}, "flags", {});

% ---- Main stepping loop ----
for k = 1:numel(pwms)
  pwm = pwms(k);

  write(s, uint8(['S', pwm]), "uint8");
  [raw, m1, e1] = readFor(s, raw, tRun);
  allMarkers = [allMarkers, m1]; %#ok<AGROW>
  allEvents  = [allEvents,  e1]; %#ok<AGROW>

  write(s, uint8('P'), "uint8");
  [raw, m2, e2] = readFor(s, raw, tRest);
  allMarkers = [allMarkers, m2]; %#ok<AGROW>
  allEvents  = [allEvents,  e2]; %#ok<AGROW>
end

[raw, m3, e3] = readFor(s, raw, 0.5);
allMarkers = [allMarkers, m3]; %#ok<AGROW>
allEvents  = [allEvents,  e3]; %#ok<AGROW>

clear s;

% ---- Build per-step segments using markers ----
if isempty(allMarkers)
  error("No step markers received (0xE0).");
end

[~, ordM] = sort([allMarkers.t_us]);
allMarkers = allMarkers(ordM);

isStart = arrayfun(@(x) any(x.pwm == pwms), allMarkers);
startMarkers = allMarkers(isStart);
if isempty(startMarkers)
  error("No start markers with requested PWM values were received.");
end

[~, ordE] = sort([allEvents.t_us]);
allEvents = allEvents(ordE);

evt_t  = [allEvents.t_us];
evt_f  = uint8([allEvents.flags]);

evt_ch = bitand(evt_f, 1);
evt_a  = bitand(evt_f, 2) ~= 0;
evt_b  = bitand(evt_f, 4) ~= 0;

steps = repmat(struct("pwm",0,"t0_us",0,"t_us",[],"ch",[],"a",[],"b",[]), 1, numel(startMarkers));
for i = 1:numel(startMarkers)
  t0 = double(startMarkers(i).t_us);
  pwm = startMarkers(i).pwm;
  if i < numel(startMarkers)
    t1 = double(startMarkers(i+1).t_us);
  else
    t1 = inf;
  end
  in = (evt_t >= t0) & (evt_t < t1);

  steps(i).pwm   = uint8(pwm);
  steps(i).t0_us = t0;
  steps(i).t_us  = evt_t(in) - t0;
  steps(i).ch    = evt_ch(in);
  steps(i).a     = evt_a(in);
  steps(i).b     = evt_b(in);
end

encoderSignalsPerSteps = steps; %#ok<NASGU>
save("encoderSignalsPerSteps.mat", "encoderSignalsPerSteps");

figure("Name","Encoder ISR timestamps per PWM step");
n = numel(steps);
nr = ceil(n/2); nc = 2;
for i = 1:n
  t = steps(i).t_us * 1e-6;
  ch = steps(i).ch;

  subplot(nr, nc, i);
  hold on;
  plot(t(ch==0), zeros(sum(ch==0),1), '.');
  plot(t(ch==1), ones(sum(ch==1),1),  '.');
  hold off;
  yticks([0 1]); yticklabels(["A","B"]);
  xlabel("Time since step start (s)");
  ylabel("Channel");
  title(sprintf("PWM = %d, events = %d", steps(i).pwm, numel(t)));
  grid on;
end

% =========================
% Local functions (end of file)
% =========================
function [markers, events, rawOut] = parseStream(rawIn)
  rawOut  = rawIn; % row
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
  rawOut = rawOut(i:end); % row tail
end

function [rawNew, markersAll, eventsAll] = readFor(s, rawCur, duration_s)
  tStart = tic;
  markersAll = struct("t_us", {}, "pwm", {});
  eventsAll  = struct("t_us", {}, "flags", {});
  rawNew = rawCur; % row

  while toc(tStart) < duration_s
    n = s.NumBytesAvailable;
    if n > 0
      bytes = read(s, n, "uint8");
      bytes = bytes(:).';             % FORCE ROW (1xN)
      rawNew = [rawNew, bytes];       % HORIZONTAL CONCAT (row-safe)
      [m, e, rawNew] = parseStream(rawNew);
      if ~isempty(m), markersAll = [markersAll, m]; end %#ok<AGROW>
      if ~isempty(e), eventsAll  = [eventsAll,  e]; end %#ok<AGROW>
    else
      pause(0.002);
    end
  end
end
