function read_ultrasonic_continuous_bt()
% Continuous ultrasonic read via Bluetooth (SPP) based on ESP32 protocol:
% PC -> send 0xEE
% ESP32 -> sends 0x20 ACK (1 byte) then 2 bytes ultrasonic (uint16, big-endian)
% Press Enter to stop.

%% ===== USER CONFIG =====
btName    = "Behind the scream";  % change if needed
btChannel = 1;

T_req     = 0.06;   % seconds (match TIME_KICK_ULTRA ~ 60 ms)
t_ack     = 0.20;   % seconds timeout waiting ACK
t_data    = 0.20;   % seconds timeout waiting 2 bytes data
flushOnStart = true;

%% ===== CONNECT =====
bt = bluetooth(btName, btChannel);

if flushOnStart
    flush(bt);
end

fprintf("Connected to %s (ch=%d). Press Enter to stop.\n", btName, btChannel);

%% ===== STOP ON ENTER (non-blocking) =====
stopFlag = false;
fig = figure( ...
    'Name','Ultrasonic Stream (Press Enter to stop)', ...
    'NumberTitle','off', ...
    'KeyPressFcn', @(~,e) keyStop(e), ...
    'CloseRequestFcn', @(~,~) closeStop() );

% Simple live plot buffer
Nbuf = 300;
tbuf = nan(1,Nbuf);
ubuf = nan(1,Nbuf);
k = 0;

h = plot(nan, nan, '-'); grid on;
xlabel('t (s)'); ylabel('Ultrasonic (uint16)'); title('Ultrasonic Stream');

t0 = tic;

%% ===== MAIN LOOP =====
while ~stopFlag
    % 1) request ultrasonic
    write(bt, uint8(0xEE), "uint8");

    % 2) wait ACK 0x20
    if ~waitByte(bt, uint8(0x20), t_ack)
        % if no ACK, resync by flushing and continue
        flush(bt);
        pause(0.005);
        continue;
    end

    % 3) read 2 bytes ultrasonic (big-endian)
    data = readN(bt, 2, t_data);
    if numel(data) ~= 2
        flush(bt);
        pause(0.005);
        continue;
    end
    ultra = uint16(data(1)) * 256 + uint16(data(2));

    % 4) log + plot
    k = k + 1;
    idx = mod(k-1, Nbuf) + 1;
    tbuf(idx) = toc(t0);
    ubuf(idx) = double(ultra);

    % plot in time order (handle wrap)
    if k < Nbuf
        set(h, 'XData', tbuf(1:k), 'YData', ubuf(1:k));
        xlim([max(0, tbuf(1) - 0.1), tbuf(max(1,k)) + 0.1]);
    else
        order = [idx+1:Nbuf, 1:idx];
        set(h, 'XData', tbuf(order), 'YData', ubuf(order));
        xlim([tbuf(order(1)), tbuf(order(end))]);
    end
    drawnow limitrate;

    % 5) pacing
    pause(T_req);
end

% cleanup
if isvalid(bt); clear bt; end
if isvalid(fig); close(fig); end
fprintf("Stopped.\n");

%% ===== NESTED CALLBACKS =====
    function keyStop(e)
        if strcmp(e.Key, "return") || strcmp(e.Key, "enter")
            stopFlag = true;
        end
    end

    function closeStop()
        stopFlag = true;
        delete(fig);
    end
end

%% ===== HELPERS =====
function ok = waitByte(bt, target, timeout_s)
t0 = tic;
ok = false;

while toc(t0) < timeout_s
    if bt.NumBytesAvailable >= 1
        b = read(bt, 1, "uint8");
        if b == target
            ok = true;
            return;
        end
        % discard other bytes and keep searching (resync)
    else
        pause(0.001);
    end
end
end

function out = readN(bt, N, timeout_s)
t0 = tic;
out = uint8([]);

while toc(t0) < timeout_s
    nAvail = bt.NumBytesAvailable;
    if nAvail > 0
        nRead = min(N - numel(out), nAvail);
        out = [out; read(bt, nRead, "uint8")]; %#ok<AGROW>
        if numel(out) >= N
            return;
        end
    else
        pause(0.001);
    end
end
end
