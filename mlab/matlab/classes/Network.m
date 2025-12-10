classdef Network < handle
    properties
        conn % struct: fields: type, dev, (ip,port) for UDP
    end
    
    methods
        %% Constructor
        function obj = Network()
            obj.conn = [];
        end

        %% ===== Connect to Bluetooth =====
        function connectBluetooth(obj, deviceName, channelID)
            % If existing connection -> disconnect first
            if ~isempty(obj.conn)
                obj.disconnect();
            end

            fprintf("Connecting BT %s (ch %d)...\n", deviceName, channelID);
            bt = bluetooth(deviceName, channelID);
            bt.Timeout = 5;   % seconds

            obj.conn = struct( ...
                "type", "bt", ...
                "dev",  bt);
        end

        %% ===== Connect to UDP =====
        function connectUDP(obj, ip, port)
            if ~isempty(obj.conn)
                obj.disconnect();
            end

            fprintf("Opening UDP %s:%d ...\n", ip, port);
            u = udpport("datagram", "IPV4");
            u.Timeout = 5;

            obj.conn = struct( ...
                "type", "udp", ...
                "dev",  u, ...
                "ip",   ip, ...
                "port", port);
        end

        %% ===== Connect to TCP =====
        function connectTCP(obj, ip, port)
            if ~isempty(obj.conn)
                obj.disconnect();
            end

            fprintf("Connecting TCP %s:%d...\n", ip, port);
            t = tcpclient(ip, port, "Timeout", 5);

            obj.conn = struct( ...
                "type", "tcp", ...
                "dev",  t);
        end

        %% ===== Send raw bytes (scalar or vector) =====
        function sendByte(obj, data)
            if isempty(obj.conn)
                return;
            end

            c = obj.conn;
            b = uint8(data);

            if c.type == "bt" || c.type == "tcp"
                write(c.dev, b, "uint8");
            elseif c.type == "udp"
                write(c.dev, b, "uint8", c.ip, c.port);
            end
        end

        %% ===== Receive a single byte (non-blocking, [] if nothing) =====
        function rx = recvByte(obj)
            if isempty(obj.conn)
                rx = [];
                return;
            end

            rx = [];
            c = obj.conn;

            if c.type == "bt" || c.type == "tcp"
                if c.dev.NumBytesAvailable > 0
                    rx = read(c.dev, 1, "uint8");
                end

            elseif c.type == "udp"
                u = c.dev;
                if u.NumDatagramsAvailable > 0
                    d = read(u, 1, "uint8");
                    if isa(d, 'udpport.datagram.Datagram')
                        p = d.Data;
                    else
                        p = d;
                    end
                    if ~isempty(p)
                        rx = p(1);
                    end
                end
            end
        end

        %% ===== Blocking read of a full frame =====
        function frame = readFrameBlocking(obj, frameLen, timeout)
            % frame = readFrameBlocking(obj, frameLen, timeout)
            %  Read exactly frameLen bytes using recvByte().
            %  Blocks up to 'timeout' seconds. Returns [] if timed out.

            if nargin < 3
                timeout = 1.0;
            end

            frame = uint8([]);
            t0 = tic;

            while numel(frame) < frameLen
                if toc(t0) > timeout
                    frame = [];
                    return;
                end

                b = obj.recvByte();
                if isempty(b)
                    pause(0.001);
                    continue;
                end

                frame = [frame; b]; %#ok<AGROW>
            end
        end

        %% ===== High-level: receive and decode sensor frame =====
        function [ok, lineRaw, ultra, mpu, encCount, encSpeed] = recvSensorFrame(obj, timeout)
            % [ok, lineRaw, ultra, mpu, encCount, encSpeed] = recvSensorFrame(obj, timeout)
            %
            % Frame (22 bytes):
            %   1..5   : line sensors (uint8)
            %   6..7   : ultrasonic (uint16, big-endian)
            %   8..19  : MPU signals (12 bytes, e.g. int16[6])
            %   20     : encoder counter (uint8)
            %   21..22 : encoder speed (uint16, rpm*100, big-endian)

            FRAME_LEN = 22;

            if nargin < 2
                timeout = 1.0;
            end

            ok        = false;
            lineRaw   = [];
            ultra     = [];
            mpu       = [];
            encCount  = [];
            encSpeed  = [];

            frame = obj.readFrameBlocking(FRAME_LEN, timeout);
            if isempty(frame)
                return;
            end

            % line sensors
            lineRaw = double(frame(1:5)).';

            % ultrasonic
            hiU   = uint16(frame(6));
            loU   = uint16(frame(7));
            ultra = bitor(bitshift(hiU, 8), loU);   % raw uint16 distance

            % MPU signals (assume 6 x int16 big-endian)
            mpuBytes = frame(8:19);
            mpu      = zeros(1, 6);
            for k = 1:6
                hi  = uint16(mpuBytes(2*k-1));
                lo  = uint16(mpuBytes(2*k));
                v16 = bitor(bitshift(hi, 8), lo);
                mpu(k) = double(typecast(v16, 'int16'));
            end

            % encoder counter (pulses this frame)
            encCount = double(frame(20));

            % encoder speed (rpm * 100)
            hiS     = uint16(frame(21));
            loS     = uint16(frame(22));
            spd_q   = bitor(bitshift(hiS, 8), loS);
            encSpeed = double(spd_q) / 100.0;  % rpm

            ok = true;
        end

        %% ===== Wait for ACK byte 0x20 (fixed) =====
        function ok = waitAck(obj, timeout)
            % ok = waitAck(obj, timeout)
            %   Waits for byte 0x20 up to 'timeout' seconds.

            if nargin < 2
                timeout = 1.0;
            end

            ok = false;
            if isempty(obj.conn)
                return;
            end

            target = uint8(hex2dec('20'));
            t0 = tic;
            while toc(t0) < timeout
                b = obj.recvByte();
                if isempty(b)
                    pause(0.001);
                    continue;
                end
                if b == target
                    ok = true;
                    return;
                end
            end
        end

        %% ===== Send robot control frame (cmd, speed, angle) =====
        function sendControl(obj, speed, angle, cmd)
            % sendControl(obj, speed, angle)
            % sendControl(obj, speed, angle, cmd)
            %
            % Builds [cmd, speed, angle_hi, angle_lo].
            % Default cmd = 0xF1 (forward + steering).

            if nargin < 4
                cmd = hex2dec('F1');
            end

            if isempty(obj.conn)
                return;
            end

            speed_u8 = uint8(speed);
            angle_u16 = uint16(angle);

            angle_hi = uint8(bitshift(angle_u16, -8));
            angle_lo = uint8(bitand(angle_u16, 255));

            pkt = uint8([uint8(cmd); speed_u8; angle_hi; angle_lo]);
            obj.sendByte(pkt);
        end

        %% ===== Disconnect =====
        function disconnect(obj)
            if isempty(obj.conn)
                return;
            end

            obj.conn = [];
            fprintf("Network: disconnected.\n");
        end
    end
end
