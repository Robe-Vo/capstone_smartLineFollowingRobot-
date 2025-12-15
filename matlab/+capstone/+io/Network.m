classdef Network < handle
    properties
        conn % struct: fields: type, dev, (ip,port) for UDP
    end

    methods
        function obj = Network()
            obj.conn = [];
        end

        function tf = isConnected(obj)
            tf = ~isempty(obj.conn) && isfield(obj.conn,"dev") && ~isempty(obj.conn.dev);
        end

        function connectBluetooth(obj, deviceName, channelID)
            if ~isempty(obj.conn), obj.disconnect(); end
            bt = bluetooth(deviceName, channelID);
            bt.Timeout = 5;
            obj.conn = struct("type","bt","dev",bt);
        end

        function connectUDP(obj, ip, port)
            if ~isempty(obj.conn), obj.disconnect(); end
            u = udpport("datagram","IPV4");
            u.Timeout = 5;
            obj.conn = struct("type","udp","dev",u,"ip",ip,"port",port);
        end

        function connectTCP(obj, ip, port)
            if ~isempty(obj.conn), obj.disconnect(); end
            t = tcpclient(ip, port, "Timeout", 5);
            obj.conn = struct("type","tcp","dev",t);
        end

        function sendBytes(obj, payload_u8)
            % Send an arbitrary uint8 vector (preferred for frames)
            if isempty(obj.conn), return; end
            c = obj.conn;
            b = uint8(payload_u8(:));
            if c.type == "bt" || c.type == "tcp"
                write(c.dev, b, "uint8");
            elseif c.type == "udp"
                write(c.dev, b, "uint8", c.ip, c.port);
            end
        end

        function sendByte(obj, data)
            obj.sendBytes(uint8(data));
        end

        function rx = recvBytes(obj, n)
            % Read up to n bytes non-blocking for bt/tcp, or one datagram for udp.
            if isempty(obj.conn)
                rx = uint8([]);
                return;
            end
            c = obj.conn;
            rx = uint8([]);

            if c.type == "bt" || c.type == "tcp"
                if c.dev.NumBytesAvailable > 0
                    k = min(n, c.dev.NumBytesAvailable);
                    rx = read(c.dev, k, "uint8");
                end
            elseif c.type == "udp"
                u = c.dev;
                if u.NumDatagramsAvailable > 0
                    d = read(u, 1, "uint8");
                    if isa(d,'udpport.datagram.Datagram')
                        rx = uint8(d.Data);
                    else
                        rx = uint8(d);
                    end
                    if numel(rx) > n
                        rx = rx(1:n);
                    end
                end
            end
        end

        function rx = recvByte(obj)
            r = obj.recvBytes(1);
            if isempty(r), rx = []; else, rx = r(1); end
        end

        function frame = readFrameBlocking(obj, frameLen, timeout)
            if nargin < 3, timeout = 1.0; end
            frame = uint8([]);
            t0 = tic;
            while numel(frame) < frameLen
                if toc(t0) > timeout
                    frame = [];
                    return;
                end
                r = obj.recvBytes(frameLen - numel(frame));
                if isempty(r)
                    pause(0.001);
                    continue;
                end
                frame = [frame; r(:)]; %#ok<AGROW>
            end
        end

        % ===================== OPERATION CONTROL (5 BYTES) =====================
        function pkt = packOp5(obj, cmd_u8, speed_u16, angle_u16)
            % Pack OPERATION control frame (5 bytes, little-endian):
            %   [cmd][spd_L][spd_H][ang_L][ang_H]
            % speed_u16: 0..65535; robot scales to 11-bit PWM internally (>>5).
            %#ok<INUSD>
            cmd = uint8(cmd_u8);
            spd = uint16(speed_u16);
            ang = uint16(angle_u16);

            spd_L = uint8(bitand(spd, 255));
            spd_H = uint8(bitshift(spd, -8));

            ang_L = uint8(bitand(ang, 255));
            ang_H = uint8(bitshift(ang, -8));

            pkt = uint8([cmd; spd_L; spd_H; ang_L; ang_H]);
        end

        function [cmd_u8, speed_u16, angle_u16] = unpackOp5(obj, pkt)
            % Unpack OPERATION control frame (5 bytes, little-endian).
            %#ok<INUSD>
            if isempty(pkt) || numel(pkt) ~= 5
                cmd_u8   = uint8(0);
                speed_u16 = uint16(0);
                angle_u16 = uint16(0);
                return;
            end
            p = uint8(pkt(:));
            cmd_u8 = p(1);

            speed_u16 = uint16(p(2)) + bitshift(uint16(p(3)), 8);
            angle_u16 = uint16(p(4)) + bitshift(uint16(p(5)), 8);
        end

        function sendControlOp5(obj, speed_u16, angle_u16, cmd_u8)
            % Send OPERATION control frame (5 bytes, LE):
            %   cmd(1B) + speed(2B) + angle(2B)
            % Default cmd is 0xF1.
            if nargin < 4 || isempty(cmd_u8)
                cmd_u8 = uint8(hex2dec('F1'));
            end
            pkt = obj.packOp5(cmd_u8, speed_u16, angle_u16);
            obj.sendBytes(pkt);
        end

        function [ok, cmd_u8, speed_u16, angle_u16] = recvControlOp5(obj, timeout)
            % Receive OPERATION control frame (5 bytes, LE) using blocking read.
            if nargin < 2, timeout = 1.0; end
            ok = false;
            cmd_u8 = uint8(0);
            speed_u16 = uint16(0);
            angle_u16 = uint16(0);

            pkt = obj.readFrameBlocking(5, timeout);
            if isempty(pkt), return; end
            ok = true;
            [cmd_u8, speed_u16, angle_u16] = obj.unpackOp5(pkt);
        end

        % Backward compatible name (keeps your existing PID code usage)
        function sendControl(obj, speed_u16, angle_u16, cmd)
            % Alias to sendControlOp5()
            if nargin < 4, cmd = uint8(hex2dec('F1')); end
            obj.sendControlOp5(speed_u16, angle_u16, cmd);
        end

        function sendCmdU16BE(obj, cmd_u8, value_u16)
            if isempty(obj.conn), return; end
            cmd = uint8(cmd_u8);
            v   = uint16(value_u16);
            hi  = uint8(bitshift(v, -8));
            lo  = uint8(bitand(v, 255));
            obj.sendBytes(uint8([cmd; hi; lo]));
        end
    
        function sendDriveStop(obj)
            proto = capstone.io.Protocol.constants();
            obj.sendByte(proto.CMD_DRIVE_STOP);
        end


        % ===================== ACK =====================
        function ok = waitAck(obj, timeout)
            if nargin < 2, timeout = 1.0; end
            ok = false;
            if isempty(obj.conn), return; end

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

        function disconnect(obj)
            obj.conn = [];
        end
    end
end
