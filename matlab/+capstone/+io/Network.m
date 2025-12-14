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

        function sendByte(obj, data)
            if isempty(obj.conn), return; end
            c = obj.conn;
            b = uint8(data);
            if c.type == "bt" || c.type == "tcp"
                write(c.dev, b, "uint8");
            elseif c.type == "udp"
                write(c.dev, b, "uint8", c.ip, c.port);
            end
        end

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
                    if isa(d,'udpport.datagram.Datagram')
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

        function frame = readFrameBlocking(obj, frameLen, timeout)
            if nargin < 3, timeout = 1.0; end
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
        
        function sendControl(obj, speed_u16, angle_u16, cmd)
            % OPERATION control frame 5B (LE):
            % [cmd][spd_L][spd_H][ang_L][ang_H]
            % speed_u16: 0..65535 (robot will scale to 11-bit internally)
        
            if nargin < 4
                cmd = hex2dec('F1');
            end
            if isempty(obj.conn)
                return;
            end
        
            spd = uint16(speed_u16);
            ang = uint16(angle_u16);
        
            spd_L = uint8(bitand(spd, 255));
            spd_H = uint8(bitshift(spd, -8));
        
            ang_L = uint8(bitand(ang, 255));
            ang_H = uint8(bitshift(ang, -8));
        
            pkt = uint8([uint8(cmd); spd_L; spd_H; ang_L; ang_H]);
            obj.sendByte(pkt);
        end
        
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
