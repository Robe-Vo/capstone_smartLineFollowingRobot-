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

        function [ok, meas] = recvSensorFrame(obj, timeout)
            % Frame fixed 22 bytes (the same as your old program)
            % 1..5:  line uint8[5]
            % 6..7:  ultrasonic uint16 (BE)
            % 8..19: MPU int16[6] (BE)
            % 20:    encCount uint8
            % 21..22:encSpeed uint16 (BE), rpm*100

            FRAME_LEN = 22;
            if nargin < 2, timeout = 1.0; end

            meas = struct("lineRaw",[],"ultra",[],"mpu",[],"encCount",[],"encSpeed",[]);
            ok = false;

            frame = obj.readFrameBlocking(FRAME_LEN, timeout);
            if isempty(frame), return; end

            meas.lineRaw = double(frame(1:5)).';

            hiU = uint16(frame(6)); loU = uint16(frame(7));
            meas.ultra = bitor(bitshift(hiU,8), loU);

            mpuBytes = frame(8:19);
            mpu = zeros(1,6);
            for k = 1:6
                hi = uint16(mpuBytes(2*k-1));
                lo = uint16(mpuBytes(2*k));
                v16 = bitor(bitshift(hi,8), lo);
                mpu(k) = double(typecast(v16,'int16'));
            end
            meas.mpu = mpu;

            meas.encCount = double(frame(20));

            hiS = uint16(frame(21)); loS = uint16(frame(22));
            spd_q = bitor(bitshift(hiS,8), loS);
            meas.encSpeed = double(spd_q)/100.0;

            ok = true;
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

        function sendControl(obj, speed, angle, cmd)
            if nargin < 4, cmd = hex2dec('F1'); end
            if isempty(obj.conn), return; end

            speed_u8 = uint8(speed);
            angle_u16 = uint16(angle);

            angle_hi = uint8(bitshift(angle_u16, -8));
            angle_lo = uint8(bitand(angle_u16, 255));

            pkt = uint8([uint8(cmd); speed_u8; angle_hi; angle_lo]);
            obj.sendByte(pkt);
        end

        function disconnect(obj)
            obj.conn = [];
        end
    end
end
