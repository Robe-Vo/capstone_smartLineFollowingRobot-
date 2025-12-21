% File: +capstone/+io/Network.m
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

        % ===================== TX/RX PRIMITIVES =====================
        function sendBytes(obj, payload_u8)
            if isempty(obj.conn), return; end
            c = obj.conn;
            b = uint8(payload_u8(:));

            if c.type == "bt" || c.type == "tcp"
                write(c.dev, b, "uint8");
            elseif c.type == "udp"
                write(c.dev, b, "uint8", c.ip, c.port);
            end
        end

        function sendByte(obj, data_u8)
            obj.sendBytes(uint8(data_u8));
        end

        function rx = recvBytes(obj, n)
            if isempty(obj.conn)
                rx = uint8([]);
                return;
            end
            c = obj.conn;
            rx = uint8([]);

            if c.type == "bt" || c.type == "tcp"
                if c.dev.NumBytesAvailable > 0
                    k  = min(n, c.dev.NumBytesAvailable);
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

        % ===================== OPERATION MODE HELPERS =====================
        function ok = opEnter(obj, ackTimeout)
            % Send 0xFF and wait ACK 0x20
            if nargin < 2, ackTimeout = 2.0; end
            p = capstone.io.Protocol.constants();
            obj.sendByte(p.CMD_MODE_OP);
            ok = obj.waitAck(ackTimeout);
        end

        function opExitToIdle(obj)
            % Send 0xFE (no ACK assumed)
            p = capstone.io.Protocol.constants();
            obj.sendByte(p.CMD_MODE_IDLE);
        end

        % ===================== SPEED CONVERSION (% -> PWM11) =====================
        function spd11 = opPwm11FromPercent(~, percent)
            % percent: -100..+100
            % spd11  : 0..2047 (abs mapping)
            if ~isscalar(percent) || ~isnumeric(percent)
                spd11 = uint16(0);
                return;
            end
            x = double(percent);
            x = max(-100, min(100, x));
            spd11 = uint16(round(abs(x) / 100 * 2047));
        end

        % ===================== OPERATION CONTROL FRAME (5 BYTES, LE) =====================
        function pkt = opPack5(~, cmd_u8, spd11_u16, ang_u16)
            % [cmd][spd_L][spd_H][ang_L][ang_H]  (LE)
            cmd = uint8(cmd_u8);

            spd = uint16(spd11_u16);
            if spd > 2047, spd = 2047; end

            ang = uint16(ang_u16);

            pkt = uint8([
                cmd
                bitand(spd,255)
                bitshift(spd,-8)
                bitand(ang,255)
                bitshift(ang,-8)
            ]);
        end

        function opSendControl(obj, speed_percent, angle_u16)
            % Send OPERATION control (CMD 0xF1), speed is percent, TX speed is 11-bit (0..2047)
            p = capstone.io.Protocol.constants();
            spd11 = obj.opPwm11FromPercent(speed_percent);
            pkt = obj.opPack5(p.CMD_OP_CTRL, spd11, uint16(angle_u16));
            obj.sendBytes(pkt);
        end

        function opSendStop(obj)
            % Send OPERATION stop (CMD 0xF0)
            p = capstone.io.Protocol.constants();
            obj.sendByte(p.CMD_OP_STOP);
        end

        % ===================== DIRECT DRIVE (BE, PWM11 0..2047) =====================
        function sendCmdU16BE(obj, cmd_u8, value_u16)
            % Firmware expects: cmd + hi + lo, and uses u16_be(hi,lo)
            if isempty(obj.conn), return; end
            v = uint16(value_u16);
            hi = uint8(bitshift(v, -8));
            lo = uint8(bitand(v, 255));
            obj.sendBytes(uint8([uint8(cmd_u8); hi; lo]));
        end

        function opDriveStepPercent(obj, percent, timeout_s, varargin)
            % opDriveStepPercent(net, percent, timeout_s, "enterOp", true/false, "exitIdle", true/false)
            % - percent > 0  => forward (0xDF + PWM11 BE)
            % - percent < 0  => backward (0xDE + PWM11 BE)
            % - percent = 0  => stop (0xDD)
            if nargin < 3, timeout_s = 1.0; end

            ip = inputParser;
            ip.addParameter("enterOp", true);
            ip.addParameter("exitIdle", false);
            ip.addParameter("ackTimeout", 2.0);
            ip.parse(varargin{:});
            opt = ip.Results;

            p = capstone.io.Protocol.constants();

            if opt.enterOp
                ok = obj.opEnter(opt.ackTimeout);
                if ok
                    fprintf("[NET] OPERATION ACK OK\n");
                else
                    fprintf("[NET] WARN: OPERATION no ACK\n");
                end
            end

            spd11 = obj.opPwm11FromPercent(percent); % 0..2047

            cleanup = onCleanup(@() localStop(obj, p, opt.exitIdle));

            if percent > 0
                obj.sendCmdU16BE(p.CMD_DRIVE_FWD, spd11);
                fprintf("[NET] STEP FWD %%=%.1f spd11=%d\n", double(percent), spd11);
            elseif percent < 0
                obj.sendCmdU16BE(p.CMD_DRIVE_BWD, spd11);
                fprintf("[NET] STEP BWD %%=%.1f spd11=%d\n", double(percent), spd11);
            else
                obj.sendByte(p.CMD_DRIVE_STOP);
                fprintf("[NET] STEP STOP\n");
                pause(timeout_s);
                return;
            end

            t0 = tic;
            while toc(t0) < timeout_s
                pause(0.005);
            end
        end

        % ===================== ACK =====================
        function ok = waitAck(obj, timeout)
            if nargin < 2, timeout = 1.0; end
            ok = false;
            if isempty(obj.conn), return; end
            p = capstone.io.Protocol.constants();
            target = p.ACK;

            t0 = tic;
            while toc(t0) < timeout
                b = obj.recvByte();
                if isempty(b)
                    pause(0.001);
                    continue;
                end
                if uint8(b) == uint8(target)
                    ok = true;
                    return;
                end
            end
        end

        function disconnect(obj)
            obj.conn = [];
        end

        function opSendSpeedFwd(obj, v_hz)
            % v_hz: wheel frequency [Hz], dương
            p = capstone.io.Protocol.constants();
            b = typecast(single(v_hz), 'uint8');  % float32 -> 4 byte LE
            obj.sendBytes(uint8([p.CMD_OP_SPD_FWD; b(:)]));
        end

        function opSendSpeedBwd(obj, v_hz)
            % v_hz: wheel frequency [Hz], dương
            p = capstone.io.Protocol.constants();
            b = typecast(single(v_hz), 'uint8');  % float32 -> 4 byte LE
            obj.sendBytes(uint8([p.CMD_OP_SPD_BWD; b(:)]));
        end
    
        function opSendBrake(obj)
            p = capstone.io.Protocol.constants();
            obj.sendByte(p.CMD_OP_BRAKE);
        end
    end
end

function localStop(net, proto, exitIdle)
    try
        net.sendByte(proto.CMD_DRIVE_STOP); % 0xDD
        fprintf("[NET] STOP sent (0xDD)\n");
    catch
    end
    if exitIdle
        try
            net.sendByte(proto.CMD_MODE_IDLE); % 0xFE
            fprintf("[NET] IDLE sent (0xFE)\n");
        catch
        end
    end
end
