% Fix: jogDrive must accept package-class Network object (capstone.io.Network)
% Replace the class check and (optionally) isConnected check.

% File: +capstone/+script/+jog/jogDrive.m
function jogDrive(net, pwm_u16, timeout_s, varargin)
%jogDrive  Send a DRIVE step command (u16) to robot, then STOP after timeout.
% (Same header as before; only the type-check is fixed for package class.)

    if nargin < 3
        disp("Error: jogDrive(net, pwm_u16, timeout_s, ...)");
        return;
    end

    % ---- FIXED type check: accept capstone.io.Network ----
    if isempty(net) || ~(isa(net,'capstone.io.Network') || isa(net,'Network'))
        disp("Error: net must be a capstone.io.Network (or Network) object.");
        return;
    end

    % ---- FIXED connection check: use method if exists ----
    if ismethod(net,'isConnected')
        if ~net.isConnected()
            disp("Error: net is not connected. Call net.connectBluetooth(...) first.");
            return;
        end
    end

    if ~isscalar(pwm_u16) || ~isnumeric(pwm_u16)
        disp("Error: pwm_u16 must be a numeric scalar.");
        return;
    end
    if ~isscalar(timeout_s) || ~isnumeric(timeout_s) || timeout_s <= 0
        disp("Error: timeout_s must be numeric scalar > 0.");
        return;
    end

    p = inputParser;
    p.addParameter("enterOp", true);
    p.addParameter("exitIdle", false);
    p.addParameter("ackTimeout", 2.0);
    p.parse(varargin{:});
    opt = p.Results;

    proto = capstone.io.Protocol.constants(); % ensure package Protocol if you placed it there

    if opt.enterOp
        net.sendByte(proto.CMD_MODE_OP);
        if net.waitAck(opt.ackTimeout)
            fprintf("[JOG] ACK 0x20: OPERATION\n");
        else
            fprintf("[JOG] WARN: no ACK after CMD_MODE_OP (0xFF)\n");
        end
    end

    spd = uint16(min(max(abs(double(pwm_u16)), 0), 65535));
    c = onCleanup(@() localStop(net, proto, opt.exitIdle));

    if double(pwm_u16) > 0
        net.sendCmdU16BE(proto.CMD_DRIVE_FWD, spd);
        fprintf("[JOG] DRIVE FWD spd_u16=%d (pwm11~= %d)\n", spd, bitshift(spd,-5));
    elseif double(pwm_u16) < 0
        net.sendCmdU16BE(proto.CMD_DRIVE_BWD, spd);
        fprintf("[JOG] DRIVE BWD spd_u16=%d (pwm11~= %d)\n", spd, bitshift(spd,-5));
    else
        net.sendDriveStop();
        fprintf("[JOG] DRIVE STOP\n");
        pause(timeout_s);
        return;
    end

    t0 = tic;
    while toc(t0) < timeout_s
        pause(0.005);
    end
end

function localStop(net, proto, exitIdle)
    try
        net.sendDriveStop();
        fprintf("[JOG] STOP sent (0xDD)\n");
    catch
    end
    if exitIdle
        try
            net.sendByte(proto.CMD_MODE_IDLE);
            fprintf("[JOG] CMD_MODE_IDLE sent (0xFE)\n");
        catch
        end
    end
end
