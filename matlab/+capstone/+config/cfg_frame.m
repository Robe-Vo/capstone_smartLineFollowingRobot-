function frame = cfg_frame(robot)
%CFG_FRAME  Allocate a default frame struct for one tx/rx iteration
% Input:
%   robot : struct from cfg_robot()
% Output:
%   frame : struct

    frame = struct();

    %% Index / time
    frame.idx = struct();
    frame.idx.k          = uint32(0);
    frame.idx.t_host_s   = NaN;
    frame.idx.t_robot_s  = NaN;        % placeholder: if robot sends timestamp later

    %% RX: robot -> host (placeholders, fill in runtime)
    frame.rx = struct();

    % Line sensor: 5x uint8 (as your protocol summary)
    frame.rx.line = struct();
    frame.rx.line.raw_u8  = zeros(1,5,'uint8');
    frame.rx.line.valid   = false;

    % Encoder / speed feedback (placeholders)
    frame.rx.enc = struct();
    frame.rx.enc.count     = int32(0);
    frame.rx.enc.delta     = int32(0);
    frame.rx.speed_meas    = NaN;      % placeholder (unit decided later)

    % General flags
    frame.rx.flags = struct();
    frame.rx.flags.lineDetected = false;
    frame.rx.flags.commOk       = false;

    %% PROC: host processing results
    frame.proc = struct();
    frame.proc.road_id      = "";
    frame.proc.error_line   = NaN;     % placeholder: define unit later
    frame.proc.state        = struct();% placeholder container (still within frame)

    %% TX: host -> robot
    frame.tx = struct();
    frame.tx.cmd_id         = uint8(0);

    % Raw protocol-level commands
    frame.tx.drive_cmd_u16  = uint16(0);   % keep uint16 but clamp to u11 in runtime
    frame.tx.steer_cmd_u16  = uint16(0);

    % Optional: store raw payload if you want replay exact bytes
    frame.tx.payload_u8     = zeros(1,0,'uint8');

    %% DIAG
    frame.diag = struct();
    frame.diag.seq_tx       = uint32(0);   % placeholder: if you add sequence
    frame.diag.seq_rx       = uint32(0);
    frame.diag.dropDetected = false;

    %% Convenience: embed robot limits snapshot (no new top-level struct types)
    frame.meta = struct();
    frame.meta.drive_cmd_max = robot.limits.drive.cmd_max; % 2047
end
