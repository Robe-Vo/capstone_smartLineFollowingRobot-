function frame = cfg_frame(robot, road, frame_prev)
    %CFG_FRAME Create a new frame struct (per rx/tx cycle).
    % Inputs:
    %   robot, road
    %   frame_prev (optional): previous frame to reuse fields
    % Output:
    %   frame struct
    
    if nargin < 3
        frame_prev = [];
    end
    
    frame = struct();
    
    %% ===== Index / time =====
    frame.k = NaN;
    frame.t_host = NaN;
    
    %% ===== Active context =====
    frame.road_id = road.id;
    frame.mode = "IDLE"; % or "OPERATION"
    
    %% ===== RX (robot -> host) =====
    frame.rx = struct();
    frame.rx.line_raw = zeros(1, robot.sense.line_n, robot.sense.line_dtype);
    frame.rx.encoder_count = NaN;
    frame.rx.encoder_delta = NaN;
    frame.rx.speed_meas = NaN;           % optional (fill if firmware sends)
    frame.rx.flags = struct('lineDetected',NaN,'commOk',NaN);
    
    %% ===== PROC (host computed) =====
    frame.proc = struct();
    frame.proc.line_error = NaN;
    frame.proc.line_mask = NaN;          % optional
    frame.proc.road_switch = "";         % if switch happened this frame
    
    %% ===== TX (host -> robot) =====
    frame.tx = struct();
    frame.tx.cmd = uint8(0);             % set by your controller
    frame.tx.drive_cmd = cast(0, robot.drive.cmd_dtype);   % 11-bit effective
    frame.tx.steer_cmd = cast(0, robot.steer.cmd_dtype);
    frame.tx.payload = uint8([]);        % optional raw bytes for logging
    
    %% ===== DIAG =====
    frame.diag = struct();
    frame.diag.drop = false;
    frame.diag.note = "";
    
    %% ===== Reuse policy =====
    if ~isempty(frame_prev)
        % Keep selected continuity signals
        frame.k = frame_prev.k + 1;
        frame.proc.prev_line_error = frame_prev.proc.line_error;
    else
        frame.k = 0;
        frame.proc.prev_line_error = NaN;
    end

end
