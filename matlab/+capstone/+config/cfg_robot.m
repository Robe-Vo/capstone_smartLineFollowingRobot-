function robot = cfg_robot()
%CFG_ROBOT Robot hardware + timing configuration (immutable for a season)
% Output:
%   robot struct

robot = struct();

%% ===== Identity =====
robot.name = "capstone_smartLineFollowingRobot";
robot.version = "v0";

%% ===== Timing (seconds) =====
robot.ts = struct();
robot.ts.speed_s    = 0.010;   % ESP32 speed PID loop
robot.ts.position_s = 0.100;   % MATLAB position PID loop
robot.ts.comm_s     = 0.050;   % host<->robot exchange period (edit if different)

%% ===== Drive command (11-bit) =====
robot.drive = struct();
robot.drive.cmd_bits = 11;                 % requested by user
robot.drive.cmd_max  = 2^11 - 1;           % 2047
robot.drive.cmd_min  = 0;
robot.drive.cmd_dtype = "uint16";          % transport type; effective bits = 11

% Units convention for host-side commands
robot.drive.cmd_unit = "raw11";            % "raw11" or "mmps" if you later map

%% ===== Mechanical constants =====
robot.mecha = struct();
robot.mecha.wheel_diameter_mm = 72;
robot.mecha.gear_motor        = 9.6;
robot.mecha.gear_diff_num     = 12;
robot.mecha.gear_diff_den     = 30;

% If you later need kinematics
robot.mecha.wheelbase_mm = NaN;            % TODO
robot.mecha.track_mm     = NaN;            % TODO

%% ===== Encoder =====
robot.encoder = struct();
robot.encoder.channels = 2;
robot.encoder.pulses_per_rev_disc = 11;    % as specified by user
robot.encoder.decode = "x4";               % TODO: "x1","x2","x4"
robot.encoder.resolution_note = "set explicitly to match firmware";

%% ===== Steering servo =====
robot.steer = struct();
robot.steer.servo_min_deg = 55;
robot.steer.servo_max_deg = 105;
robot.steer.servo_center_deg = NaN;        % TODO (if you calibrate center)

% Transport format for steer command (keep simple)
robot.steer.cmd_dtype = "uint16";
robot.steer.cmd_unit  = "deg";             % or "raw" if firmware expects raw

%% ===== Line sensor framing =====
robot.sense = struct();
robot.sense.line_n = 5;                    % 5x uint8
robot.sense.line_dtype = "uint8";

%% ===== Safety / saturation =====
robot.limits = struct();
robot.limits.steer_min_deg = robot.steer.servo_min_deg;
robot.limits.steer_max_deg = robot.steer.servo_max_deg;
robot.limits.drive_min_cmd = robot.drive.cmd_min;
robot.limits.drive_max_cmd = robot.drive.cmd_max;

end
