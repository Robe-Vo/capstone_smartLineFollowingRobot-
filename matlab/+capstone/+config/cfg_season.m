function season = cfg_season(robot, roads, btName)
%CFG_SEASON  Season container + calibration + runtime states (no extra top-level structs)
% Usage:
%   season = capstone.config.cfg_season(robot, roads, "Behind the scream");

    season = struct();

    %% ===== Setup =====
    season.setup = struct();
    season.setup.robot = robot;
    season.setup.roads = roads;

    season.setup.comm = struct();
    season.setup.comm.transport = "bluetooth";
    season.setup.comm.btName    = string(btName);
    season.setup.comm.channel   = NaN;
    season.setup.comm.proto_ver = "OP_u16_u16";

    %% ===== Calibration / Sensors =====
    % Line sensor calibration (5 sensors)
    season.setup.calib = struct();

    season.setup.calib.line = struct();
    season.setup.calib.line.n           = 5;
    season.setup.calib.line.weights     = [-2 -1 0 1 2];     % EDIT if needed
    season.setup.calib.line.invert      = false;             % EDIT
    season.setup.calib.line.min_u8      = zeros(1,5,'double');% EDIT: calibration floor
    season.setup.calib.line.max_u8      = 255*ones(1,5);      % EDIT: calibration ceiling
    season.setup.calib.line.th_detect   = NaN;               % EDIT: detection threshold on sum/contrast
    season.setup.calib.line.err_clip    = 2.0;               % clip error in weight units
    season.setup.calib.line.err_lp_alpha= NaN;               % optional LPF on error [0..1]
    season.setup.calib.line.center_ref  = 0.0;               % desired error center

    % Ultrasonic config (placeholders)
    season.setup.calib.ultra = struct();
    season.setup.calib.ultra.enable       = false;           % EDIT when sensor exists
    season.setup.calib.ultra.dist_unit    = "m";
    season.setup.calib.ultra.th_obstacle_m= NaN;             % EDIT
    season.setup.calib.ultra.debounce_n   = 3;               % consecutive frames
    season.setup.calib.ultra.timeout_s    = NaN;             % EDIT
    season.setup.calib.ultra.valid_min_m  = NaN;             % EDIT
    season.setup.calib.ultra.valid_max_m  = NaN;             % EDIT

    % MPU config (placeholders)
    season.setup.calib.mpu = struct();
    season.setup.calib.mpu.enable       = false;             % EDIT when used
    season.setup.calib.mpu.sample_s     = NaN;               % EDIT
    season.setup.calib.mpu.lpf_hz       = NaN;               % EDIT
    season.setup.calib.mpu.bias_gyr     = [NaN NaN NaN];      % EDIT
    season.setup.calib.mpu.bias_acc     = [NaN NaN NaN];      % EDIT
    season.setup.calib.mpu.use_yaw      = false;             % EDIT

    %% ===== Runtime =====
    season.runtime = struct();
    season.runtime.mode     = "IDLE";          % "IDLE" | "OPERATION"
    season.runtime.road_idx = uint16(1);
    season.runtime.road_id  = season.setup.roads(1).id;

    % T-junction turn decision (current)
    season.runtime.nav = struct();
    season.runtime.nav.t_junction_turn = "STRAIGHT"; % "LEFT"|"RIGHT"|"STRAIGHT" (EDIT runtime)

    % Robot high-level state (as requested)
    season.runtime.robot_state = "FOLLOW_LINE";  % "FOLLOW_LINE"|"AVOID"|"BLIND"

    % Line tracking stability state (as requested)
    season.runtime.line_state  = "STABLE";       % "STABLE"|"UNSTABLE"|"OUTLINE"

    % Internal counters for stability/obstacle
    season.runtime.counters = struct();
    season.runtime.counters.lostline_n   = uint16(0);
    season.runtime.counters.obstacle_n   = uint16(0);

    % Last frame template
    season.runtime.frame_prev = capstone.config.cfg_frame(robot);

    % Steering milestone + PID memory
    season.runtime.steer = struct('milestone_idx',uint16(1),'prev_cmd_deg',robot.hw.steer.center_deg);
    season.runtime.pid   = struct('I',0,'e_prev',0,'err_filt',0,'steer_prev_deg',robot.hw.steer.center_deg);

    %% ===== Log =====
    season.log = struct();
    season.log.frames = repmat(capstone.config.cfg_frame(robot), 1, 0);
    season.log.events = strings(1,0);

    season.meta = struct();
    season.meta.created_host_time = NaN;
end
