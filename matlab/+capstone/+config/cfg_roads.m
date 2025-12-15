function roads = cfg_roads(robot)
%CFG_ROADS  Define all road segments + full tuning blocks (edit here)
% Usage:
%   robot = capstone.config.cfg_robot();
%   roads = capstone.config.cfg_roads(robot);

    roads = repmat(cfg_oneRoad(robot), 1, 11); % 9 map segments + END + AVOID

    %% =====================================================================
    %  [1] L1 (LINE)
    %% =====================================================================
    i = 1;
    roads(i).id   = "L1";
    roads(i).type = "LINE";

    roads(i).geom.start_xy_mm  = [   0.00    0.00];
    roads(i).geom.end_xy_mm    = [-2500.00   0.00];
    roads(i).geom.center_xy_mm = [NaN NaN];
    roads(i).geom.R_mm         = NaN;

    roads(i).switch.entry_condition = "START_RUN";
    roads(i).switch.exit_condition  = "ENC_REACH_SEG_END";
    roads(i).switch.fallback        = "LOSTLINE->BLIND";
    roads(i).switch.next_id_default = "A2";
    roads(i).switch.branch_ids      = strings(1,0);

    % Drive: constant speed mode (11-bit)
    roads(i).tune.drive.mode          = "CONST_SPEED";
    roads(i).tune.drive.const_cmd_u11 = uint16(800);     % EDIT: 0..2047
    roads(i).tune.drive.pid_speed.Kp  = NaN;
    roads(i).tune.drive.pid_speed.Ki  = NaN;
    roads(i).tune.drive.pid_speed.Kd  = NaN;
    roads(i).tune.drive.pid_speed.u_min = 0;
    roads(i).tune.drive.pid_speed.u_max = double(robot.limits.drive.cmd_max);
    roads(i).tune.drive.pid_speed.aw_enable = true;
    roads(i).tune.drive.pid_speed.aw_gain   = NaN;

    % Steering: PID + milestones
    roads(i).tune.steer.mode = "PID_MILESTONE";
    roads(i).tune.steer.pid.Kp = NaN;
    roads(i).tune.steer.pid.Ki = NaN;
    roads(i).tune.steer.pid.Kd = NaN;
    roads(i).tune.steer.pid.deadband_err = NaN;         % in error-unit
    roads(i).tune.steer.pid.I_min = NaN;                % integrator clamp
    roads(i).tune.steer.pid.I_max = NaN;
    roads(i).tune.steer.pid.dLim_deg_per_s = NaN;       % optional slew rate
    roads(i).tune.steer.pid.milestones_deg = [robot.hw.steer.center_deg]; % EDIT: [80 78 80 ...]
    roads(i).tune.steer.pid.overshoot_guard.enable = true;
    roads(i).tune.steer.pid.overshoot_guard.next_guard_deg = NaN; % optional

    % ONOFF placeholders
    roads(i).tune.steer.onoff.th_on = NaN;
    roads(i).tune.steer.onoff.th_off = NaN;
    roads(i).tune.steer.onoff.cmd_left_deg = NaN;
    roads(i).tune.steer.onoff.cmd_right_deg = NaN;

    % Motion placeholders (unused in CONST_SPEED but kept for consistency)
    roads(i).motion.encoder_count_seg = NaN;
    roads(i).motion.v0_prev_end       = NaN;
    roads(i).motion.vmax              = NaN;
    roads(i).motion.vmin              = NaN;
    roads(i).motion.t_finish          = NaN;
    roads(i).motion.profile.valid     = false;
    roads(i).motion.profile.type      = "TRAPEZOID";
    roads(i).motion.profile.data      = [];


    %% =====================================================================
    %  [2] A2 (ARC, R=500)
    %% =====================================================================
    i = 2;
    roads(i).id   = "A2";
    roads(i).type = "ARC";

    roads(i).geom.start_xy_mm  = [-2500.00     0.00];
    roads(i).geom.end_xy_mm    = [-2500.00  1000.00];
    roads(i).geom.center_xy_mm = [-2500.00   500.00];
    roads(i).geom.R_mm         = 500.00;

    roads(i).switch.entry_condition = "FROM:L1";
    roads(i).switch.exit_condition  = "ENC_REACH_SEG_END";
    roads(i).switch.fallback        = "LOSTLINE->BLIND";
    roads(i).switch.next_id_default = "L3";
    roads(i).switch.branch_ids      = strings(1,0);

    roads(i).tune.drive.mode          = "CONST_SPEED";
    roads(i).tune.drive.const_cmd_u11 = uint16(700);     % EDIT
    roads(i).tune.drive.pid_speed.Kp  = NaN; roads(i).tune.drive.pid_speed.Ki = NaN; roads(i).tune.drive.pid_speed.Kd = NaN;

    roads(i).tune.steer.mode = "PID_MILESTONE";
    roads(i).tune.steer.pid.Kp = NaN; roads(i).tune.steer.pid.Ki = NaN; roads(i).tune.steer.pid.Kd = NaN;
    roads(i).tune.steer.pid.deadband_err = NaN;
    roads(i).tune.steer.pid.I_min = NaN; roads(i).tune.steer.pid.I_max = NaN;
    roads(i).tune.steer.pid.dLim_deg_per_s = NaN;
    roads(i).tune.steer.pid.milestones_deg = [robot.hw.steer.center_deg]; % EDIT


    %% =====================================================================
    %  [3] L3 (LINE)
    %% =====================================================================
    i = 3;
    roads(i).id   = "L3";
    roads(i).type = "LINE";

    roads(i).geom.start_xy_mm  = [-2500.00 1000.00];
    roads(i).geom.end_xy_mm    = [-2331.37 1000.00];

    roads(i).switch.entry_condition = "FROM:A2";
    roads(i).switch.exit_condition  = "ENC_REACH_SEG_END";
    roads(i).switch.fallback        = "LOSTLINE->BLIND";
    roads(i).switch.next_id_default = "A4";

    roads(i).tune.drive.mode          = "CONST_SPEED";
    roads(i).tune.drive.const_cmd_u11 = uint16(750);     % EDIT

    roads(i).tune.steer.mode = "PID_MILESTONE";
    roads(i).tune.steer.pid.Kp = NaN; roads(i).tune.steer.pid.Ki = NaN; roads(i).tune.steer.pid.Kd = NaN;
    roads(i).tune.steer.pid.deadband_err = NaN;
    roads(i).tune.steer.pid.I_min = NaN; roads(i).tune.steer.pid.I_max = NaN;
    roads(i).tune.steer.pid.dLim_deg_per_s = NaN;
    roads(i).tune.steer.pid.milestones_deg = [robot.hw.steer.center_deg]; % EDIT


    %% =====================================================================
    %  [4] A4 (ARC, R=800)  (T-junction node in your map list)
    %% =====================================================================
    i = 4;
    roads(i).id   = "A4";
    roads(i).type = "ARC";

    roads(i).geom.start_xy_mm  = [-2331.37 1000.00];
    roads(i).geom.end_xy_mm    = [-1765.69 1234.31];
    roads(i).geom.center_xy_mm = [-2331.37 1800.00];
    roads(i).geom.R_mm         = 800.00;

    roads(i).switch.entry_condition = "FROM:L3";
    roads(i).switch.exit_condition  = "ENC_REACH_SEG_END";
    roads(i).switch.fallback        = "LOSTLINE->BLIND";
    roads(i).switch.next_id_default = "A5";
    roads(i).switch.branch_ids      = ["A7"]; % branch candidate

    % Junction policy placeholders
    roads(i).junction = struct();
    roads(i).junction.isTjunction = true;
    roads(i).junction.turn_default = "STRAIGHT"; % "LEFT"|"RIGHT"|"STRAIGHT" (EDIT)
    roads(i).junction.turn_left_id  = "A7";
    roads(i).junction.turn_right_id = "A5";      % placeholder
    roads(i).junction.turn_straight_id = "A5";   % placeholder

    roads(i).tune.drive.mode          = "CONST_SPEED";
    roads(i).tune.drive.const_cmd_u11 = uint16(650);     % EDIT (slow down at junction)

    roads(i).tune.steer.mode = "PID_MILESTONE";
    roads(i).tune.steer.pid.Kp = NaN; roads(i).tune.steer.pid.Ki = NaN; roads(i).tune.steer.pid.Kd = NaN;
    roads(i).tune.steer.pid.deadband_err = NaN;
    roads(i).tune.steer.pid.I_min = NaN; roads(i).tune.steer.pid.I_max = NaN;
    roads(i).tune.steer.pid.dLim_deg_per_s = NaN;
    roads(i).tune.steer.pid.milestones_deg = [robot.hw.steer.center_deg]; % EDIT


    %% =====================================================================
    %  [5] A5 (ARC, R=800)
    %% =====================================================================
    i = 5;
    roads(i).id   = "A5";
    roads(i).type = "ARC";

    roads(i).geom.start_xy_mm  = [-1765.69 1234.31];
    roads(i).geom.end_xy_mm    = [-1168.63 1500.00];
    roads(i).geom.center_xy_mm = [-1170.28  700.00];
    roads(i).geom.R_mm         = 800.00;

    roads(i).switch.entry_condition = "FROM:A4";
    roads(i).switch.exit_condition  = "ENC_REACH_SEG_END";
    roads(i).switch.fallback        = "LOSTLINE->BLIND";
    roads(i).switch.next_id_default = "L6";

    roads(i).tune.drive.mode          = "CONST_SPEED";
    roads(i).tune.drive.const_cmd_u11 = uint16(700);     % EDIT
    roads(i).tune.steer.mode = "PID_MILESTONE";
    roads(i).tune.steer.pid.Kp = NaN; roads(i).tune.steer.pid.Ki = NaN; roads(i).tune.steer.pid.Kd = NaN;
    roads(i).tune.steer.pid.deadband_err = NaN;
    roads(i).tune.steer.pid.I_min = NaN; roads(i).tune.steer.pid.I_max = NaN;
    roads(i).tune.steer.pid.dLim_deg_per_s = NaN;
    roads(i).tune.steer.pid.milestones_deg = [robot.hw.steer.center_deg]; % EDIT


    %% =====================================================================
    %  [6] L6 (LINE)
    %% =====================================================================
    i = 6;
    roads(i).id   = "L6";
    roads(i).type = "LINE";

    roads(i).geom.start_xy_mm  = [-1168.63 1500.00];
    roads(i).geom.end_xy_mm    = [    0.00 1500.00];

    roads(i).switch.entry_condition = "FROM:A5";
    roads(i).switch.exit_condition  = "ENC_REACH_SEG_END";
    roads(i).switch.fallback        = "LOSTLINE->BLIND";
    roads(i).switch.next_id_default = "END";

    roads(i).tune.drive.mode          = "CONST_SPEED";
    roads(i).tune.drive.const_cmd_u11 = uint16(800);     % EDIT
    roads(i).tune.steer.mode = "PID_MILESTONE";
    roads(i).tune.steer.pid.Kp = NaN; roads(i).tune.steer.pid.Ki = NaN; roads(i).tune.steer.pid.Kd = NaN;
    roads(i).tune.steer.pid.deadband_err = NaN;
    roads(i).tune.steer.pid.I_min = NaN; roads(i).tune.steer.pid.I_max = NaN;
    roads(i).tune.steer.pid.dLim_deg_per_s = NaN;
    roads(i).tune.steer.pid.milestones_deg = [robot.hw.steer.center_deg]; % EDIT


    %% =====================================================================
    %  [7] A7 (ARC, R=800) (branch)
    %% =====================================================================
    i = 7;
    roads(i).id   = "A7";
    roads(i).type = "ARC";

    roads(i).geom.start_xy_mm  = [-2331.37 1000.00];
    roads(i).geom.end_xy_mm    = [-1765.69  765.69];
    roads(i).geom.center_xy_mm = [-2331.37  200.00];
    roads(i).geom.R_mm         = 800.00;

    roads(i).switch.entry_condition = "FROM:A4_BRANCH";
    roads(i).switch.exit_condition  = "ENC_REACH_SEG_END";
    roads(i).switch.fallback        = "LOSTLINE->BLIND";
    roads(i).switch.next_id_default = "A8";

    roads(i).tune.drive.mode          = "CONST_SPEED";
    roads(i).tune.drive.const_cmd_u11 = uint16(650);     % EDIT
    roads(i).tune.steer.mode = "PID_MILESTONE";
    roads(i).tune.steer.pid.Kp = NaN; roads(i).tune.steer.pid.Ki = NaN; roads(i).tune.steer.pid.Kd = NaN;
    roads(i).tune.steer.pid.deadband_err = NaN;
    roads(i).tune.steer.pid.I_min = NaN; roads(i).tune.steer.pid.I_max = NaN;
    roads(i).tune.steer.pid.dLim_deg_per_s = NaN;
    roads(i).tune.steer.pid.milestones_deg = [robot.hw.steer.center_deg]; % EDIT


    %% =====================================================================
    %  [8] A8 (ARC, R=800)
    %% =====================================================================
    i = 8;
    roads(i).id   = "A8";
    roads(i).type = "ARC";

    roads(i).geom.start_xy_mm  = [-1765.69  765.69];
    roads(i).geom.end_xy_mm    = [-1168.63  500.00];
    roads(i).geom.center_xy_mm = [-1170.28 1300.00];
    roads(i).geom.R_mm         = 800.00;

    roads(i).switch.entry_condition = "FROM:A7";
    roads(i).switch.exit_condition  = "ENC_REACH_SEG_END";
    roads(i).switch.fallback        = "LOSTLINE->BLIND";
    roads(i).switch.next_id_default = "L9";

    roads(i).tune.drive.mode          = "CONST_SPEED";
    roads(i).tune.drive.const_cmd_u11 = uint16(650);     % EDIT
    roads(i).tune.steer.mode = "PID_MILESTONE";
    roads(i).tune.steer.pid.Kp = NaN; roads(i).tune.steer.pid.Ki = NaN; roads(i).tune.steer.pid.Kd = NaN;
    roads(i).tune.steer.pid.deadband_err = NaN;
    roads(i).tune.steer.pid.I_min = NaN; roads(i).tune.steer.pid.I_max = NaN;
    roads(i).tune.steer.pid.dLim_deg_per_s = NaN;
    roads(i).tune.steer.pid.milestones_deg = [robot.hw.steer.center_deg]; % EDIT


    %% =====================================================================
    %  [9] L9 (LINE)
    %% =====================================================================
    i = 9;
    roads(i).id   = "L9";
    roads(i).type = "LINE";

    roads(i).geom.start_xy_mm  = [-1168.63  500.00];
    roads(i).geom.end_xy_mm    = [    0.00  500.00];

    roads(i).switch.entry_condition = "FROM:A8";
    roads(i).switch.exit_condition  = "ENC_REACH_SEG_END";
    roads(i).switch.fallback        = "LOSTLINE->BLIND";
    roads(i).switch.next_id_default = "END";

    roads(i).tune.drive.mode          = "CONST_SPEED";
    roads(i).tune.drive.const_cmd_u11 = uint16(800);     % EDIT
    roads(i).tune.steer.mode = "PID_MILESTONE";
    roads(i).tune.steer.pid.Kp = NaN; roads(i).tune.steer.pid.Ki = NaN; roads(i).tune.steer.pid.Kd = NaN;
    roads(i).tune.steer.pid.deadband_err = NaN;
    roads(i).tune.steer.pid.I_min = NaN; roads(i).tune.steer.pid.I_max = NaN;
    roads(i).tune.steer.pid.dLim_deg_per_s = NaN;
    roads(i).tune.steer.pid.milestones_deg = [robot.hw.steer.center_deg]; % EDIT


    %% =====================================================================
    %  [10] END
    %% =====================================================================
    i = 10;
    roads(i).id   = "END";
    roads(i).type = "END";

    roads(i).switch.entry_condition = "REACHED_FINAL";
    roads(i).switch.exit_condition  = "NONE";
    roads(i).switch.fallback        = "NONE";
    roads(i).switch.next_id_default = "";

    roads(i).tune.drive.mode          = "CONST_SPEED";
    roads(i).tune.drive.const_cmd_u11 = uint16(0);
    roads(i).tune.steer.mode          = "PID_MILESTONE";
    roads(i).tune.steer.pid.milestones_deg = [robot.hw.steer.center_deg];

    roads(i).motion.profile.valid = true;
    roads(i).motion.profile.type  = "STOP";


    %% =====================================================================
    %  [11] AVOID (obstacle avoidance)
    %% =====================================================================
    i = 11;
    roads(i).id   = "AVOID";
    roads(i).type = "AVOID";

    roads(i).switch.entry_condition = "OBSTACLE_DETECTED";
    roads(i).switch.exit_condition  = "OBSTACLE_CLEARED";
    roads(i).switch.fallback        = "TIMEOUT->IDLE";
    roads(i).switch.next_id_default = ""; % will be set at runtime (return-to-road)

    roads(i).tune.drive.mode          = "CONST_SPEED";
    roads(i).tune.drive.const_cmd_u11 = uint16(400);     % EDIT: slow

    roads(i).tune.steer.mode = "ONOFF";
    roads(i).tune.steer.onoff.th_on        = NaN;
    roads(i).tune.steer.onoff.th_off       = NaN;
    roads(i).tune.steer.onoff.cmd_left_deg = NaN;
    roads(i).tune.steer.onoff.cmd_right_deg= NaN;

end

function r = cfg_oneRoad(robot)
    r = struct();

    r.id   = "";
    r.type = "";

    r.geom = struct('start_xy_mm',[NaN NaN],'end_xy_mm',[NaN NaN],'center_xy_mm',[NaN NaN],'R_mm',NaN);

    r.switch = struct();
    r.switch.entry_condition = "";
    r.switch.exit_condition  = "";
    r.switch.fallback        = "";
    r.switch.next_id_default = "";
    r.switch.branch_ids      = strings(1,0);

    r.junction = struct();
    r.junction.isTjunction = false;
    r.junction.turn_default = "";
    r.junction.turn_left_id = "";
    r.junction.turn_right_id = "";
    r.junction.turn_straight_id = "";

    r.tune = struct();

    r.tune.drive = struct();
    r.tune.drive.mode          = "CONST_SPEED";
    r.tune.drive.const_cmd_u11 = uint16(0);
    r.tune.drive.pid_speed = struct('Kp',NaN,'Ki',NaN,'Kd',NaN,'u_min',0,'u_max',double(robot.limits.drive.cmd_max),'aw_enable',true,'aw_gain',NaN);

    r.tune.steer = struct();
    r.tune.steer.mode = "PID_MILESTONE";
    r.tune.steer.pid  = struct( ...
        'Kp',NaN,'Ki',NaN,'Kd',NaN, ...
        'deadband_err',NaN, ...
        'I_min',NaN,'I_max',NaN, ...
        'dLim_deg_per_s',NaN, ...
        'milestones_deg',[robot.hw.steer.center_deg], ...
        'overshoot_guard',struct('enable',true,'next_guard_deg',NaN) ...
    );
    r.tune.steer.onoff = struct('th_on',NaN,'th_off',NaN,'cmd_left_deg',NaN,'cmd_right_deg',NaN);

    r.motion = struct('encoder_count_seg',NaN,'v0_prev_end',NaN,'vmax',NaN,'vmin',NaN,'t_finish',NaN);
    r.motion.profile = struct('valid',false,'type',"TRAPEZOID",'data',[]);
end
