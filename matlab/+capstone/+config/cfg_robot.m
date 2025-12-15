function robot = cfg_robot()
%CFG_ROBOT  Robot hardware + timing configuration (immutable in a season)
% Output:
%   robot : struct

    robot = struct();

    %% ===== Hardware =====
    robot.hw = struct();

    % Drive
    robot.hw.drive = struct();
    robot.hw.drive.motor_name      = "GA25 370";
    robot.hw.drive.motor_rpm_nom   = 620;              % as provided
    robot.hw.drive.gear_motor      = 9.6;              % motor gearbox ratio
    robot.hw.drive.gear_diff_num   = 12;               % diff ratio numerator
    robot.hw.drive.gear_diff_den   = 30;               % diff ratio denominator

    % Wheel
    robot.hw.wheel = struct();
    robot.hw.wheel.diameter_mm     = 72;               % as provided
    robot.hw.wheel.radius_m        = (robot.hw.wheel.diameter_mm/1000)/2;

    % Encoder
    robot.hw.encoder = struct();
    robot.hw.encoder.channels      = 2;
    robot.hw.encoder.pulses_disc   = 11;               % as provided
    robot.hw.encoder.decode_mode   = "x4";             % placeholder: "x1"/"x2"/"x4"
    robot.hw.encoder.resolution    = 1;                % user-stated resolution = 1 (sampling freq tracking)

    % Steering
    robot.hw.steer = struct();
    robot.hw.steer.servo_name      = "MG996R";
    robot.hw.steer.min_deg         = 55;
    robot.hw.steer.max_deg         = 105;
    robot.hw.steer.center_deg      = 80;               % placeholder (fill later)
    robot.hw.steer.angle_unit      = "deg";            % keep consistent end-to-end

    %% ===== Timing =====
    robot.ts = struct();
    robot.ts.drive_speed_s   = 0.010;                  % 10ms speed loop on ESP32
    robot.ts.drive_pos_s     = 0.100;                  % 100ms position loop on MATLAB
    robot.ts.comm_s          = 0.025;                  % placeholder (frame rate host<->robot)

    %% ===== Command Limits / Protocol =====
    robot.limits = struct();

    % Drive command is 11-bit
    robot.limits.drive = struct();
    robot.limits.drive.cmd_bits    = 11;
    robot.limits.drive.cmd_min     = uint16(0);
    robot.limits.drive.cmd_max     = uint16(2^robot.limits.drive.cmd_bits - 1);  % 2047
    robot.limits.drive.cmd_unit    = "u11";            % raw command scale (protocol-level)

    % Steering command storage (keep as uint16 to match OPERATION frame plan)
    robot.limits.steer = struct();
    robot.limits.steer.cmd_min     = uint16(0);        % placeholder (fill later)
    robot.limits.steer.cmd_max     = uint16(65535);    % placeholder (fill later)
    robot.limits.steer.cmd_unit    = "u16";            % protocol-level

    %% ===== Conversions (placeholders) =====
    robot.conv = struct();
    robot.conv.drive = struct();
    robot.conv.drive.cmd_to_pwm    = [];              % placeholder
    robot.conv.drive.pwm_to_cmd    = [];              % placeholder

    robot.conv.steer = struct();
    robot.conv.steer.deg_to_cmd    = [];              % placeholder
    robot.conv.steer.cmd_to_deg    = [];              % placeholder
end
