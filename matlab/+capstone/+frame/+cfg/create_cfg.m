function cfg = create_cfg()
% Create cfg struct

    % Init cfg struct
    cfg = struct([]);

    % cfg configuration
    cfg.idx = 1; % -1 for blind-run , 0 for lost-line
    cfg.name = 'LINE 1';

    % Geometry
    cfg.road.type = 'LINE'; % 'Curve', 'NONE', 'LOST_LINE'
    cfg.road.startp = [0 0];
    cfg.road.endp = [0 0];
    cfg.road.normalVec = [-1 0];
    cfg.road.tangentVec = [0 1];

    % Characteristics
    cfg.tick = 0; % Encoder pulse for end cfg
    cfg.maxAngle = 76; % Stable requirement 
    cfg.minAngle = 74; % Stable requirement 
    cfg.motionProfile = [];

    % Controller - steer
    cfg.steer.controller = 'PID' % 'ON_OFF'
    
    % PID tune params
    cfg.steer.pid.kp = 0;
    cfg.steer.pid.ki = 0;
    cfg.steer.pid.kd = 0;

    % ON_OFF tune params
    cfg.steer.on_off.threshold = 0;

    % PID - Position drive
    cfg.drive.pid.kp = 0;
    cfg.drive.pid.ki = 0;
    cfg.drive.pid.kd = 0;

    % Switch statement -
    % [00000]
    % 1st: 
    cfg.switch.mask
end

