function frame = create_frame(idx)
% This function use to create a new frame

    % Init frame
    frame = struct([])

    %% Frame structure
    frame.idx = idx;
    frame.roadidx = 1; % Use this to access to params of the map
    frame.status = 'STABLE'; % 'OCSILLATE' | 'LOST_LINE'
    frame.time = 0;
    frame.distance = 0;
    frame.tick_enc = 0;

    % Controller storage
    frame.controller.type = 'MANUAL'; % PID | ON_OFF | AVOID_OBSTACLE
    frame.controller.error = 0;
    frame.controller.derror = 0;

    % PID buffer
    frame.controller.pid.error = 0;
    frame.controller.pid.d_error = 0;
    frame.controller.pid. = 0;

    % ON_OFF params and buffer
    frame.controller.on_off.

    % Signal data storage
    frame.line.raw = zeros(5);
    frame.line.normalized = zeros(5);
    frame.line.logicMask = zeros(5);

    frame.mpu.raw   = zeros(6); 

    frame.ultra.raw = 0;
    frame.ultra.data = 0;
    
    frame.enc.count = 0;
    frame.enc.speed = 0;

    % Control setpoint
    frame.control.speed = 0;
    frame.control.writed_angle = 75;
    
    
end