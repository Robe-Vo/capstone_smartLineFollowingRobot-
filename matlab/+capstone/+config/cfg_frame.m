function frame = cfg_frame()
    % cfg_frame
    % Structure of ONE frame (1 tick)
    % Used for logging, plotting, offline analysis
    
    frame = struct();
    
    % ===== TIME =====
    frame.time.t        = 0;      % [s] time since start (toc)
    frame.time.tickIdx  = 0;      % uint32 tick index
    
    % ===== SENSOR RAW =====
    frame.sensor.line.raw       = zeros(1,5);   % uint8 raw ADC (ESP32)
    frame.sensor.ultra.raw      = 0;             % uint16 raw (mm)
    frame.sensor.mpu.gyro       = zeros(1,3);    % int16
    frame.sensor.mpu.accel      = zeros(1,3);    % int16
    frame.sensor.encoder.count = 0;             % uint8 pulses this frame
    
    % ===== SENSOR PROCESSED =====
    frame.sensor.line.norm      = zeros(1,5);   % normalized [0..1]
    frame.sensor.line.mask      = zeros(1,5);   % logic mask (0/1)
    frame.sensor.line.error     = 0;             % signed error
    frame.sensor.line.quality   = "UNKNOWN";     % OK / LOST / NOISE
    
    frame.sensor.encoder.total  = 0;             % uint32 accumulated in MATLAB
    
    % ===== ROAD / FSM STATE =====
    frame.road.currentId        = 0;
    frame.road.prevId           = 0;
    frame.road.type             = "NONE";        % NORMAL / SPECIAL / STOP
    frame.road.name             = "";
    frame.road.lengthEnc        = 0;
    frame.road.encFromEnter     = 0;
    
    % ===== CONTROLLER STATE =====
    frame.control.type          = "NONE";         % PID / ONOFF / STOP / SPECIAL
    frame.control.tune          = struct();       % copy of tune used this frame
    
    % internal controller buffers 
    frame.control.internal = struct( ...
        "error",        0, ...
        "errorInt",     0, ...
        "errorDer",     0, ...
        "lastError",    0 );
    
    % ===== COMMAND SENT TO ROBOT =====
    frame.cmd.speed_u16         = uint16(0);
    frame.cmd.steerDeg          = uint16(0);
    frame.cmd.cmdByte           = uint8(0);

end
