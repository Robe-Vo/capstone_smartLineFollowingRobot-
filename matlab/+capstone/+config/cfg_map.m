function road = cfg_map()
    % ========================================================================
    % ROADMAP / ROADS CONFIG STRUCT (Matlab)
    % ========================================================================
    % Purpose
    %   Define a list of "roads" (segments) that the robot will execute in order.
    %   Each road includes:
    %     - Geometry (for visualization / debugging / offline checks)
    %     - Controller selection + full controller parameters
    %     - Transition logic to next road (supports AND/OR combination)
    %     - Motion-profile (position-based using encoder pulse count) that outputs
    %       a velocity setpoint for the motor speed PID (cascaded position loop)
    %
    % Data Model
    %   road(i) is a struct with the fields below.
    %
    % ------------------------------------------------------------------------
    % 1) Identification / Metadata
    % ------------------------------------------------------------------------
    % road(i).id
    %   uint32 (or double). Unique index. Recommended: same as i.
    %
    % road(i).name
    %   string/char. Human-readable name (e.g., "LINE_1", "CURVE_R800_1").
    %
    % road(i).type
    %   enum-like string: "STRAIGHT" | "CURVE" | "OBSTACLE_AVOID" | "STOP"
    %   Used for logging and optional special logic (e.g., obstacle behavior).
    %
    % ------------------------------------------------------------------------
    % 2) Geometry (optional for control, useful for verification/plot)
    % ------------------------------------------------------------------------
    % road(i).geom.startP
    %   [x y] in mm (or meters) - must be consistent globally.
    %
    % road(i).geom.endP
    %   [x y] in mm (or meters).
    %
    % road(i).geom.radius
    %   0 for straight, >0 for curve radius.
    %
    % road(i).geom.centerP
    %   [x y] for curve center. For straight, can be [NaN NaN].
    %
    % road(i).geom.length
    %   Scalar length of road segment in mm (or meters).
    %   For encoder-based execution, this is commonly mapped to target pulses.
    %
    % road(i).geom.dir
    %   Optional. +1/-1 direction along the segment if needed.
    %
    % ------------------------------------------------------------------------
    % 3) Controller Selection and Parameters
    % ------------------------------------------------------------------------
    % road(i).controller.type
    %   "PID" | "ONOFF" | "NONE"
    %
    % road(i).controller.steering
    %   Steering controller (line following). This is separate from drive motors.
    %
    %   road(i).controller.steering.centerAngle
    %     Servo neutral angle (deg or servo units). Used as base.
    %
    %   road(i).controller.steering.minAngle, maxAngle
    %     Output clamp for steering command.
    %
    %   road(i).controller.steering.deadband
    %     Deadband on lateral error (e.g., normalized error threshold).
    %
    % --- A) PID line controller (if controller.type == "PID")
    % road(i).controller.pid.kp, ki, kd
    %   PID gains for line error -> steering command (or steering delta).
    %
    % road(i).controller.pid.iLimit
    %   Integral clamp (anti-windup), in same units as integral term.
    %
    % road(i).controller.pid.dFilterAlpha
    %   Derivative low-pass smoothing factor in [0..1]. Example: 0.2.
    %
    % road(i).controller.pid.outLimit
    %   Max magnitude of PID output (before adding centerAngle).
    %
    % --- B) ON/OFF line controller (if controller.type == "ONOFF")
    % Using 5 discrete levels (quantized steering increments).
    %
    % road(i).controller.onoff.levels
    %   1x5 vector (sorted) of steering deltas (deg or servo units), e.g.:
    %     [-12 -6 0 6 12]
    %
    % road(i).controller.onoff.thresholds
    %   1x4 vector defining decision boundaries for |error| mapping to levels.
    %   Example (normalized error):
    %     thresholds = [0.10 0.25 0.45 0.70]
    %   Mapping logic example:
    %     if e in (-t1,t1) -> level 3 (0)
    %     if e in [t1,t2) -> level 4/2
    %     if e in [t2,t3) -> level 5/1
    %     if e >= t4     -> saturate to level 5/1
    %
    % road(i).controller.onoff.hysteresis
    %   Optional hysteresis to prevent chatter, e.g. 0.02 (normalized error).
    %
    % road(i).controller.onoff.rateLimit
    %   Optional slew limit (deg per tick) to smooth discrete jumps.
    %
    % ------------------------------------------------------------------------
    % 4) Drive Motion Profile (position loop using encoder pulse count)
    % ------------------------------------------------------------------------
    % Goal
    %   Use encoder pulses to define how far the robot must move along the road.
    %   Motion profile generates v_ref(t) (speed setpoint) from position error.
    %   Speed PID then tracks v_ref.
    %
    % road(i).drive.encoder
    %   road(i).drive.encoder.ppr
    %     pulses per revolution (effective pulses you count after x2/x4).
    %   road(i).drive.encoder.wheelRadius
    %     wheel radius in mm (or meters). Used to convert pulses->distance.
    %   road(i).drive.encoder.gearRatio
    %     gear ratio if encoder on motor shaft.
    %   road(i).drive.encoder.xMode
    %     1, 2, or 4 for x1/x2/x4 decoding to match counted pulses.
    %
    % road(i).drive.target
    %   road(i).drive.target.pulses
    %     Target pulses for this segment (position target).
    %     If you prefer geometry length:
    %       pulses = length / (2*pi*Rwheel) * ppr * gearRatio * xMode
    %
    % road(i).drive.profile
    %   Position-loop motion profile parameters (generates v_ref):
    %
    %   road(i).drive.profile.vMax
    %     Maximum allowed speed setpoint (mm/s or m/s).
    %
    %   road(i).drive.profile.vMin
    %     Minimum speed floor (to avoid stalling) (>=0).
    %
    %   road(i).drive.profile.aMax
    %     Max acceleration for trapezoidal/S-curve planning (mm/s^2).
    %
    %   road(i).drive.profile.jMax
    %     Optional jerk limit (mm/s^3). If unused, set [] or 0.
    %
    %   road(i).drive.profile.stopWindowPulses
    %     Position tolerance around target pulses considered "arrived".
    %
    %   road(i).drive.profile.kp_pos, ki_pos, kd_pos
    %     Position-loop PID gains:
    %       e_pos = targetPulses - currentPulses
    %       v_ref = PID_pos(e_pos)
    %     Then clamp v_ref to [vMin, vMax].
    %
    %   road(i).drive.profile.iLimit_pos
    %     Integral clamp for position loop.
    %
    %   road(i).drive.profile.outLimit_pos
    %     Clamp for v_ref magnitude (typically equals vMax).
    %
    % road(i).drive.speedPID
    %   Speed PID parameters for each wheel or common:
    %   road(i).drive.speedPID.kp, ki, kd
    %   road(i).drive.speedPID.iLimit
    %   road(i).drive.speedPID.outLimit
    %   road(i).drive.speedPID.ffGain
    %     Optional feedforward mapping v_ref -> PWM baseline.
    %
    % ------------------------------------------------------------------------
    % 5) Road Transition Logic (3 requirements + AND/OR combination)
    % ------------------------------------------------------------------------
    % You have three requirements to change road. Model them as conditions:
    %
    % road(i).transition.cond(1..3)
    %   Each condition is a struct describing what to check.
    %
    % road(i).transition.logic
    %   How to combine conditions: "AND" | "OR" | "CUSTOM"
    %   If "CUSTOM", provide a function handle (pure boolean):
    %     road(i).transition.customFcn = @(c) (c(1) && (c(2) || c(3)));
    %
    % Condition definitions (examples):
    %   Type A: Encoder position reached
    %     .type = "ENCODER_REACH"
    %     .targetPulses = ...
    %     .tolPulses = ...
    %
    %   Type B: Sensor pattern event (e.g., IR mask indicates junction)
    %     .type = "IR_MASK"
    %     .mask = [0 1 1 1 0]   % example
    %     .polarity = "EQUAL"   % or "CONTAINS"
    %     .holdMs = 50          % require stable for N ms
    %
    %   Type C: Obstacle distance threshold
    %     .type = "ULTRASONIC_LT"
    %     .distMm = 150
    %     .holdMs = 100
    %
    % Transition action:
    % road(i).transition.nextId
    %   Next road id to switch to if condition is satisfied.
    %
    % road(i).transition.resetOnEnter
    %   true/false. If true: reset PID integrators, pulse counters, etc.
    %
    % ========================================================================
    % Example: Fully populated sample road struct array
    % ========================================================================
    
    road = struct([]);
    
    % -------------------------
    % ROAD 1: Straight line
    % -------------------------
    road(1).id   = 1;
    road(1).name = "LINE_1";
    road(1).type = "STRAIGHT";
    
    road(1).geom.startP  = [0 0];        % mm
    road(1).geom.endP    = [600 0];      % mm
    road(1).geom.radius  = 0;
    road(1).geom.centerP = [NaN NaN];
    road(1).geom.length  = 2500;
    
    road(1).controller.type = "PID";
    road(1).controller.steering.centerAngle = 85;
    road(1).controller.steering.minAngle    = 55;
    road(1).controller.steering.maxAngle    = 115;
    road(1).controller.steering.deadband    = 0.10;
    
    road(1).controller.pid.kp = 18.0;
    road(1).controller.pid.ki = 0.0;
    road(1).controller.pid.kd = 2.0;
    road(1).controller.pid.iLimit       = 30;
    road(1).controller.pid.dFilterAlpha = 0.2;
    road(1).controller.pid.outLimit     = 20;
    
    
    % Here we directly set target pulses (preferred if you already tuned by pulse count)
    road(1).drive.target.pulses = 5200;
    
    road(1).drive.profile.vMax = 500;        % mm/s
    road(1).drive.profile.vMin = 120;        % mm/s
    road(1).drive.profile.aMax = 2000;       % mm/s^2
    road(1).drive.profile.jMax = 0;          % unused
    road(1).drive.profile.stopWindowPulses = 30;
    
    road(1).drive.profile.kp_pos = 0.25;
    road(1).drive.profile.ki_pos = 0.00;
    road(1).drive.profile.kd_pos = 0.00;
    road(1).drive.profile.iLimit_pos   = 800;
    road(1).drive.profile.outLimit_pos = road(1).drive.profile.vMax;
    
    road(1).drive.speedPID.kp = 0.40;
    road(1).drive.speedPID.ki = 2.50;
    road(1).drive.speedPID.kd = 0.00;
    road(1).drive.speedPID.iLimit   = 200;
    road(1).drive.speedPID.outLimit = 255;   % PWM clamp
    road(1).drive.speedPID.ffGain   = 0.0;
    
    % 3 conditions + combination
    road(1).transition.logic  = "CUSTOM";
    road(1).transition.nextId = 2;
    road(1).transition.resetOnEnter = true;
    
    road(1).transition.cond(1).type       = "ENCODER_REACH";
    road(1).transition.cond(1).targetPulses = road(1).drive.target.pulses;
    road(1).transition.cond(1).tolPulses    = road(1).drive.profile.stopWindowPulses;
    
    road(1).transition.cond(2).type   = "IR_MASK";
    road(1).transition.cond(2).mask   = [0 1 1 1 0];
    road(1).transition.cond(2).polarity = "EQUAL";
    road(1).transition.cond(2).holdMs = 50;
    
    road(1).transition.cond(3).type   = "ULTRASONIC_LT";
    road(1).transition.cond(3).distMm = 150;
    road(1).transition.cond(3).holdMs = 100;
    
    % Custom boolean combine: (Reached pulses) AND ( (IR junction) OR (Obstacle) )
    road(1).transition.customFcn = @(c) (c(1) && (c(2) || c(3)));
    
    % -------------------------
    % ROAD 2: Curve with ON/OFF steering (5-level)
    % -------------------------
    road(2).id   = 2;
    road(2).name = "CURVE_R800_1";
    road(2).type = "CURVE";
    
    road(2).geom.startP  = [600 0];
    road(2).geom.endP    = [600 600];
    road(2).geom.radius  = 800;
    road(2).geom.centerP = [ -200 0 ];
    road(2).geom.length  = 950;
    
    road(2).controller.type = "ONOFF";
    road(2).controller.steering.centerAngle = 85;
    road(2).controller.steering.minAngle    = 55;
    road(2).controller.steering.maxAngle    = 115;
    road(2).controller.steering.deadband    = 0.08;
    
    road(2).controller.onoff.levels     = [-14 -7 0 7 14];
    road(2).controller.onoff.thresholds = [0.08 0.20 0.40 0.65];
    road(2).controller.onoff.hysteresis = 0.02;
    road(2).controller.onoff.rateLimit  = 6;   % deg/tick
    
    road(2).drive.encoder = road(1).drive.encoder;
    road(2).drive.target.pulses = 8200;
    
    road(2).drive.profile.vMax = 350;
    road(2).drive.profile.vMin = 120;
    road(2).drive.profile.aMax = 1500;
    road(2).drive.profile.jMax = 0;
    road(2).drive.profile.stopWindowPulses = 35;
    
    road(2).drive.profile.kp_pos = 0.22;
    road(2).drive.profile.ki_pos = 0.00;
    road(2).drive.profile.kd_pos = 0.00;
    road(2).drive.profile.iLimit_pos   = 800;
    road(2).drive.profile.outLimit_pos = road(2).drive.profile.vMax;
    
    road(2).drive.speedPID = road(1).drive.speedPID;
    
    road(2).transition.logic = "AND";
    road(2).transition.nextId = 3;
    road(2).transition.resetOnEnter = true;
    
    road(2).transition.cond(1).type = "ENCODER_REACH";
    road(2).transition.cond(1).targetPulses = road(2).drive.target.pulses;
    road(2).transition.cond(1).tolPulses    = road(2).drive.profile.stopWindowPulses;
    
    road(2).transition.cond(2).type = "IR_MASK";
    road(2).transition.cond(2).mask = [1 1 1 1 1];     % example stop-line/full black
    road(2).transition.cond(2).polarity = "EQUAL";
    road(2).transition.cond(2).holdMs = 60;
    
    road(2).transition.cond(3).type = "NONE";          % unused slot (still 3 total if you keep fixed-size)
    road(2).transition.customFcn = [];
    
    % -------------------------
    % ROAD 3: Stop
    % -------------------------
    road(3).id   = 3;
    road(3).name = "STOP_1";
    road(3).type = "STOP";
    
    road(3).geom.startP  = [600 600];
    road(3).geom.endP    = [600 600];
    road(3).geom.radius  = 0;
    road(3).geom.centerP = [NaN NaN];
    road(3).geom.length  = 0;
    
    road(3).controller.type = "NONE";
    road(3).controller.steering.centerAngle = 85;
    road(3).controller.steering.minAngle    = 55;
    road(3).controller.steering.maxAngle    = 115;
    road(3).controller.steering.deadband    = 0.0;
    
    road(3).drive.encoder = road(1).drive.encoder;
    road(3).drive.target.pulses = 0;
    
    road(3).drive.profile.vMax = 0;
    road(3).drive.profile.vMin = 0;
    road(3).drive.profile.aMax = 0;
    road(3).drive.profile.jMax = 0;
    road(3).drive.profile.stopWindowPulses = 0;
    
    road(3).drive.profile.kp_pos = 0;
    road(3).drive.profile.ki_pos = 0;
    road(3).drive.profile.kd_pos = 0;
    road(3).drive.profile.iLimit_pos   = 0;
    road(3).drive.profile.outLimit_pos = 0;
    
    road(3).drive.speedPID = road(1).drive.speedPID;
    
    road(3).transition.logic  = "OR";
    road(3).transition.nextId = 0;            % 0 = end
    road(3).transition.resetOnEnter = false;
    
    road(3).transition.cond(1).type = "NONE";
    road(3).transition.cond(2).type = "NONE";
    road(3).transition.cond(3).type = "NONE";
    road(3).transition.customFcn = [];

end