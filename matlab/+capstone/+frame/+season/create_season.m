function season = create_season(frameNum)
% Create a new season struct for each run
% idx


    % Init season
    season = struct([]);

    % Declare season structure
    season.startTime = datetime();
    season.endTime = datetime();
    season.duration = season.startTime - season.endTime;
    season.error

    % Hardware setup
    season.hardware.wheelDiameter = 72; % (mm)
    season.hardware.diffRatio = 12/30;
    season.hardware.motorRatio = 9.6;
    season.hardware.steer.maxAngle = 110;
    season.hardware.steer.minAngle = 55;
    season.hardware.steer.middleAngle = 75;
    season.hardware.drive.maxSpeed = 1000000;
    season.hardware.drive.minSpeed = 0;
    season.hardware.drive.deadBand = 0;
    % Robot PID speed params
    season.hardware.drive.pid.kp = 0;
    season.hardware.drive.pid.ki = 0;
    season.hardware.drive.pid.kd = 0;


    % Calibration params
    season.calib.line.weightMask = [-2 -1 0 1 2];
    season.calib.line.offset = 0;
    season.calib.line.scale = 1;

    season.calib.ultra.offset = 0;
    season.calib.ultra.scale = 1;

    season.calib.mpu.threshold = 10000; % Value for robot to start avoid-obstacle program
    season.calib.mpu.offset = 0;
    season.calib.mpu.scale = 1;
    
    season.cfgs  = capstone.config.create_cfg();
    season.frame = capstone.frame.frame.create_frame();
    


end