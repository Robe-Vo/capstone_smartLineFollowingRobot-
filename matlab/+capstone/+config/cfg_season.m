function season = cfg_season(robot, roads)
    %CFG_SEASON Create season struct: setup + runtime + logs.
    season = struct();
    
    %% ===== Setup =====
    season.setup = struct();
    season.setup.robot = robot;
    season.setup.roads = roads;
    
    season.setup.comm = struct();
    season.setup.comm.transport = "bluetooth";   % current
    season.setup.comm.btName = "Behind the scream"; % TODO if you keep
    season.setup.comm.channel = NaN;
    season.setup.comm.protocol_version = "v0";   % TODO lock with firmware
    
    %% ===== Runtime =====
    season.runtime = struct();
    season.runtime.mode = "IDLE";
    season.runtime.road_idx = 1;
    season.runtime.stable = NaN;
    season.runtime.lostLine = NaN;
    season.runtime.stop_reason = "";
    season.runtime.encoder_total = 0;            % global accumulated (if you want)
    season.runtime.t_start = NaN;
    season.runtime.t_end = NaN;
    
    %% ===== Log =====
    season.log = struct();
    season.log.frames = repmat(struct(), 0, 1);  % dynamic append
    season.log.events = repmat(struct('t',NaN,'name',"",'data',struct()), 0, 1);

end
