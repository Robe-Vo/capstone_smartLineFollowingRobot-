function run = makeRunLog(runName, cfg)
    run = struct();
    run.name = string(runName);
    run.cfgSnapshot = cfg;
    run.startTime = datetime("now");
    run.frames = repmat(capstone.types.makeRuntimeFrame(), 0, 1);
end
