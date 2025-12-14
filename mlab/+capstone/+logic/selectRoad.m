% To know which road is it
function roadId = selectRoad(roadMem, cfg, appState)

    roadId = appState.roadId;
    
    meanErr = mean(abs(roadMem.errorHist));
    
    if roadMem.encSum > cfg.detect.special.minEnc ...
            && sum(roadMem.lastMask) >= cfg.detect.special.minMaskSum
        roadId = "SPECIAL";
    
    elseif roadMem.encSum > cfg.detect.curve.minEnc ...
            && meanErr > cfg.detect.curve.minMeanError
        roadId = "CURVE";
    
    elseif roadMem.encSum > cfg.detect.straight.minEnc ...
            && meanErr < cfg.detect.straight.maxMeanError
        roadId = "STRAIGHT";
    end
end
