function ok = roadConditionCheck(cond, appState, roadRt, meas, proc, curRoad)
% cond: string or struct (ở đây dùng string tối thiểu)
    
    ok = false;
    
    if isstruct(cond)
        % mở rộng sau (AND/OR) nếu cần
        return;
    end
    
    c = string(cond);
    
    switch c
        case "ENC_REACHED"
            % so lengthEnc với encoder total tích lũy trong MATLAB
            traveled = double(appState.encTotal - roadRt.enterEncTotal);
            ok = traveled >= double(curRoad.lengthEnc);
    
        case "LINE_FOUND"
            ok = isfield(proc,"quality") && string(proc.quality) == "OK";
    
        case "TIMEOUT"
            if isfield(curRoad,"param") && isfield(curRoad.param,"timeoutS")
                ok = toc(roadRt.enterTic) >= double(curRoad.param.timeoutS);
            end
    
        otherwise
            ok = false;
    end
end
