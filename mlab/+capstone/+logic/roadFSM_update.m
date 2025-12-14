function [appState, roadRt] = roadFSM_update(appState, roadRt, roadMap, meas, proc)

    curRoad = roadMap(appState.currentRoadId);
    
    for k = 1:numel(curRoad.transition)
        tr = curRoad.transition{k};
    
        if roadConditionCheck(tr.cond, appState, roadRt, meas, proc, curRoad)
            prev = appState.currentRoadId;
            appState.prevRoadId = prev;
    
            if (isstring(tr.next) || ischar(tr.next)) && string(tr.next) == "PREV"
                appState.currentRoadId = prev;
            else
                appState.currentRoadId = double(tr.next);
            end
    
            roadRt = initRoadRuntime();
            roadRt.enterEncTotal = appState.encTotal; % mốc encoder total
            break;
        end
    end

end
