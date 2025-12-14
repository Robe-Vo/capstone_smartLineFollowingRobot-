function tune = selectTune(appState, tuneDB, ctrlType, condKey, roadId)
    ctrlType = char(ctrlType);
    condKey  = char(condKey);
    
    if appState.tune.useGlobal || ~isfield(tuneDB,"road") || isempty(tuneDB.road)
        tune = tuneDB.global.(ctrlType).(condKey);
        return;
    end
    
    rid = double(roadId);
    if rid <= numel(tuneDB.road) && isfield(tuneDB.road(rid), ctrlType) && isfield(tuneDB.road(rid).(ctrlType), condKey)
        tune = tuneDB.road(rid).(ctrlType).(condKey);
    else
        tune = tuneDB.global.(ctrlType).(condKey);
    end
end
