% Apply the order: LOST > EXIT > SPECIAL > NORMAL
function cond = selectCondition(proc, appState, cfg)
    if proc.quality == "LOST"
        cond = "LOST";
    elseif abs(proc.error - appState.prevError) > cfg.exit.errorJump
        cond = "EXIT";
    elseif appState.roadId == "SPECIAL"
        cond = "SPECIAL";
    else
        cond = "NORMAL";
    end
end
