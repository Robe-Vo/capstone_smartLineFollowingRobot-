function gain = selectGain(ctrlType, road, cond, G) %#ok<INUSD>

    if cond.line == "LOST"
        gain = G.LOST_LINE.(ctrlType);
        return;
    end

    if cond.stability == "UNSTABLE"
        gain = G.UNSTABLE.(ctrlType);
        return;
    end

    gain = G.GLOBAL.(ctrlType);
end
