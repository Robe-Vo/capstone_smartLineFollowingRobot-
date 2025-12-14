function key = conditionKey(rt, cfg) %#ok<INUSD>
    % tối thiểu: LOST nếu mask rỗng, còn lại NORMAL
    if ~any(rt.proc.mask)
        key = "LOST";
    else
        key = "NORMAL";
    end
end
