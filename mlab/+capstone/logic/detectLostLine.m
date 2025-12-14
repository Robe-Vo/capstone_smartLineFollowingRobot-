function s = detectLostLine(mask, cfg) %#ok<INUSD>
    if any(mask), s = "OK"; else, s = "LOST"; end
end
