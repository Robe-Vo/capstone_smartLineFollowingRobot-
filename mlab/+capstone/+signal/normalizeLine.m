function [lineNorm, mask] = normalizeLine(lineRaw, offset, scale, threshold)
    % Normalize line sensors using white/black calibration
    
    r = double(lineRaw(:)).';
    lineNorm = (r - offset) .* scale;
    lineNorm = max(0, min(1, lineNorm));
    mask = lineNorm > threshold;
end
