function [outputArg1,outputArg2] = normalizeLine(inputArg1,inputArg2)
%NORMALIZELINE Summary of this function goes here
%   Detailed explanation goes here
outputArg1 = inputArg1;
outputArg2 = inputArg2;
end

function [lineNorm, mask] = normalizeLine(lineRaw, offset, scale, threshold)
    r = double(lineRaw(:)).';
    lineNorm = (r - offset) .* scale;
    lineNorm = max(0, min(1, lineNorm));      % clamp 0..1
    mask = lineNorm > threshold;              % active sensors (on line)
end