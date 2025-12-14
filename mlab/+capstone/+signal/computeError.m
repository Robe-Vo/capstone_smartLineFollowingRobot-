function e = computeError(lineNorm, mask, positions)
    % Weighted-average line position error
    
    w = lineNorm;
    w(~mask) = 0;
    
    if all(w == 0)
        e = 0;
        return;
    end
    
    pos = sum(positions .* w) / sum(w);
    e = -pos;   % giữ cùng quy ước dấu với code cũ
end
