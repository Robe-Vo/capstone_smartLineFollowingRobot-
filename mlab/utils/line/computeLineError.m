function e = computeLineError(lineNorm, mask, positions)
    w = lineNorm;
    w(~mask) = 0;
    if all(w == 0)
        e = 0;
        return;
    end
    pos = sum(positions .* w) / sum(w);
    e   = -pos;   % đổi dấu nếu ngược chiều
end