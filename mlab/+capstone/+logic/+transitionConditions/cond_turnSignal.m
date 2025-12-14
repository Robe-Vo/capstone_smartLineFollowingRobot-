% T-junction
function tf = cond_turnSignal(ctx)
    mask = ctx.proc.mask;
    tf = sum(mask) >= 3;  % junction pattern
end
