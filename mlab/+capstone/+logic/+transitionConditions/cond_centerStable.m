function tf = cond_centerStable(ctx)

    N = ctx.cfg.road.detect.stableFrames;
    hist = ctx.frameBuf.proc;
    
    centerHits = arrayfun(@(f) f.mask(3), hist(end-N+1:end));
    tf = sum(centerHits) == N;
end
