function tf = cond_encReached(ctx)
    % ctx: context runtime (xem mục IV)
    
    tf = ctx.roadEnc >= ctx.segment.lengthEnc;
end
