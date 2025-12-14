function [out, st] = onoffSteerController(input, tune, st, cfg) %#ok<INUSD>
    if isempty(st)
        st.lastU = 0.0;
    end
    
    if ~any(input.mask)
        out = struct("steerOffsetDeg",0.0,"speedScale",1.0);
        return;
    end
    
    e = input.error;
    
    thrHi = tune.threshold + tune.hysteresis;
    thrLo = tune.threshold - tune.hysteresis;
    
    u = 0;
    if abs(e) <= thrLo
        u = 0;
    elseif e > thrHi
        % line is to left/right depends on your sign; keep consistent with computeError()
        % using steerOffsetDeg here; conversion to absolute angle in App.
        u = -abs(double(tune.rightMaxU16) - double(tune.centerU16));
    elseif e < -thrHi
        u =  abs(double(tune.centerU16) - double(tune.leftMaxU16));
    else
        u = st.lastU; % hysteresis band -> hold
    end
    
    st.lastU = u;
    
    out = struct("steerOffsetDeg",u,"speedScale",1.0);
end
