function [out, st] = pidSteerController(input, tune, st, cfg) %#ok<INUSD>

    if isempty(st)
        st.intE = 0.0;
        st.prevE = input.error;
        st.dF = 0.0;
    end
    
    if ~any(input.mask)
        out = struct("steerOffsetDeg",0.0,"speedScale",1.0);
        return;
    end
    
    e = input.error;
    if abs(e) < tune.deadband
        e = 0;
    end
    
    st.intE = st.intE + e*input.dt;
    st.intE = max(-tune.intMax, min(tune.intMax, st.intE));
    
    de = (e - st.prevE) / max(input.dt,1e-3);
    st.prevE = e;
    
    % optional D filter
    a = tune.dAlpha;
    st.dF = a*de + (1-a)*st.dF;
    
    u = tune.Kp*e + tune.Ki*st.intE + tune.Kd*st.dF;
    
    % clamp output
    u = max(-tune.outMaxDeg, min(tune.outMaxDeg, u));
    
    out = struct("steerOffsetDeg",u,"speedScale",1.0);
end
