function [out, st] = pidController(input, gain, st, cfg) %#ok<INUSD>

    if isempty(st)
        st = struct("intE",0.0,"prevE",input.error,"first",true);
    end

    if ~any(input.mask)
        out = struct("steerOffsetDeg",0.0,"speedScale",1.0);
        return;
    end

    if st.first
        st.first = false;
        st.prevE = input.error;
    end

    st.intE = st.intE + input.error * input.dt;
    st.intE = max(-gain.intMax, min(gain.intMax, st.intE));

    de = (input.error - st.prevE) / max(input.dt, 1e-3);
    st.prevE = input.error;

    u = gain.Kp*input.error + gain.Ki*st.intE + gain.Kd*de;

    % steering offset in "deg-like" unit (same as your old code)
    out = struct("steerOffsetDeg", u, "speedScale", 1.0);
end
