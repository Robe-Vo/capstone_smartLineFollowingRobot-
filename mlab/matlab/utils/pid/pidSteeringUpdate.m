function [uDeg, pidState] = pidSteeringUpdate(e, dt, pid, pidState, mask)
    if ~any(mask)
        uDeg = 0.0;
        return;
    end

    e_eff = e;

    if pidState.first
        pidState.first = false;
        pidState.prevE = e_eff;
    end

    % Tích phân + chống bão hòa
    pidState.intE = pidState.intE + e_eff * dt;
    pidState.intE = max(-pid.intMax, min(pid.intMax, pidState.intE));

    % Đạo hàm
    de = (e_eff - pidState.prevE) / dt;
    pidState.prevE = e_eff;

    % PID
    uDeg = pid.Kp * e_eff + pid.Ki * pidState.intE + pid.Kd * de;

    % Giới hạn đầu ra
    uDeg = max(-pid.outMax, min(pid.outMax, uDeg));
end
