function plan = roadPlan()

    % ===== SEGMENT 1 =====
    plan(1).id = "LINE_1";
    plan(1).lengthEnc = 300;        % encoder ticks
    plan(1).controller = "PID";
    plan(1).tuneId = "LINE_FAST";
    
    plan(1).transition.mode = "AND";  % AND / OR
    plan(1).transition.conds = { ...
        @cond_encReached, ...
        @cond_centerStable ...
    };
    plan(1).transition.next = 2;      % go to segment 2
    
    % ===== SEGMENT 2 =====
    plan(2).id = "CURVE_R500";
    plan(2).lengthEnc = 180;
    plan(2).controller = "PID";
    plan(2).tuneId = "CURVE_SOFT";
    
    plan(2).transition.mode = "OR";
    plan(2).transition.conds = { ...
        @cond_turnSignal, ...
        @cond_encReached ...
    };
    plan(2).transition.next = 3;
    
    % ===== SEGMENT 3 =====
    plan(3).id = "END";
    plan(3).lengthEnc = Inf;
    plan(3).controller = "STOP";
    plan(3).transition = [];
end
