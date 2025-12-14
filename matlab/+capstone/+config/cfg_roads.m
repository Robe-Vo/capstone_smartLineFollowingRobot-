function roads = cfg_roads(robot)
    %CFG_ROADS Create road structs with unified geometry fields for all segments.
    %
    % - Unified geometry struct is used for LINE / ARC / JUNCTION segments.
    % - Params that do not apply are set to NaN (or 0 where appropriate).
    % - All non-tuning constants / map construction math are grouped at the end
    %   of this file in local helper functions, to keep tuning clean.
    
    %% ===== 0) Build map primitives (from map.m logic; no numeric hardcode here) =====
    M = local_buildMapPrimitives();
    
    %% ===== 1) Compute derived geometry for continuity (Arc5, Arc8) + endpoints =====
    G = local_computeDerivedGeometry(M);
    
    %% ===== 2) Allocate road array =====
    roads = repmat(struct(), 1, 9);
    
    %% ===== 3) Templates =====
    geom0    = local_geomTemplate();                % unified geometry
    profile0 = local_profileTemplate(robot);        % placeholder (you fill later)
    ctrl0    = local_controlTemplate();             % placeholder tuning (you fill later)
    sw0      = local_switchTemplate();              % placeholder switching logic
    
    %% ===== 4) Road 1: Line1 =====
    i = 1;
    roads(i).id     = "1_LINE1";
    roads(i).type   = "LINE";
    roads(i).branch = "MAIN";
    roads(i).geometry = geom0;
    roads(i).geometry.kind = "LINE";
    
    roads(i).geometry.p_s = G.p1_s;
    roads(i).geometry.p_e = G.p1_e;
    
    % LINE-only fields (keep unified struct; others remain NaN)
    roads(i).geometry.len_mm = norm(roads(i).geometry.p_e - roads(i).geometry.p_s);
    roads(i).geometry.tan_s  = local_unit(roads(i).geometry.p_e - roads(i).geometry.p_s);
    roads(i).geometry.tan_e  = roads(i).geometry.tan_s;
    roads(i).geometry.nor_s  = local_rot90(roads(i).geometry.tan_s);
    roads(i).geometry.nor_e  = roads(i).geometry.nor_s;
    
    roads(i).profile = profile0;
    roads(i).control = ctrl0;
    roads(i).switch  = sw0;
    roads(i).switch.next = "2_ARC2";
    
    %% ===== 5) Road 2: Arc2 =====
    i = 2;
    roads(i).id     = "2_ARC2";
    roads(i).type   = "ARC";
    roads(i).branch = "MAIN";
    roads(i).geometry = geom0;
    roads(i).geometry.kind = "ARC";
    
    roads(i).geometry.cen   = G.c2_cen;
    roads(i).geometry.r_mm  = G.c2_r;
    roads(i).geometry.th_s  = G.c2_s;
    roads(i).geometry.th_e  = G.c2_e;
    roads(i).geometry.dir   = local_arcDir(G.c2_s, G.c2_e); % +1 ccw / -1 cw heuristic
    
    roads(i).geometry.p_s   = G.p2_s;
    roads(i).geometry.p_e   = G.p2_e;
    roads(i).geometry.len_mm = abs(G.c2_r) * abs(G.c2_e - G.c2_s);
    
    roads(i).geometry.tan_s = local_arcTangent(G.c2_s, roads(i).geometry.dir);
    roads(i).geometry.tan_e = local_arcTangent(G.c2_e, roads(i).geometry.dir);
    roads(i).geometry.nor_s = local_rot90(roads(i).geometry.tan_s);
    roads(i).geometry.nor_e = local_rot90(roads(i).geometry.tan_e);
    
    roads(i).profile = profile0;
    roads(i).control = ctrl0;
    roads(i).switch  = sw0;
    roads(i).switch.next = "3_LINE3";
    
    %% ===== 6) Road 3: Line3 (ends at junction) =====
    i = 3;
    roads(i).id     = "3_LINE3";
    roads(i).type   = "LINE";
    roads(i).branch = "MAIN";
    roads(i).geometry = geom0;
    roads(i).geometry.kind = "LINE";
    
    roads(i).geometry.p_s = G.p3_s;
    roads(i).geometry.p_e = G.p3_e; % junction point pJ
    
    roads(i).geometry.len_mm = norm(roads(i).geometry.p_e - roads(i).geometry.p_s);
    roads(i).geometry.tan_s  = local_unit(roads(i).geometry.p_e - roads(i).geometry.p_s);
    roads(i).geometry.tan_e  = roads(i).geometry.tan_s;
    roads(i).geometry.nor_s  = local_rot90(roads(i).geometry.tan_s);
    roads(i).geometry.nor_e  = roads(i).geometry.nor_s;
    
    roads(i).profile = profile0;
    roads(i).control = ctrl0;
    roads(i).switch  = sw0;
    roads(i).switch.nextA  = "4_ARC4";
    roads(i).switch.nextB  = "7_ARC7";
    roads(i).switch.policy = "preselected"; % TODO
    
    %% ===== 7) Road 4: Arc4 (branch A) =====
    i = 4;
    roads(i).id     = "4_ARC4";
    roads(i).type   = "ARC";
    roads(i).branch = "A";
    roads(i).geometry = geom0;
    roads(i).geometry.kind = "ARC";
    
    roads(i).geometry.cen   = G.c4_cen;
    roads(i).geometry.r_mm  = G.c4_r;
    roads(i).geometry.th_s  = G.c4_s;
    roads(i).geometry.th_e  = G.c4_e;
    roads(i).geometry.dir   = local_arcDir(G.c4_s, G.c4_e);
    
    roads(i).geometry.p_s   = G.p4_s;
    roads(i).geometry.p_e   = G.p4_e;
    roads(i).geometry.len_mm = abs(G.c4_r) * abs(G.c4_e - G.c4_s);
    
    roads(i).geometry.tan_s = local_arcTangent(G.c4_s, roads(i).geometry.dir);
    roads(i).geometry.tan_e = local_arcTangent(G.c4_e, roads(i).geometry.dir);
    roads(i).geometry.nor_s = local_rot90(roads(i).geometry.tan_s);
    roads(i).geometry.nor_e = local_rot90(roads(i).geometry.tan_e);
    
    roads(i).profile = profile0;
    roads(i).control = ctrl0;
    roads(i).switch  = sw0;
    roads(i).switch.next = "5_ARC5";
    
    %% ===== 8) Road 5: Arc5 (branch A, continuity computed, r=800) =====
    i = 5;
    roads(i).id     = "5_ARC5";
    roads(i).type   = "ARC";
    roads(i).branch = "A";
    roads(i).geometry = geom0;
    roads(i).geometry.kind = "ARC";
    
    roads(i).geometry.cen   = G.c5_cen;
    roads(i).geometry.r_mm  = G.c5_r;
    roads(i).geometry.th_s  = G.c5_s;
    roads(i).geometry.th_e  = G.c5_e;
    roads(i).geometry.dir   = local_dirFromString(G.dir5);
    
    roads(i).geometry.p_s   = G.p5_s;
    roads(i).geometry.p_e   = G.p5_e;
    roads(i).geometry.len_mm = abs(G.c5_r) * abs(G.c5_e - G.c5_s);
    
    roads(i).geometry.tan_s = local_arcTangent(G.c5_s, roads(i).geometry.dir);
    roads(i).geometry.tan_e = local_arcTangent(G.c5_e, roads(i).geometry.dir);
    roads(i).geometry.nor_s = local_rot90(roads(i).geometry.tan_s);
    roads(i).geometry.nor_e = local_rot90(roads(i).geometry.tan_e);
    
    roads(i).profile = profile0;
    roads(i).control = ctrl0;
    roads(i).switch  = sw0;
    roads(i).switch.next = "6_LINE6";
    
    %% ===== 9) Road 6: Line6 (branch A) =====
    i = 6;
    roads(i).id     = "6_LINE6";
    roads(i).type   = "LINE";
    roads(i).branch = "A";
    roads(i).geometry = geom0;
    roads(i).geometry.kind = "LINE";
    
    roads(i).geometry.p_s = G.p6_s;
    roads(i).geometry.p_e = G.p6_e;
    
    roads(i).geometry.len_mm = norm(roads(i).geometry.p_e - roads(i).geometry.p_s);
    roads(i).geometry.tan_s  = local_unit(roads(i).geometry.p_e - roads(i).geometry.p_s);
    roads(i).geometry.tan_e  = roads(i).geometry.tan_s;
    roads(i).geometry.nor_s  = local_rot90(roads(i).geometry.tan_s);
    roads(i).geometry.nor_e  = roads(i).geometry.nor_s;
    
    roads(i).profile = profile0;
    roads(i).control = ctrl0;
    roads(i).switch  = sw0;
    roads(i).switch.next = ""; % end
    
    %% ===== 10) Road 7: Arc7 (branch B) =====
    i = 7;
    roads(i).id     = "7_ARC7";
    roads(i).type   = "ARC";
    roads(i).branch = "B";
    roads(i).geometry = geom0;
    roads(i).geometry.kind = "ARC";
    
    roads(i).geometry.cen   = G.c7_cen;
    roads(i).geometry.r_mm  = G.c7_r;
    roads(i).geometry.th_s  = G.c7_s;
    roads(i).geometry.th_e  = G.c7_e;
    roads(i).geometry.dir   = local_arcDir(G.c7_s, G.c7_e);
    
    roads(i).geometry.p_s   = G.p7_s;
    roads(i).geometry.p_e   = G.p7_e;
    roads(i).geometry.len_mm = abs(G.c7_r) * abs(G.c7_e - G.c7_s);
    
    roads(i).geometry.tan_s = local_arcTangent(G.c7_s, roads(i).geometry.dir);
    roads(i).geometry.tan_e = local_arcTangent(G.c7_e, roads(i).geometry.dir);
    roads(i).geometry.nor_s = local_rot90(roads(i).geometry.tan_s);
    roads(i).geometry.nor_e = local_rot90(roads(i).geometry.tan_e);
    
    roads(i).profile = profile0;
    roads(i).control = ctrl0;
    roads(i).switch  = sw0;
    roads(i).switch.next = "8_ARC8";
    
    %% ===== 11) Road 8: Arc8 (branch B, continuity computed, r=800) =====
    i = 8;
    roads(i).id     = "8_ARC8";
    roads(i).type   = "ARC";
    roads(i).branch = "B";
    roads(i).geometry = geom0;
    roads(i).geometry.kind = "ARC";
    
    roads(i).geometry.cen   = G.c8_cen;
    roads(i).geometry.r_mm  = G.c8_r;
    roads(i).geometry.th_s  = G.c8_s;
    roads(i).geometry.th_e  = G.c8_e;
    roads(i).geometry.dir   = local_dirFromString(G.dir8);
    
    roads(i).geometry.p_s   = G.p8_s;
    roads(i).geometry.p_e   = G.p8_e;
    roads(i).geometry.len_mm = abs(G.c8_r) * abs(G.c8_e - G.c8_s);
    
    roads(i).geometry.tan_s = local_arcTangent(G.c8_s, roads(i).geometry.dir);
    roads(i).geometry.tan_e = local_arcTangent(G.c8_e, roads(i).geometry.dir);
    roads(i).geometry.nor_s = local_rot90(roads(i).geometry.tan_s);
    roads(i).geometry.nor_e = local_rot90(roads(i).geometry.tan_e);
    
    roads(i).profile = profile0;
    roads(i).control = ctrl0;
    roads(i).switch  = sw0;
    roads(i).switch.next = "9_LINE9";
    
    %% ===== 12) Road 9: Line9 (branch B) =====
    i = 9;
    roads(i).id     = "9_LINE9";
    roads(i).type   = "LINE";
    roads(i).branch = "B";
    roads(i).geometry = geom0;
    roads(i).geometry.kind = "LINE";
    
    roads(i).geometry.p_s = G.p9_s;
    roads(i).geometry.p_e = G.p9_e;
    
    roads(i).geometry.len_mm = norm(roads(i).geometry.p_e - roads(i).geometry.p_s);
    roads(i).geometry.tan_s  = local_unit(roads(i).geometry.p_e - roads(i).geometry.p_s);
    roads(i).geometry.tan_e  = roads(i).geometry.tan_s;
    roads(i).geometry.nor_s  = local_rot90(roads(i).geometry.tan_s);
    roads(i).geometry.nor_e  = roads(i).geometry.nor_s;
    
    roads(i).profile = profile0;
    roads(i).control = ctrl0;
    roads(i).switch  = sw0;
    roads(i).switch.next = ""; % end

end

%% ========================================================================
%  Local helpers (grouped for convenient tuning / editing)
%  - Everything not "tuning" or not "road struct schema" stays here.
%  - You can paste/copy your map.m primitive definitions into
%    local_buildMapPrimitives() without changing cfg_roads() above.
% ========================================================================

function geom = local_geomTemplate()
    % Unified geometry struct for ALL road types.
    % Non-applicable fields must remain NaN (or 0 if you prefer).
    geom = struct();
    geom.kind   = "";              % "LINE" / "ARC" / "JUNCTION" (if later)
    geom.branch = "";              % "MAIN"/"A"/"B" (optional mirror of roads(i).branch)
    
    % Key points
    geom.p_s = [NaN NaN];
    geom.p_e = [NaN NaN];
    
    % Common derived
    geom.len_mm = NaN;
    
    % Line features (valid for LINE)
    geom.tan_s = [NaN NaN];
    geom.tan_e = [NaN NaN];
    geom.nor_s = [NaN NaN];
    geom.nor_e = [NaN NaN];
    
    % Arc features (valid for ARC)
    geom.cen  = [NaN NaN];
    geom.r_mm = NaN;
    geom.th_s = NaN;
    geom.th_e = NaN;
    geom.dir  = NaN;               % +1 ccw, -1 cw
    
    % Reserved (keep unified even if unused now)
    geom.kappa_s = NaN;            % curvature at start (1/r)
    geom.kappa_e = NaN;            % curvature at end   (1/r)
    geom.note = "";

end

function prof = local_profileTemplate(robot)
    prof = struct();
    prof.encoder_total = NaN;
    prof.v_min = NaN;
    prof.v_max = NaN;
    prof.t_total = NaN;
    
    prof.v_end_prev = NaN;
    prof.unit = "mmps";     % keep consistent with your main controller
    prof.ts   = robot.ts.comm_s;
    
    prof.profile = [];      % will be filled by cfg_motionProfile()
    prof.gen = struct('name',"generateProfile",'args',struct('vmax',NaN,'vmin',NaN,'t_total',NaN,'encoder_total',NaN,'v0',NaN,'v1',NaN));
end

function ctrl = local_controlTemplate()
    % Keep placeholders; fill later in road-specific tuning.
    emptyPID  = struct('Kp',NaN,'Ki',NaN,'Kd',NaN,'u_min',NaN,'u_max',NaN,'i_limit',NaN,'ff',NaN);
    emptyOnOff = struct('th',NaN,'hys',NaN,'u_low',NaN,'u_high',NaN);
    
    ctrl = struct();
    ctrl.drive = struct('mode',"SPEED",'pid',emptyPID);
    ctrl.steer = struct('mode',"PID",'pid',emptyPID,'onoff',emptyOnOff);
end

function sw = local_switchTemplate()
    sw = struct();
    sw.entry  = "";
    sw.exit   = "";
    sw.next   = "";
    sw.nextA  = "";
    sw.nextB  = "";
    sw.policy = "";
end

function M = local_buildMapPrimitives()
    % Paste your original map.m "primitives" here:
    % - l1_s,l1_e,l3_s,l3_e,l6_s,l6_e,l9_s,l9_e
    % - c2_cen,c2_r,c2_s,c2_e
    % - c4_cen,c4_r,c4_s,c4_e
    % - c7_cen,c7_r,c7_s,c7_e
    %
    % Keep as symbols/expressions; do NOT hardcode numeric values in cfg_roads.
    
    M = struct();
    
    % ---- Lines (start/end) ----
    M.l1_s = [NaN NaN];  M.l1_e = [NaN NaN];
    M.l3_s = [NaN NaN];  M.l3_e = [NaN NaN];
    M.l6_s = [NaN NaN];  M.l6_e = [NaN NaN];
    M.l9_s = [NaN NaN];  M.l9_e = [NaN NaN];
    
    % ---- Arcs (center, radius, angles) ----
    M.c2_cen = [NaN NaN]; M.c2_r = NaN; M.c2_s = NaN; M.c2_e = NaN;
    M.c4_cen = [NaN NaN]; M.c4_r = NaN; M.c4_s = NaN; M.c4_e = NaN;
    M.c7_cen = [NaN NaN]; M.c7_r = NaN; M.c7_s = NaN; M.c7_e = NaN;
    
    % ---- Constant used in your snippet (kept for parity) ----
    M.t = tan(pi/8);

end

function G = local_computeDerivedGeometry(M)
    % Implements the continuity fix for Arc5 and Arc8, and computes all segment endpoints.
    
    G = struct();
    
    % ===== Segment 1 endpoints
    G.p1_s = M.l1_s;
    G.p1_e = M.l1_e;
    
    % ===== Segment 2 endpoints (Arc2)
    G.c2_cen = M.c2_cen; G.c2_r = M.c2_r; G.c2_s = M.c2_s; G.c2_e = M.c2_e;
    G.p2_s = M.c2_cen + M.c2_r*[cos(M.c2_s) sin(M.c2_s)];
    G.p2_e = M.c2_cen + M.c2_r*[cos(M.c2_e) sin(M.c2_e)];
    
    % ===== Segment 3 endpoints
    G.p3_s = M.l3_s;
    G.p3_e = M.l3_e;
    G.pJ   = G.p3_e;
    
    % ===== Segment 4 (Arc4) endpoints
    G.c4_cen = M.c4_cen; G.c4_r = M.c4_r; G.c4_s = M.c4_s; G.c4_e = M.c4_e;
    G.p4_s = G.pJ;
    G.p4_e = M.c4_cen + M.c4_r*[cos(M.c4_e) sin(M.c4_e)];
    
    % ===== Segment 7 (Arc7) endpoints
    G.c7_cen = M.c7_cen; G.c7_r = M.c7_r; G.c7_s = M.c7_s; G.c7_e = M.c7_e;
    G.p7_s = G.pJ;
    G.p7_e = M.c7_cen + M.c7_r*[cos(M.c7_e) sin(M.c7_e)];
    
    % ===== Segment 6 line start for continuity target of Arc5
    G.p6_s = M.l6_s;
    G.p6_e = M.l6_e;
    
    % ===== Segment 9 line start for continuity target of Arc8
    G.p9_s = M.l9_s;
    G.p9_e = M.l9_e;
    
    % ===== Arc5 compute (connect p4_e -> l6_s) with r=800
    r = 800;
    P1 = G.p4_e;           % MUST equal Arc4 end
    P2 = G.p6_s;           % MUST equal Line6 start
    
    [d,Mm,u,perp,h,C1,C2] = local_twoCircleCenters(P1, P2, r);
    
    theta4e = M.c4_e;
    t_des = [-sin(theta4e) cos(theta4e)];  % CCW tangent at arc4 end
    
    [c5_cen, dir5] = local_chooseArcCenterDir(P1, C1, C2, t_des);
    c5_s = atan2(P1(2)-c5_cen(2), P1(1)-c5_cen(1));
    c5_e = atan2(P2(2)-c5_cen(2), P2(1)-c5_cen(1));
    [c5_s, c5_e] = local_fixSweep(c5_s, c5_e, dir5);
    
    G.c5_cen = c5_cen;
    G.c5_r   = r;
    G.c5_s   = c5_s;
    G.c5_e   = c5_e;
    G.dir5   = dir5;
    
    G.p5_s = P1;
    G.p5_e = P2;
    
    % ===== Arc8 compute (connect p7_e -> l9_s) with r=800
    r = 800;
    P1 = G.p7_e;           % MUST equal Arc7 end
    P2 = G.p9_s;           % MUST equal Line9 start
    
    [d,Mm,u,perp,h,C1,C2] = local_twoCircleCenters(P1, P2, r);
    
    theta7e = M.c7_e;
    t_des = [ sin(theta7e) -cos(theta7e)];  % CW tangent at arc7 end
    
    [c8_cen, dir8] = local_chooseArcCenterDir(P1, C1, C2, t_des);
    c8_s = atan2(P1(2)-c8_cen(2), P1(1)-c8_cen(1));
    c8_e = atan2(P2(2)-c8_cen(2), P2(1)-c8_cen(1));
    [c8_s, c8_e] = local_fixSweep(c8_s, c8_e, dir8);
    
    G.c8_cen = c8_cen;
    G.c8_r   = r;
    G.c8_s   = c8_s;
    G.c8_e   = c8_e;
    G.dir8   = dir8;
    
    G.p8_s = P1;
    G.p8_e = P2;

end

function [d,Mm,u,perp,h,C1,C2] = local_twoCircleCenters(P1,P2,r)
    d  = norm(P2 - P1);
    Mm = 0.5*(P1 + P2);
    u  = (P2 - P1)/d;
    perp = [-u(2) u(1)];
    h  = sqrt(r^2 - (d/2)^2);
    C1 = Mm + h*perp;
    C2 = Mm - h*perp;
end

function [cen, dirStr] = local_chooseArcCenterDir(P1, C1, C2, t_des)
    thetaC1_s = atan2(P1(2)-C1(2), P1(1)-C1(1));
    thetaC2_s = atan2(P1(2)-C2(2), P1(1)-C2(1));
    tC1_ccw = [-sin(thetaC1_s) cos(thetaC1_s)];
    tC1_cw  = [ sin(thetaC1_s) -cos(thetaC1_s)];
    tC2_ccw = [-sin(thetaC2_s) cos(thetaC2_s)];
    tC2_cw  = [ sin(thetaC2_s) -cos(thetaC2_s)];
    
    score = [
        dot(tC1_ccw,t_des)
        dot(tC1_cw ,t_des)
        dot(tC2_ccw,t_des)
        dot(tC2_cw ,t_des)
    ];
    [~,k] = max(score);
    
    if k==1, cen=C1; dirStr="ccw";
    elseif k==2, cen=C1; dirStr="cw";
    elseif k==3, cen=C2; dirStr="ccw";
    else, cen=C2; dirStr="cw";
    end
end

function [th_s, th_e] = local_fixSweep(th_s, th_e, dirStr)
    if dirStr=="ccw" && th_e < th_s
        th_e = th_e + 2*pi;
    end
    if dirStr=="cw"  && th_e > th_s
        th_e = th_e - 2*pi;
    end
end

function v = local_unit(v)
    n = norm(v);
    if n < eps
        v = [NaN NaN];
    else
        v = v / n;
    end
end

function v = local_rot90(v)
    v = [-v(2) v(1)];
end

function dir = local_arcDir(th_s, th_e)
% Heuristic: if th_e >= th_s => ccw (+1) else cw (-1)
% For computed arcs (Arc5/Arc8) we override using dir5/dir8.
    if th_e >= th_s
        dir = +1;
    else
        dir = -1;
    end
end

function t = local_arcTangent(theta, dir)
% Unit tangent for circle at angle theta.
% ccw: [-sin cos], cw: [sin -cos]
    if dir >= 0
        t = [-sin(theta) cos(theta)];
    else
        t = [ sin(theta) -cos(theta)];
    end
end

function dir = local_dirFromString(dirStr)
    if dirStr=="ccw"
        dir = +1;
    else
        dir = -1;
    end
end
