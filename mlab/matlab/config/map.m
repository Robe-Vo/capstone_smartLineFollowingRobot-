% Draw map and its 2-side offsets (road width = 130)

clear; clc; close all;

d_off = 130;          % offset distance
nLine = 200;
nArc  = 200;

% Imagine points
I1 = [-2500 500];
I2 = [-2000 1000];
I3 = [-1500 1500];
I4 = [-1500 500];

% Line 1
l1_s = [0 0];
l1_e = [-2500 0];

% Arc 2
c2_cen = [-2500 500];
c2_r   = 500;
c2_s   = 3*pi/2;
c2_e   = pi/2;

% Line 3
l3_s = [-2500 1000];
l3_e = I2 + [-800*tan(pi/8) 0];

% Arc 4
c4_cen = I2 + [-800*tan(pi/8) 800];
c4_r   = 800;
c4_s   = -pi/2;
c4_e   = -pi/4;

% Arc 5 (continuous with arc 4 and line 6)
c5_cen = I3 + [800*tan(pi/8) -800];
c5_r   = 800;

% Line 6
l6_s = I3 + [800*tan(pi/8) 0];
l6_e = [0 1500];

% Arc 7 (T-junction at l3_e)
c7_cen = I2 + [-800*tan(pi/8) -800];
c7_r   = 800;
c7_s   = pi/2;
c7_e   = pi/4;

% Arc 8 (continuous with arc 7 and line 9)
c8_cen = I4 + [800*tan(pi/8) 800];
c8_r   = 800;

% Line 9
l9_s = I4 + [800*tan(pi/8) 0];
l9_e = [0 500];

%% ========= Build main path segments =========

% Line 1
[l1_P, l1_T] = line_pts_tan(l1_s, l1_e, nLine);
[l1_left, l1_right] = offset_from_PT(l1_P, l1_T, d_off);

% Arc 2
[a2_P, a2_T] = arc_pts_tan(c2_cen, c2_r, c2_s, c2_e, nArc);
[a2_left, a2_right] = offset_from_PT(a2_P, a2_T, d_off);

% Line 3
[l3_P, l3_T] = line_pts_tan(l3_s, l3_e, nLine);
[l3_left, l3_right] = offset_from_PT(l3_P, l3_T, d_off);

% Arc 4
[a4_P, a4_T] = arc_pts_tan(c4_cen, c4_r, c4_s, c4_e, nArc);
[a4_left, a4_right] = offset_from_PT(a4_P, a4_T, d_off);

% Arc 5: start at end of arc 4, end at start of line 6
joint_45 = a4_P(end,:);
[a5_P, a5_T] = arc_pts_two_points_tan(c5_cen, c5_r, joint_45, l6_s, nArc);
[a5_left, a5_right] = offset_from_PT(a5_P, a5_T, d_off);

% Line 6
[l6_P, l6_T] = line_pts_tan(l6_s, l6_e, nLine);
[l6_left, l6_right] = offset_from_PT(l6_P, l6_T, d_off);

%% ========= Build T-branch segments =========

% Arc 7: from l3_e to angle near c7_e
[a7_P, a7_T] = arc_pts_two_points_tan(c7_cen, c7_r, l3_e, ...
                                      [c7_cen(1)+c7_r*cos(c7_e), ...
                                       c7_cen(2)+c7_r*sin(c7_e)], nArc);
[a7_left, a7_right] = offset_from_PT(a7_P, a7_T, d_off);

% Arc 8: from end of arc 7 to start of line 9
joint_78 = a7_P(end,:);
[a8_P, a8_T] = arc_pts_two_points_tan(c8_cen, c8_r, joint_78, l9_s, nArc);
[a8_left, a8_right] = offset_from_PT(a8_P, a8_T, d_off);

% Line 9
[l9_P, l9_T] = line_pts_tan(l9_s, l9_e, nLine);
[l9_left, l9_right] = offset_from_PT(l9_P, l9_T, d_off);

%% ========= Plot original map =========
figure;
hold on; axis equal; grid on;
title('Original map');
xlabel('X'); ylabel('Y');

plot(l1_P(:,1), l1_P(:,2), 'k.-', 'LineWidth', 2);
plot(a2_P(:,1), a2_P(:,2), 'k.-', 'LineWidth', 2);
plot(l3_P(:,1), l3_P(:,2), 'k.-', 'LineWidth', 2);
plot(a4_P(:,1), a4_P(:,2), 'k.-', 'LineWidth', 2);
plot(a5_P(:,1), a5_P(:,2), 'k.-', 'LineWidth', 2);
plot(l6_P(:,1), l6_P(:,2), 'k.-', 'LineWidth', 2);

plot(a7_P(:,1), a7_P(:,2), 'k.-', 'LineWidth', 2);
plot(a8_P(:,1), a8_P(:,2), 'k.-', 'LineWidth', 2);
plot(l9_P(:,1), l9_P(:,2), 'k.-', 'LineWidth', 2);

plot(I1(1),I1(2),'ko'); plot(I2(1),I2(2),'ko');
plot(I3(1),I3(2),'ko'); plot(I4(1),I4(2),'ko');

%% ========= Plot offset roads =========
figure;
hold on; axis equal; grid on;
title('Offset roads');
xlabel('X'); ylabel('Y');

% main
plot(l1_left(:,1), l1_left(:,2), 'k.-', 'LineWidth', 2);
plot(l1_right(:,1),l1_right(:,2),'k.-', 'LineWidth', 2);

plot(a2_left(:,1), a2_left(:,2), 'k.-', 'LineWidth', 2);
plot(a2_right(:,1),a2_right(:,2),'k.-', 'LineWidth', 2);

plot(l3_left(:,1), l3_left(:,2), 'k.-', 'LineWidth', 2);
plot(l3_right(:,1),l3_right(:,2),'k.-', 'LineWidth', 2);

plot(a4_left(:,1), a4_left(:,2), 'k.-', 'LineWidth', 2);
plot(a4_right(:,1),a4_right(:,2),'k.-', 'LineWidth', 2);

plot(a5_left(:,1), a5_left(:,2), 'k.-', 'LineWidth', 2);
plot(a5_right(:,1),a5_right(:,2),'k.-', 'LineWidth', 2);

plot(l6_left(:,1), l6_left(:,2), 'k.-', 'LineWidth', 2);
plot(l6_right(:,1),l6_right(:,2),'k.-', 'LineWidth', 2);

% branch
plot(a7_left(:,1), a7_left(:,2), 'k.-', 'LineWidth', 2);
plot(a7_right(:,1),a7_right(:,2),'k.-', 'LineWidth', 2);

plot(a8_left(:,1), a8_left(:,2), 'k.-', 'LineWidth', 2);
plot(a8_right(:,1),a8_right(:,2),'k.-', 'LineWidth', 2);

plot(l9_left(:,1), l9_left(:,2), 'k.-', 'LineWidth', 2);
plot(l9_right(:,1),l9_right(:,2),'k.-', 'LineWidth', 2);

%% ========= Support functions =========
function [P,T] = line_pts_tan(Ps, Pe, n)
t = linspace(0,1,n).';
P = [Ps(1)+(Pe(1)-Ps(1))*t, ...
     Ps(2)+(Pe(2)-Ps(2))*t];
v = Pe - Ps;
v = v / norm(v);
T = repmat(v, n, 1);
end

function [P,T] = arc_pts_tan(c, r, a1, a2, n)
th = linspace(a1,a2,n).';
x = c(1) + r*cos(th);
y = c(2) + r*sin(th);
P = [x y];
dx = -r*sin(th);
dy =  r*cos(th);
T  = [dx dy];
T  = T ./ vecnorm(T,2,2);
end

function [P,T] = arc_pts_two_points_tan(c, r, P1, P2, n)
a1 = atan2(P1(2)-c(2), P1(1)-c(1));
a2 = atan2(P2(2)-c(2), P2(1)-c(1));
da = a2 - a1;
if da >  pi
    a2 = a2 - 2*pi;
elseif da < -pi
    a2 = a2 + 2*pi;
end
[P,T] = arc_pts_tan(c, r, a1, a2, n);
end

function [L,R] = offset_from_PT(P,T,d)
N = [-T(:,2), T(:,1)];
L = P + d*N;
R = P - d*N;
end
