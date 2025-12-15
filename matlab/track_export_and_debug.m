function track_export_and_debug(doPlot)
% ============================================================
% TRACK_EXPORT_AND_DEBUG
% - Xuất start / end / center / R cho từng đoạn
% - Đảm bảo liên tục tại T-junction (4->5, 7->8)
% - Plot debug khi doPlot = true
%
% OUTPUT: in workspace + command window
% ============================================================
% doPlot : logical (true/false)

if nargin==0
    doPlot = true;
end

%% ================== CONSTANTS ==================
t = tan(pi/8);
N = 200;                 % plot resolution
arcXY = @(c,r,a,b) [c(1)+r*cos(linspace(a,b,N));
                    c(2)+r*sin(linspace(a,b,N))];

%% ================== IMAGINE POINTS ==================
I1 = [-2500 500];
I2 = [-2000 1000];
I3 = [-1500 1500];
I4 = [-1500 500];

%% ================== SEGMENT DEFINITIONS ==================
% ---------- Line 1
seg(1).type = 'line';
seg(1).name = 'L1';
seg(1).s = [0 0];
seg(1).e = [-2500 0];
seg(1).cen = [];
seg(1).R = NaN;

% ---------- Arc 2
seg(2).type = 'arc';
seg(2).name = 'A2';
seg(2).cen = [-2500 500];
seg(2).R = 500;
seg(2).ang = [3*pi/2 pi/2];
seg(2).s = seg(2).cen + seg(2).R*[cos(seg(2).ang(1)) sin(seg(2).ang(1))];
seg(2).e = seg(2).cen + seg(2).R*[cos(seg(2).ang(2)) sin(seg(2).ang(2))];

% ---------- Line 3
seg(3).type = 'line';
seg(3).name = 'L3';
seg(3).s = seg(2).e;
seg(3).e = I2 + [-800*t 0];
seg(3).cen = [];
seg(3).R = NaN;

J = seg(3).e;   % T-junction

% ---------- Arc 4 (upper)
seg(4).type = 'arc';
seg(4).name = 'A4';
seg(4).cen = I2 + [-800*t 800];
seg(4).R = 800;
seg(4).ang = [-pi/2 -pi/4];
seg(4).s = J;
seg(4).e = seg(4).cen + seg(4).R*[cos(seg(4).ang(2)) sin(seg(4).ang(2))];

% ---------- Arc 7 (lower)
seg(7).type = 'arc';
seg(7).name = 'A7';
seg(7).cen = I2 + [-800*t -800];
seg(7).R = 800;
seg(7).ang = [pi/2 pi/4];
seg(7).s = J;
seg(7).e = seg(7).cen + seg(7).R*[cos(seg(7).ang(2)) sin(seg(7).ang(2))];

% ---------- Line 6 (upper end)
seg(6).type = 'line';
seg(6).name = 'L6';
seg(6).s = I3 + [800*t 0];
seg(6).e = [0 1500];
seg(6).cen = [];
seg(6).R = NaN;

% ---------- Line 9 (lower end)
seg(9).type = 'line';
seg(9).name = 'L9';
seg(9).s = I4 + [800*t 0];
seg(9).e = [0 500];
seg(9).cen = [];
seg(9).R = NaN;

%% ================== AUTO ARC 5 (A4 -> L6) ==================
[seg(5)] = connectArc(seg(4).e, seg(6).s, seg(4), 800, 'A5');

%% ================== AUTO ARC 8 (A7 -> L9) ==================
[seg(8)] = connectArc(seg(7).e, seg(9).s, seg(7), 800, 'A8');

%% ================== EXPORT TABLE ==================
fprintf('\n===== TRACK SEGMENTS =====\n');
for k = [1 2 3 4 5 6 7 8 9]
    fprintf('%s (%s)\n', seg(k).name, seg(k).type);
    fprintf('  start : [%.2f %.2f]\n', seg(k).s);
    fprintf('  end   : [%.2f %.2f]\n', seg(k).e);
    if strcmp(seg(k).type,'arc')
        fprintf('  center: [%.2f %.2f]\n', seg(k).cen);
        fprintf('  R     : %.2f\n', seg(k).R);
    end
end

%% ================== DEBUG PLOT ==================
if doPlot
    figure; hold on; axis equal; grid on;

    for k = [1 2 3 4 5 6 7 8 9]
        if strcmp(seg(k).type,'line')
            plot([seg(k).s(1) seg(k).e(1)], ...
                 [seg(k).s(2) seg(k).e(2)], 'LineWidth',2);
        else
            xy = arcXY(seg(k).cen, seg(k).R, ...
                       seg(k).ang(1), seg(k).ang(2));
            plot(xy(1,:), xy(2,:), 'LineWidth',2);
            plot(seg(k).cen(1), seg(k).cen(2),'x');
        end
        plot(seg(k).s(1), seg(k).s(2),'o');
        plot(seg(k).e(1), seg(k).e(2),'o');
        text(seg(k).s(1)+20, seg(k).s(2)+20, [seg(k).name,'_s']);
        text(seg(k).e(1)+20, seg(k).e(2)+20, [seg(k).name,'_e']);
    end

    plot(J(1),J(2),'ks','MarkerSize',8,'LineWidth',2);
    text(J(1)+20,J(2)+20,'J');

    title('Track debug plot');
    xlabel('x (mm)'); ylabel('y (mm)');
end

end

%% ============================================================
% ===== FIX: Arc8 wrong direction -> fix connectArc() to inherit prev arc direction =====
% Replace ONLY the connectArc() function in your file by the version below.

function seg = connectArc(P1, P2, prevArc, R, name)
% Build arc from P1 -> P2, matching tangent at P1.
% Direction is inherited from prevArc sweep:
%   prevArc.ang(2) > prevArc.ang(1)  => CCW
%   prevArc.ang(2) < prevArc.ang(1)  => CW

d = norm(P2-P1);
M = 0.5*(P1+P2);
u = (P2-P1)/d;
perp = [-u(2) u(1)];
h = sqrt(R^2 - (d/2)^2);

C = [M + h*perp;
     M - h*perp];

% --- Desired tangent at P1 = tangent at end of prevArc (direction-aware)
thetaPrev = prevArc.ang(2);
isPrevCCW = (prevArc.ang(2) > prevArc.ang(1));

if isPrevCCW
    t_des = [-sin(thetaPrev)  cos(thetaPrev)];   % CCW tangent
else
    t_des = [ sin(thetaPrev) -cos(thetaPrev)];   % CW tangent
end

% --- Choose center and direction (cw/ccw) maximizing tangent alignment at P1
best = -inf;
cen  = [NaN NaN];
dir  = 1;   % +1=ccw, -1=cw

for i = 1:2
    th = atan2(P1(2)-C(i,2), P1(1)-C(i,1));
    tccw = [-sin(th)  cos(th)];
    tcw  = [ sin(th) -cos(th)];

    s1 = dot(tccw, t_des);
    if s1 > best
        best = s1; cen = C(i,:); dir = +1;
    end

    s2 = dot(tcw, t_des);
    if s2 > best
        best = s2; cen = C(i,:); dir = -1;
    end
end

% --- Angles for the chosen arc
a1 = atan2(P1(2)-cen(2), P1(1)-cen(1));
a2 = atan2(P2(2)-cen(2), P2(1)-cen(1));

% Normalize sweep to follow chosen direction (so linspace works)
if dir==+1 && a2 < a1, a2 = a2 + 2*pi; end
if dir==-1 && a2 > a1, a2 = a2 - 2*pi; end

seg.type = 'arc';
seg.name = name;
seg.s    = P1;
seg.e    = P2;
seg.cen  = cen;
seg.R    = R;
seg.ang  = [a1 a2];
end

