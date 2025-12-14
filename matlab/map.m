%% ===== FIX continuity at T-junction: force Arc5 start=Arc4 end, Arc8 start=Arc7 end =====
% Keep your original definitions for: I1..I4, l1,l3,l6,l9, c2, c4, c7
% Replace ONLY Arc5 + Arc8 blocks by the code below.

t = tan(pi/8);

%% --- Endpoints you already have (for continuity targets)
% Arc4 end point (junction -> branch up)
p4_e = c4_cen + c4_r*[cos(c4_e) sin(c4_e)];      % end of arc4
% Arc7 end point (junction -> branch down)
p7_e = c7_cen + c7_r*[cos(c7_e) sin(c7_e)];      % end of arc7

% Line6 start (target end of Arc5)
p6_s = l6_s;
% Line9 start (target end of Arc8)
p9_s = l9_s;

%% ===== Arc 5 (connect p4_e -> p6_s) with r=800 =====
r = 800;
P1 = p4_e;           % MUST equal Arc4 end
P2 = p6_s;           % MUST equal Line6 start

d  = norm(P2 - P1);
M  = 0.5*(P1 + P2);
u  = (P2 - P1)/d;
perp = [-u(2) u(1)];
h  = sqrt(r^2 - (d/2)^2);

C1 = M + h*perp;      % two possible centers
C2 = M - h*perp;

% Desired tangent at Arc4 end (Arc4 goes from -pi/2 -> -pi/4, i.e., CCW)
theta4e = c4_e;
t_des = [-sin(theta4e) cos(theta4e)];  % CCW tangent at arc4 end

% Choose center + direction so Arc5 tangent at start matches t_des best
thetaC1_s = atan2(P1(2)-C1(2), P1(1)-C1(1));
thetaC2_s = atan2(P1(2)-C2(2), P1(1)-C2(1));
tC1_ccw = [-sin(thetaC1_s) cos(thetaC1_s)];
tC1_cw  = [ sin(thetaC1_s) -cos(thetaC1_s)];
tC2_ccw = [-sin(thetaC2_s) cos(thetaC2_s)];
tC2_cw  = [ sin(thetaC2_s) -cos(thetaC2_s)];

score = [
    dot(tC1_ccw,t_des)  % 1: C1 CCW
    dot(tC1_cw ,t_des)  % 2: C1 CW
    dot(tC2_ccw,t_des)  % 3: C2 CCW
    dot(tC2_cw ,t_des)  % 4: C2 CW
];
[~,k] = max(score);

if k==1, c5_cen=C1; dir5="ccw";
elseif k==2, c5_cen=C1; dir5="cw";
elseif k==3, c5_cen=C2; dir5="ccw";
else, c5_cen=C2; dir5="cw";
end

c5_r = r;
c5_s = atan2(P1(2)-c5_cen(2), P1(1)-c5_cen(1));
c5_e = atan2(P2(2)-c5_cen(2), P2(1)-c5_cen(1));

% Ensure sweep follows chosen direction (so linspace works)
if dir5=="ccw" && c5_e < c5_s, c5_e = c5_e + 2*pi; end
if dir5=="cw"  && c5_e > c5_s, c5_e = c5_e - 2*pi; end

%% ===== Arc 8 (connect p7_e -> p9_s) with r=800 =====
r = 800;
P1 = p7_e;           % MUST equal Arc7 end
P2 = p9_s;           % MUST equal Line9 start

d  = norm(P2 - P1);
M  = 0.5*(P1 + P2);
u  = (P2 - P1)/d;
perp = [-u(2) u(1)];
h  = sqrt(r^2 - (d/2)^2);

C1 = M + h*perp;
C2 = M - h*perp;

% Desired tangent at Arc7 end (Arc7 goes from pi/2 -> pi/4, i.e., CW)
theta7e = c7_e;
t_des = [ sin(theta7e) -cos(theta7e)];  % CW tangent at arc7 end

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

if k==1, c8_cen=C1; dir8="ccw";
elseif k==2, c8_cen=C1; dir8="cw";
elseif k==3, c8_cen=C2; dir8="ccw";
else, c8_cen=C2; dir8="cw";
end

c8_r = r;
c8_s = atan2(P1(2)-c8_cen(2), P1(1)-c8_cen(1));
c8_e = atan2(P2(2)-c8_cen(2), P2(1)-c8_cen(1));

if dir8=="ccw" && c8_e < c8_s, c8_e = c8_e + 2*pi; end
if dir8=="cw"  && c8_e > c8_s, c8_e = c8_e - 2*pi; end

%% ===== Plot (unchanged): just use your arc plot with (c5_s->c5_e) and (c8_s->c8_e) =====
% Now: Arc4->Arc5 is continuous at p4_e, and Arc7->Arc8 is continuous at p7_e.
% Also Arc5 ends exactly at l6_s, and Arc8 ends exactly at l9_s.

%% ===== Display (start/end) =====
disp('--- MAIN: 1 -> 2 -> 3 -> Junction ---');
disp(['1: ', mat2str(p1_s), ' -> ', mat2str(p1_e)]);
disp(['2: ', mat2str(p2_s), ' -> ', mat2str(p2_e)]);
disp(['3: ', mat2str(p3_s), ' -> ', mat2str(p3_e)]);
disp(['J: ', mat2str(pJ)]);

disp('--- BRANCH A: 4 -> 5 -> 6 ---');
disp(['4: ', mat2str(p4_s), ' -> ', mat2str(p4_e)]);
disp(['5: ', mat2str(p5_s), ' -> ', mat2str(p5_e)]);
disp(['6: ', mat2str(p6_s), ' -> ', mat2str(p6_e)]);

disp('--- BRANCH B: 7 -> 8 -> 9 ---');
disp(['7: ', mat2str(p7_s), ' -> ', mat2str(p7_e)]);
disp(['8: ', mat2str(p8_s), ' -> ', mat2str(p8_e)]);
disp(['9: ', mat2str(p9_s), ' -> ', mat2str(p9_e)]);

%% ===== Continuity checks (should be ~0 if map is consistent) =====
epsTol = 1e-6;
fprintf('\nContinuity errors (norm):\n');
fprintf('||p1_e - p2_s|| = %.6g\n', norm(p1_e - p2_s));
fprintf('||p2_e - p3_s|| = %.6g\n', norm(p2_e - p3_s));
fprintf('||p3_e - p4_s|| = %.6g\n', norm(p3_e - p4_s));
fprintf('||p3_e - p7_s|| = %.6g\n', norm(p3_e - p7_s));
fprintf('||p4_e - p5_s|| = %.6g\n', norm(p4_e - p5_s));
fprintf('||p5_e - p6_s|| = %.6g\n', norm(p5_e - p6_s));
fprintf('||p7_e - p8_s|| = %.6g\n', norm(p7_e - p8_s));
fprintf('||p8_e - p9_s|| = %.6g\n', norm(p8_e - p9_s));

%% ===== Plot map to verify geometry =====
figure; hold on; axis equal; grid on;

N = 200;
arcXY = @(cen,r,th1,th2,N) [cen(1)+r*cos(linspace(th1,th2,N)) ; cen(2)+r*sin(linspace(th1,th2,N))];

% Lines
plot([l1_s(1) l1_e(1)],[l1_s(2) l1_e(2)],'LineWidth',2);
plot([l3_s(1) l3_e(1)],[l3_s(2) l3_e(2)],'LineWidth',2);
plot([l6_s(1) l6_e(1)],[l6_s(2) l6_e(2)],'LineWidth',2);
plot([l9_s(1) l9_e(1)],[l9_s(2) l9_e(2)],'LineWidth',2);

% Arcs
xy2 = arcXY(c2_cen,c2_r,c2_s,c2_e,N); plot(xy2(1,:),xy2(2,:),'LineWidth',2);
xy4 = arcXY(c4_cen,c4_r,c4_s,c4_e,N); plot(xy4(1,:),xy4(2,:),'LineWidth',2);
xy5 = arcXY(c5_cen,c5_r,c5_s,c5_e,N); plot(xy5(1,:),xy5(2,:),'LineWidth',2);
xy7 = arcXY(c7_cen,c7_r,c7_s,c7_e,N); plot(xy7(1,:),xy7(2,:),'LineWidth',2);
xy8 = arcXY(c8_cen,c8_r,c8_s,c8_e,N); plot(xy8(1,:),xy8(2,:),'LineWidth',2);

% Mark key points (segment endpoints + junction)
P = [p1_s;p1_e;p2_s;p2_e;p3_s;p3_e;p4_e;p5_s;p5_e;p6_e;p7_e;p8_s;p8_e;p9_e];
plot(P(:,1),P(:,2),'o','MarkerSize',6,'LineWidth',1.5);
text(pJ(1),pJ(2),'  J','FontSize',10,'FontWeight','bold');

title('Map check: lines + arcs');
xlabel('x (mm)'); ylabel('y (mm)');
%% ===== Add named points on plot (append after your current plotting code) =====

% ---- (Re)compute any missing endpoints used in labels
p1_s = l1_s;  p1_e = l1_e;

p2_s = c2_cen + c2_r*[cos(c2_s) sin(c2_s)];
p2_e = c2_cen + c2_r*[cos(c2_e) sin(c2_e)];

p3_s = l3_s;  p3_e = l3_e;
pJ   = p3_e;

p4_s = pJ;
p4_e = c4_cen + c4_r*[cos(c4_e) sin(c4_e)];

p5_s = p4_e;
p5_e = c5_cen + c5_r*[cos(c5_e) sin(c5_e)];

p6_s = l6_s;  p6_e = l6_e;

p7_s = pJ;
p7_e = c7_cen + c7_r*[cos(c7_e) sin(c7_e)];

p8_s = p7_e;
p8_e = c8_cen + c8_r*[cos(c8_e) sin(c8_e)];

p9_s = l9_s;  p9_e = l9_e;

% ---- Point list + names
P = [
    p1_s
    p1_e
    p2_e
    pJ
    p4_e
    p5_e
    p6_e
    p7_e
    p8_e
    p9_e
];

names = {
    'P1s (L1 start)'
    'P1e (L1 end)'
    'P2e (Arc2 end)'
    'J (T-junction)'
    'P4e (Arc4 end)'
    'P5e (Arc5 end)'
    'P6e (Line6 end)'
    'P7e (Arc7 end)'
    'P8e (Arc8 end)'
    'P9e (Line9 end)'
};

% ---- Plot points + labels (offset to avoid overlap)
plot(P(:,1), P(:,2), 's', 'MarkerSize', 6, 'LineWidth', 1.5);

dx = 20;  % mm label offset in x
dy = 20;  % mm label offset in y
for i = 1:size(P,1)
    text(P(i,1)+dx, P(i,2)+dy, names{i}, 'FontSize', 9, 'Interpreter', 'none');
end

% ---- Optional: annotate arc centers too
C = [c2_cen; c4_cen; c5_cen; c7_cen; c8_cen];
cn = {'C2','C4','C5','C7','C8'};
plot(C(:,1), C(:,2), 'x', 'MarkerSize', 8, 'LineWidth', 1.5);
for i = 1:size(C,1)
    text(C(i,1)+dx, C(i,2)+dy, cn{i}, 'FontSize', 9, 'Interpreter', 'none');
end

legend({'Lines/Arcs','Key points','Centers'}, 'Location','bestoutside');
