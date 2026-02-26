% Chapter3_Lesson3.m
% Autonomous Mobile Robots — Chapter 3 Lesson 3
% Feasible Path Families for Car-Like Robots (Dubins)
%
% Requirements:
%   - For toolbox-based approach: Robotics System Toolbox (dubinsConnection)
%   - For from-scratch approach: no extra toolboxes
%
% Usage:
%   Chapter3_Lesson3

function Chapter3_Lesson3()
clc; close all;

q0 = [0.0, 0.0, deg2rad(10.0)];      % [x,y,theta]
q1 = [8.0, 4.0, deg2rad(110.0)];
Rmin = 2.0;

fprintf('--- Dubins path demo ---\n');

if exist('dubinsConnection','class') == 8
    fprintf('Using Robotics System Toolbox: dubinsConnection\n');
    dubConn = dubinsConnection('MinTurningRadius', Rmin);
    [pathSegObj, pathLen] = connect(dubConn, q0, q1);
    fprintf('Toolbox length = %.4f\n', pathLen);

    % Interpolate
    s = linspace(0, pathLen, 300);
    states = interpolate(pathSegObj{1}, s);
    x = states(:,1); y = states(:,2); th = states(:,3);
else
    fprintf('Using from-scratch Dubins solver (no toolbox)\n');
    path = dubinsShortest(q0, q1, Rmin);
    fprintf('Type = %s, length = %.4f\n', path.type, path.length);

    pts = sampleDubins(q0, path, 0.05);
    x = pts(:,1); y = pts(:,2); th = pts(:,3);
end

figure; plot(x,y,'LineWidth',1.5); hold on;
plot(q0(1),q0(2),'ko','MarkerFaceColor','k');
plot(q1(1),q1(2),'ks','MarkerFaceColor','k');
axis equal; grid on;
title('Dubins feasible path (bounded curvature)');
xlabel('x'); ylabel('y');

% Optional: build a simple Simulink model that integrates kinematics
% buildSimulinkDubinsModel();  % Uncomment if you want auto-generated model

end

% -------------------- From-scratch implementation --------------------

function a = mod2pi(a)
a = mod(a, 2*pi);
if a < 0
    a = a + 2*pi;
end
end

function d = angdiff(a,b)
d = mod(a-b+pi, 2*pi);
d = d - pi;
end

function pose = segUnit(pose, st, sl)
x = pose(1); y = pose(2); th = pose(3);
switch st
    case 'S'
        x = x + sl*cos(th);
        y = y + sl*sin(th);
    case 'L'
        x = x + (sin(th+sl) - sin(th));
        y = y + (-cos(th+sl) + cos(th));
        th = mod2pi(th + sl);
    case 'R'
        x = x + (-sin(th-sl) + sin(th));
        y = y + (cos(th-sl) - cos(th));
        th = mod2pi(th - sl);
    otherwise
        error('Unknown segment type');
end
pose = [x;y;th];
end

function err = endpointError(alpha, d, beta, seg, prm)
pose = [0;0;alpha];
pose = segUnit(pose, seg(1), prm(1));
pose = segUnit(pose, seg(2), prm(2));
pose = segUnit(pose, seg(3), prm(3));
err = hypot(pose(1)-d, pose(2)) + abs(angdiff(pose(3), beta));
end

function prm = cand_LSL(alpha,beta,d)
sa = sin(alpha); sb = sin(beta);
ca = cos(alpha); cb = cos(beta);
cab = cos(alpha-beta);
tmp0 = d + sa - sb;
p2 = 2 + d^2 - 2*cab + 2*d*(sa - sb);
if p2 < 0, prm = []; return; end
p = sqrt(p2);
tmp1 = atan2(cb - ca, tmp0);
t = mod2pi(-alpha + tmp1);
q = mod2pi(beta - tmp1);
prm = [t;p;q];
end

function prm = cand_RSR(alpha,beta,d)
sa = sin(alpha); sb = sin(beta);
ca = cos(alpha); cb = cos(beta);
cab = cos(alpha-beta);
tmp0 = d - sa + sb;
p2 = 2 + d^2 - 2*cab + 2*d*(-sa + sb);
if p2 < 0, prm = []; return; end
p = sqrt(p2);
tmp1 = atan2(ca - cb, tmp0);
t = mod2pi(alpha - tmp1);
q = mod2pi(-beta + tmp1);
prm = [t;p;q];
end

function prm = cand_LSR(alpha,beta,d)
sa = sin(alpha); sb = sin(beta);
ca = cos(alpha); cb = cos(beta);
cab = cos(alpha-beta);
p2 = -2 + d^2 + 2*cab + 2*d*(sa + sb);
if p2 < 0, prm = []; return; end
p = sqrt(p2);
tmp0 = atan2(-ca - cb, d + sa + sb);
tmp1 = atan2(-2, p);
t = mod2pi(-alpha + tmp0 - tmp1);
q = mod2pi(-beta + tmp0 - tmp1);
prm = [t;p;q];
end

function prm = cand_RSL(alpha,beta,d)
sa = sin(alpha); sb = sin(beta);
ca = cos(alpha); cb = cos(beta);
cab = cos(alpha-beta);
p2 = -2 + d^2 + 2*cab - 2*d*(sa + sb);
if p2 < 0, prm = []; return; end
p = sqrt(p2);
tmp0 = atan2(ca + cb, d - sa - sb);
tmp1 = atan2(2, p);
t = mod2pi(alpha - tmp0 + tmp1);
q = mod2pi(beta - tmp0 + tmp1);
prm = [t;p;q];
end

function prm = cand_RLR(alpha,beta,d)
sa = sin(alpha); sb = sin(beta);
ca = cos(alpha); cb = cos(beta);
cab = cos(alpha-beta);
tmp0 = (6 - d^2 + 2*cab + 2*d*(sa - sb))/8;
if abs(tmp0) > 1, prm = []; return; end
p = mod2pi(2*pi - acos(tmp0));
tmp1 = atan2(ca - cb, d - sa + sb);
t = mod2pi(alpha - tmp1 + p/2);
q = mod2pi(alpha - beta - t + p);
prm = [t;p;q];
end

function prm = cand_LRL(alpha,beta,d)
% symmetry: LRL(alpha,beta,d) = RLR(-alpha,-beta,d)
prm = cand_RLR(mod2pi(-alpha), mod2pi(-beta), d);
end

function path = dubinsShortest(q0,q1,Rmin)
dx = q1(1)-q0(1);
dy = q1(2)-q0(2);
c0 = cos(q0(3)); s0 = sin(q0(3));

x = (c0*dx + s0*dy)/Rmin;
y = (-s0*dx + c0*dy)/Rmin;
phi = mod2pi(q1(3) - q0(3));

d = hypot(x,y);
theta = 0;
if d > 0
    theta = atan2(y,x);
end
alpha = mod2pi(-theta);
beta  = mod2pi(phi - theta);

types = {'LSL','RSR','LSR','RSL','RLR','LRL'};
segs  = {'LSL','RSR','LSR','RSL','RLR','LRL'};
cands = {@cand_LSL,@cand_RSR,@cand_LSR,@cand_RSL,@cand_RLR,@cand_LRL};

bestLen = inf;
best = struct();

for i=1:numel(types)
    prm = cands{i}(alpha,beta,d);
    if isempty(prm), continue; end
    err = endpointError(alpha,d,beta,segs{i},prm);
    if err > 1e-6, continue; end
    L = Rmin * sum(prm);
    if L < bestLen
        bestLen = L;
        best.type = types{i};
        best.seg  = segs{i};
        best.prm  = prm;
        best.Rmin = Rmin;
        best.length = L;
    end
end

if isinf(bestLen)
    error('No feasible Dubins path found');
end
path = best;
end

function pts = sampleDubins(q0, path, step)
t = path.prm(1); p = path.prm(2); q = path.prm(3);
segTypes = path.seg;
lens = [t,p,q];

cur = [q0(1);q0(2);mod2pi(q0(3))];
pts = cur.';
for i=1:3
    st = segTypes(i);
    sl = lens(i);
    if st == 'S'
        L = sl * path.Rmin;
        n = max(1, ceil(L/step));
        ds = L/n;
        for k=1:n
            cur(1) = cur(1) + ds*cos(cur(3));
            cur(2) = cur(2) + ds*sin(cur(3));
            pts(end+1,:) = cur.'; %#ok<AGROW>
        end
    else
        a = sl;
        arc = a * path.Rmin;
        n = max(1, ceil(arc/step));
        da = a/n;
        for k=1:n
            if st == 'L'
                cur(1) = cur(1) + path.Rmin*(sin(cur(3)+da)-sin(cur(3)));
                cur(2) = cur(2) + path.Rmin*(-cos(cur(3)+da)+cos(cur(3)));
                cur(3) = mod2pi(cur(3)+da);
            else
                cur(1) = cur(1) + path.Rmin*(-sin(cur(3)-da)+sin(cur(3)));
                cur(2) = cur(2) + path.Rmin*(cos(cur(3)-da)-cos(cur(3)));
                cur(3) = mod2pi(cur(3)-da);
            end
            pts(end+1,:) = cur.'; %#ok<AGROW>
        end
    end
end
end

% -------------------- Simulink (optional) --------------------
function buildSimulinkDubinsModel()
% This helper creates a minimal Simulink model that integrates:
%   xdot = v cos(theta), ydot = v sin(theta), thetadot = v * kappa
% for a piecewise-constant curvature profile (e.g., Dubins primitives).
%
% It is provided as an engineering template and is not required to run the main demo.

model = 'Chapter3_Lesson3_DubinsSim';
if bdIsLoaded(model), close_system(model,0); end
new_system(model); open_system(model);

add_block('simulink/Sources/Constant',[model '/v'],'Value','1.0','Position',[30 30 80 60]);
add_block('simulink/Sources/Constant',[model '/kappa'],'Value','0.2','Position',[30 90 80 120]);

add_block('simulink/Math Operations/Trigonometric Function',[model '/cos'],'Operator','cos','Position',[140 20 170 50]);
add_block('simulink/Math Operations/Trigonometric Function',[model '/sin'],'Operator','sin','Position',[140 80 170 110]);

add_block('simulink/Math Operations/Product',[model '/vx'],'Position',[220 30 250 60]);
add_block('simulink/Math Operations/Product',[model '/vy'],'Position',[220 90 250 120]);

add_block('simulink/Continuous/Integrator',[model '/Int_x'],'Position',[310 30 340 60]);
add_block('simulink/Continuous/Integrator',[model '/Int_y'],'Position',[310 90 340 120]);
add_block('simulink/Continuous/Integrator',[model '/Int_theta'],'Position',[310 150 340 180]);

add_block('simulink/Math Operations/Product',[model '/v_kappa'],'Position',[220 150 250 180]);

% theta feedback to trig blocks
add_line(model,'Int_theta/1','cos/1');
add_line(model,'Int_theta/1','sin/1');

% v and trig to vx, vy
add_line(model,'v/1','vx/1'); add_line(model,'cos/1','vx/2');
add_line(model,'v/1','vy/1'); add_line(model,'sin/1','vy/2');

% integrate
add_line(model,'vx/1','Int_x/1');
add_line(model,'vy/1','Int_y/1');

% theta dot
add_line(model,'v/1','v_kappa/1');
add_line(model,'kappa/1','v_kappa/2');
add_line(model,'v_kappa/1','Int_theta/1');

save_system(model);
fprintf('Created Simulink model: %s.slx\n', model);
end
