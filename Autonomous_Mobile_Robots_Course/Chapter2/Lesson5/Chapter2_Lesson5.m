% Chapter2_Lesson5.m
%{
Chapter 2 — Wheeled Locomotion Kinematics (Mobile-Specific)
Lesson 5 — Kinematic Calibration of Wheel Parameters

This script calibrates p = [r_L, r_R, b] for a differential-drive robot
from relative body-frame increments using a toolbox-free damped Gauss-Newton method.

Input CSV format (header optional):
  dphi_L, dphi_R, dx_gt, dy_gt, dtheta_gt
Units: radians, meters, radians

Optional Simulink note:
At the end of this file there is a function that programmatically builds a simple
Simulink model that computes (dx,dy,dtheta) from (dphi_L,dphi_R) and parameters p.

Run:
  Chapter2_Lesson5  % uses synthetic data if no CSV is given

Or:
  Chapter2_Lesson5('data.csv')
%}

function Chapter2_Lesson5(csvPath)
if nargin < 1
    csvPath = '';
end

if isempty(csvPath)
    data = simulateDataset(200, [0.050; 0.051; 0.300], 0.002);
else
    data = loadCSV(csvPath);
end

p0 = [0.05; 0.05; 0.30];
[pHat, info] = calibrateGN(data, p0);

disp('Initial p0  = [rL, rR, b]'); disp(p0.');
disp('Estimated p = [rL, rR, b]'); disp(pHat.');
disp(info);

[r, ~] = residualAndJacobian(pHat, data);
R = reshape(r, 3, []).';
rms = sqrt(mean(R.^2, 1));
fprintf('RMS residuals [dx dy dtheta] = %.6e %.6e %.6e\n', rms(1), rms(2), rms(3));

% Uncomment to build the Simulink model programmatically:
% buildSimulinkModel();

end

function data = loadCSV(path)
T = readtable(path);
% Expect columns: dphi_L, dphi_R, dx_gt, dy_gt, dtheta_gt
data.dphiL = T{:,1};
data.dphiR = T{:,2};
data.dx    = T{:,3};
data.dy    = T{:,4};
data.dth   = T{:,5};
end

function data = simulateDataset(N, pTrue, noiseStd)
rng(7);
dphiL = 0.4 * randn(N,1);
dphiR = 0.4 * randn(N,1);
u = rand(N,1);
% straight-ish segments
idx = (u < 0.35);
dphiR(idx) = dphiL(idx) + 0.03 * randn(sum(idx),1);
% in-place-ish
idx = (u >= 0.35) & (u < 0.55);
dphiR(idx) = -dphiL(idx) + 0.03 * randn(sum(idx),1);

[dx, dy, dth] = predictIncrement(dphiL, dphiR, pTrue);

data.dphiL = dphiL;
data.dphiR = dphiR;
data.dx = dx + noiseStd * randn(N,1);
data.dy = dy + noiseStd * randn(N,1);
data.dth = dth + 0.5*noiseStd * randn(N,1);
end

function a = wrapToPi(a)
a = mod(a + pi, 2*pi) - pi;
end

function y = sinc1(x)
y = zeros(size(x));
small = abs(x) < 1e-6;
xs = x(small);
y(small) = 1 - (xs.^2)/6 + (xs.^4)/120;
xb = x(~small);
y(~small) = sin(xb) ./ xb;
end

function y = cosc1(x)
y = zeros(size(x));
small = abs(x) < 1e-6;
xs = x(small);
y(small) = xs/2 - (xs.^3)/24 + (xs.^5)/720;
xb = x(~small);
y(~small) = (1 - cos(xb)) ./ xb;
end

function y = dsinc1(x)
y = zeros(size(x));
small = abs(x) < 1e-5;
xs = x(small);
y(small) = -(xs)/3 + (xs.^3)/30 - (xs.^5)/840;
xb = x(~small);
y(~small) = (xb.*cos(xb) - sin(xb)) ./ (xb.^2);
end

function y = dcosc1(x)
y = zeros(size(x));
small = abs(x) < 1e-5;
xs = x(small);
y(small) = 0.5 - (xs.^2)/8 + (xs.^4)/144;
xb = x(~small);
y(~small) = (xb.*sin(xb) - (1 - cos(xb))) ./ (xb.^2);
end

function [dx, dy, dth] = predictIncrement(dphiL, dphiR, p)
rL = p(1); rR = p(2); b = p(3);
sL = rL .* dphiL;
sR = rR .* dphiR;
ds = 0.5 * (sR + sL);
dth = (sR - sL) ./ b;

dx = ds .* sinc1(dth);
dy = ds .* cosc1(dth);
end

function [r, J] = residualAndJacobian(p, data)
rL = p(1); rR = p(2); b = p(3);
dphiL = data.dphiL; dphiR = data.dphiR;

sL = rL .* dphiL;
sR = rR .* dphiR;
A = sR + sL;
B = sR - sL;

ds = 0.5 * A;
dth = B ./ b;

f  = sinc1(dth);
g  = cosc1(dth);
fp = dsinc1(dth);
gp = dcosc1(dth);

dx = ds .* f;
dy = ds .* g;

ex = dx - data.dx;
ey = dy - data.dy;
eth = wrapToPi(dth - data.dth);

N = numel(dphiL);
r = zeros(3*N,1);
r(1:3:end) = ex;
r(2:3:end) = ey;
r(3:3:end) = eth;

dds_drL = 0.5 .* dphiL;
dds_drR = 0.5 .* dphiR;

ddth_drL = -(dphiL) ./ b;
ddth_drR = (dphiR) ./ b;
ddth_db  = -(B) ./ (b.^2);

ddx_drL = dds_drL .* f + ds .* fp .* ddth_drL;
ddx_drR = dds_drR .* f + ds .* fp .* ddth_drR;
ddx_db  = 0 .* f       + ds .* fp .* ddth_db;

ddy_drL = dds_drL .* g + ds .* gp .* ddth_drL;
ddy_drR = dds_drR .* g + ds .* gp .* ddth_drR;
ddy_db  = 0 .* g       + ds .* gp .* ddth_db;

J = zeros(3*N, 3);
J(1:3:end, 1) = ddx_drL;
J(1:3:end, 2) = ddx_drR;
J(1:3:end, 3) = ddx_db;

J(2:3:end, 1) = ddy_drL;
J(2:3:end, 2) = ddy_drR;
J(2:3:end, 3) = ddy_db;

J(3:3:end, 1) = ddth_drL;
J(3:3:end, 2) = ddth_drR;
J(3:3:end, 3) = ddth_db;
end

function [pHat, info] = calibrateGN(data, p0)
p = p0;
lambda = 1e-3;

for it = 1:60
    [r, J] = residualAndJacobian(p, data);
    H = J.'*J;
    g = J.'*r;
    dp = -(H + lambda*eye(3)) \ g;

    if norm(dp) < 1e-12
        break;
    end

    pNew = p + dp;
    [rNew, ~] = residualAndJacobian(pNew, data);

    if (rNew.'*rNew) < (r.'*r)
        p = pNew;
        lambda = 0.7*lambda;
    else
        lambda = 2.0*lambda;
    end
end

pHat = p;
info = struct('iterations', it, 'lambda_final', lambda);
end

function buildSimulinkModel()
% buildSimulinkModel
% Creates a simple model "Chapter2_Lesson5_SimulinkModel" that computes
% (dx, dy, dtheta) from (dphiL, dphiR) and parameters p in a MATLAB Function block.
%
% This is intended as an instructional scaffold; you can extend it to read
% encoder streams, integrate over time, and compare to external measurements.

model = 'Chapter2_Lesson5_SimulinkModel';
if bdIsLoaded(model); close_system(model, 0); end
new_system(model);
open_system(model);

add_block('simulink/Sources/In1', [model '/dphiL'], 'Position', [30 50 60 70]);
add_block('simulink/Sources/In1', [model '/dphiR'], 'Position', [30 110 60 130]);
add_block('simulink/Sources/In1', [model '/p'],     'Position', [30 170 60 190]);

add_block('simulink/User-Defined Functions/MATLAB Function', [model '/dd_kin'], ...
    'Position', [150 60 320 170]);

set_param([model '/dd_kin'], 'Script', sprintf([ ...
'function [dx,dy,dth] = dd_kin(dphiL,dphiR,p)\n' ...
'%% p = [rL;rR;b]\n' ...
'rL = p(1); rR = p(2); b = p(3);\n' ...
'sL = rL*dphiL; sR = rR*dphiR;\n' ...
'ds = 0.5*(sR+sL);\n' ...
'dth = (sR-sL)/b;\n' ...
'%% stable sinc/cosc\n' ...
'if abs(dth) < 1e-6\n' ...
'  f = 1 - dth^2/6;\n' ...
'  g = dth/2;\n' ...
'else\n' ...
'  f = sin(dth)/dth;\n' ...
'  g = (1-cos(dth))/dth;\n' ...
'end\n' ...
'dx = ds*f;\n' ...
'dy = ds*g;\n' ...
'end\n']));

add_block('simulink/Sinks/Out1', [model '/dx'], 'Position', [380 70 410 90]);
add_block('simulink/Sinks/Out1', [model '/dy'], 'Position', [380 115 410 135]);
add_block('simulink/Sinks/Out1', [model '/dtheta'], 'Position', [380 160 410 180]);

add_line(model, 'dphiL/1', 'dd_kin/1');
add_line(model, 'dphiR/1', 'dd_kin/2');
add_line(model, 'p/1', 'dd_kin/3');
add_line(model, 'dd_kin/1', 'dx/1');
add_line(model, 'dd_kin/2', 'dy/1');
add_line(model, 'dd_kin/3', 'dtheta/1');

save_system(model);
disp(['Saved Simulink model: ' model]);
end
