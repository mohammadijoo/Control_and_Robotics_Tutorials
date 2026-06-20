% Chapter 6 - Lesson 1: Belief as a Probability Distribution Over Pose
% Autonomous Mobile Robots (Control Engineering)
%
% This script demonstrates:
%   1) A Gaussian belief over pose x = [x; y; theta] (theta approximated on R)
%   2) Discretization onto a grid and normalization by cell volume
%   3) Expected pose with circular mean for theta
%   4) Programmatic creation of a simple Simulink model that calls a MATLAB Function
%      to normalize a vector (illustrating how belief normalization can be embedded).

clear; clc;

wrapToPi = @(a) mod(a + pi, 2*pi) - pi;

% Gaussian belief parameters
mu = [2.0; -1.0; 0.7];
Sigma = diag([0.2^2, 0.3^2, (10*pi/180)^2]);

% Evaluate PDF at a point (manual 3D Gaussian pdf)
x_test = [2.1; -1.2; 0.75];
dx = x_test - mu;
dx(3) = wrapToPi(dx(3));
invS = inv(Sigma);
detS = det(Sigma);
normC = 1/sqrt((2*pi)^3 * detS);
pdf_test = normC * exp(-0.5 * (dx' * invS * dx));
fprintf('pdf(x_test) = %.6e\n', pdf_test);

% Grid discretization
xs = linspace(1.0, 3.0, 101);
ys = linspace(-2.0, 0.0, 101);
thetas = linspace(-pi, pi, 121+1); thetas(end) = []; % endpoint excluded

dxg = xs(2)-xs(1);
dyg = ys(2)-ys(1);
dthg = thetas(2)-thetas(1);
cellVol = dxg * dyg * dthg;

b = zeros(length(xs), length(ys), length(thetas));
for i = 1:length(xs)
    for j = 1:length(ys)
        for k = 1:length(thetas)
            x = [xs(i); ys(j); thetas(k)];
            d = x - mu;
            d(3) = wrapToPi(d(3));
            b(i,j,k) = normC * exp(-0.5 * (d' * invS * d));
        end
    end
end

Z = sum(b(:)) * cellVol;
b = b / Z;
fprintf('grid normalization check: %.12f\n', sum(b(:)) * cellVol);

% Expectation with circular mean
[X, Y, TH] = ndgrid(xs, ys, thetas);
ex = sum(X(:) .* b(:)) * cellVol;
ey = sum(Y(:) .* b(:)) * cellVol;

esin = sum(sin(TH(:)) .* b(:)) * cellVol;
ecos = sum(cos(TH(:)) .* b(:)) * cellVol;
eth = atan2(esin, ecos);

fprintf('grid E[pose] = [%.6f, %.6f, %.6f]\n', ex, ey, eth);

%% (Optional) Simulink: Create a tiny model for normalization
% The model generates a random vector u and normalizes it with a MATLAB Function block.
% This is a minimal pattern you can adapt to normalize a discrete belief vector in Simulink.

modelName = 'Chapter6_Lesson1_Simulink';
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
new_system(modelName);
open_system(modelName);

add_block('simulink/Sources/Random Number', [modelName '/Random']);
set_param([modelName '/Random'], 'Mean', '0', 'Variance', '1', 'SampleTime', '0.1');

add_block('simulink/Math Operations/Gain', [modelName '/Gain']);
set_param([modelName '/Gain'], 'Gain', '1');

add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/Normalize']);
set_param([modelName '/Normalize'], 'MATLABFcn', ...
['function y = f(u)\n' ...
'  % Normalize a scalar or vector to sum to 1 (simple example)\n' ...
'  u = abs(u);\n' ...
'  s = sum(u);\n' ...
'  if s == 0\n' ...
'    y = u;\n' ...
'  else\n' ...
'    y = u / s;\n' ...
'  end\n' ...
'end']);

add_block('simulink/Sinks/Scope', [modelName '/Scope']);

add_line(modelName, 'Random/1', 'Gain/1');
add_line(modelName, 'Gain/1', 'Normalize/1');
add_line(modelName, 'Normalize/1', 'Scope/1');

set_param(modelName, 'StopTime', '2.0');
save_system(modelName);

disp(['Created Simulink model: ' modelName '.slx']);
