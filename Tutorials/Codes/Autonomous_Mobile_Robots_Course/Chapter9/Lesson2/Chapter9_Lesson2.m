% Chapter9_Lesson2.m
% Autonomous Mobile Robots — Chapter 9, Lesson 2
% Log-Odds Updates and Bayesian Cells (Occupancy Grid Mapping)
%
% This script demonstrates a log-odds occupancy grid update using a simple
% inverse sensor model and Bresenham ray traversal.
%
% Notes:
%   - Requires base MATLAB for the script portion.
%   - Optional: Simulink model construction is included at the end (requires Simulink).

clear; clc; close all;

%% Parameters
W = 220; H = 220;
res = 0.05;            % meters per cell
origin = [-5.5, -5.5]; % world coordinate of cell (1,1) center
rangeMax = 6.0;

p0 = 0.5;
pOcc = 0.7;
pFree = 0.3;
lMin = -10; lMax = 10;

logOdds = zeros(H, W, 'single');

%% Helpers
logit = @(p) log(max(min(p, 1-1e-6), 1e-6) ./ (1 - max(min(p, 1-1e-6), 1e-6)));
sigmoid = @(l) 1 ./ (1 + exp(-l));

worldToMap = @(x,y) [round((x - origin(1)) / res) + 1, round((y - origin(2)) / res) + 1]; % 1-based

inBounds = @(ix,iy) (ix >= 1 && ix <= W && iy >= 1 && iy <= H);

updateCell = @(ix,iy,dl) assignin('caller','logOdds', ...
  localUpdate(logOdds, ix, iy, dl, lMin, lMax, inBounds));

%% Synthetic environment: two circles
circles = [ 1.5, 0.5, 0.6;
           -1.2, 1.2, 0.5];

poses = [ -2.0, -2.0, 0.2;
           0.0, -2.5, 0.6;
           2.0, -1.5, 1.0;
           2.0,  1.0, 1.6;
           0.0,  2.5, 2.5;
          -2.0,  2.0,-2.7];

angles = linspace(-pi/2, pi/2, 121);

%% Main loop: update scans
l0 = logit(p0);
lOcc = logit(pOcc) - l0;
lFree = logit(pFree) - l0;

for k = 1:size(poses,1)
    x = poses(k,1); y = poses(k,2); th = poses(k,3);

    ranges = zeros(size(angles));
    for i = 1:numel(angles)
        ranges(i) = raycastCircles(x, y, th, angles(i), rangeMax, circles);
    end

    s = worldToMap(x,y);
    sx = s(1); sy = s(2);

    for i = 1:numel(angles)
        a = angles(i); r = ranges(i);
        if ~isfinite(r) || r <= 0, continue; end
        rEff = min(r, rangeMax);

        bx = x + rEff * cos(th + a);
        by = y + rEff * sin(th + a);
        e = worldToMap(bx,by);
        ex = e(1); ey = e(2);

        ray = bresenhamLine(sx, sy, ex, ey);
        if isempty(ray), continue; end

        % Free cells (excluding endpoint)
        for j = 1:size(ray,1)-1
            ix = ray(j,1); iy = ray(j,2);
            if inBounds(ix,iy)
                logOdds(iy,ix) = min(max(logOdds(iy,ix) + lFree, lMin), lMax);
            end
        end

        % Occupied endpoint if hit
        if r < rangeMax - 1e-9
            ix = ray(end,1); iy = ray(end,2);
            if inBounds(ix,iy)
                logOdds(iy,ix) = min(max(logOdds(iy,ix) + lOcc, lMin), lMax);
            end
        end
    end
end

%% Visualize
P = sigmoid(double(logOdds));
figure('Color','w'); imagesc([origin(1), origin(1)+W*res], [origin(2), origin(2)+H*res], P);
axis image; set(gca,'YDir','normal'); colorbar;
title('Occupancy probability (log-odds fused)');
xlabel('x [m]'); ylabel('y [m]');

%% ---- Optional: programmatically build a Simulink "log-odds accumulator" block diagram ----
% This section creates a *conceptual* model that integrates delta_l updates
% and applies saturation. A full LiDAR ray-tracing sensor model is typically
% implemented in MATLAB Function blocks or external code.
%
% Requires Simulink. Uncomment to generate.
%
%{
modelName = 'Chapter9_Lesson2_LogOdds_Accumulator';
new_system(modelName); open_system(modelName);

add_block('simulink/Sources/In1', [modelName '/delta_l'], 'Position',[30 50 60 70]);
add_block('simulink/Discrete/Unit Delay', [modelName '/z^-1'], 'Position',[110 45 160 75]);
add_block('simulink/Math Operations/Add', [modelName '/add'], 'Position',[200 45 240 75]);
add_block('simulink/Discontinuities/Saturation', [modelName '/sat'], 'Position',[280 45 340 75]);
add_block('simulink/Sinks/Out1', [modelName '/l_t'], 'Position',[380 50 410 70]);

set_param([modelName '/z^-1'], 'InitialCondition', '0');
set_param([modelName '/sat'], 'LowerLimit', num2str(lMin), 'UpperLimit', num2str(lMax));

add_line(modelName, 'delta_l/1', 'add/2');
add_line(modelName, 'z^-1/1', 'add/1');
add_line(modelName, 'add/1', 'sat/1');
add_line(modelName, 'sat/1', 'l_t/1');
add_line(modelName, 'sat/1', 'z^-1/1');

save_system(modelName);
%}

%% ---- Local functions ----
function logOddsOut = localUpdate(logOddsIn, ix, iy, dl, lMin, lMax, inBoundsFun)
    logOddsOut = logOddsIn;
    if inBoundsFun(ix,iy)
        logOddsOut(iy,ix) = min(max(logOddsOut(iy,ix) + dl, lMin), lMax);
    end
end

function r = raycastCircles(x,y,th,a,rangeMax,circles)
    dx = cos(th+a); dy = sin(th+a);
    step = 0.02;
    t = 0.0;
    while t <= rangeMax
        px = x + t*dx; py = y + t*dy;
        for i = 1:size(circles,1)
            cx = circles(i,1); cy = circles(i,2); rr = circles(i,3);
            if (px-cx)^2 + (py-cy)^2 <= rr^2
                r = max(0.05, t);
                return;
            end
        end
        t = t + step;
    end
    r = rangeMax;
end

function pts = bresenhamLine(x0,y0,x1,y1)
    dx = abs(x1-x0);
    dy = abs(y1-y0);
    sx = 1; if x1 < x0, sx = -1; end
    sy = 1; if y1 < y0, sy = -1; end

    x = x0; y = y0;
    pts = [];

    if dy <= dx
        err = floor(dx/2);
        while x ~= x1
            pts = [pts; x, y]; %#ok<AGROW>
            err = err - dy;
            if err < 0
                y = y + sy;
                err = err + dx;
            end
            x = x + sx;
        end
        pts = [pts; x1, y1];
    else
        err = floor(dy/2);
        while y ~= y1
            pts = [pts; x, y]; %#ok<AGROW>
            err = err - dx;
            if err < 0
                x = x + sx;
                err = err + dy;
            end
            y = y + sy;
        end
        pts = [pts; x1, y1];
    end
end
