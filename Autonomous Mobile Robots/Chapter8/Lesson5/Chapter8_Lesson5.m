% Chapter8_Lesson5.m
% Autonomous Mobile Robots — Chapter 8 (Particle-Filter Localization)
% Lesson 5 Lab: Implement MCL on a Map
%
% Self-contained MATLAB MCL demo on a 2D occupancy grid.
% Motion: sampled odometry model. Sensor: multi-beam likelihood via ray casting.

clear; clc; close all; rng(1);

% Map
res = 0.05; W = 220; H = 160;
occ = zeros(H, W, 'uint8');
occ(1,:) = 1; occ(end,:) = 1; occ(:,1) = 1; occ(:,end) = 1;
occ(31:55, 41:160) = 1;
occ(86:110, 21:90) = 1;
occ(86:140, 141:170) = 1;
occ(16:40, 181:205) = 1;

worldToGrid = @(wx,wy) [floor(wx/res)+1, floor(wy/res)+1]; % 1-index
isOcc = @(gx,gy) (gx<1 || gx>W || gy<1 || gy>H || occ(gy,gx)~=0);

beamAngles = deg2rad(linspace(-90,90,13));
rMax = 6.0;

% PF params
N = 1200;
sigmaHit = 0.20; zHit = 0.85; zRand = 0.15;
a1 = 0.03; a2 = 0.01; a3 = 0.03; a4 = 0.01;

% init particles uniformly on free cells
[fy, fx] = find(occ == 0);
idx = randi(numel(fx), N, 1);
px = (fx(idx) - 0.5) * res;
py = (fy(idx) - 0.5) * res;
pth = (rand(N,1)*2*pi - pi);
pw = ones(N,1)/N;

truePose = [2.0; 2.0; 0.0];
dt = 0.25; T = 180;
vCmd = 0.35; wCmd = 0.25;

trueHist = zeros(3,T);
estHist  = zeros(3,T);

for t = 1:T
    next = simulateStep(truePose, vCmd, wCmd, dt);

    g = worldToGrid(next(1), next(2));
    if isOcc(g(1), g(2))
        next = simulateStep(truePose, 0.0, 0.8, dt);
    end

    odom = odomFromTrue(truePose, next);
    z = rangeScan(next, beamAngles, rMax, 0.05, @(x,y,th) rayCast(x,y,th,rMax,res,worldToGrid,isOcc));

    % Motion update
    dRot1 = odom(1); dTrans = odom(2); dRot2 = odom(3);
    sr1 = sqrt(a1*dRot1^2 + a2*dTrans^2);
    st  = sqrt(a3*dTrans^2 + a4*(dRot1^2 + dRot2^2));
    sr2 = sqrt(a1*dRot2^2 + a2*dTrans^2);

    dr1 = dRot1 + sr1*randn(N,1);
    dtr = dTrans + st*randn(N,1);
    dr2 = dRot2 + sr2*randn(N,1);

    xn = px + dtr .* cos(pth + dr1);
    yn = py + dtr .* sin(pth + dr1);
    thn = wrapAngle(pth + dr1 + dr2);

    for i = 1:N
        g = worldToGrid(xn(i), yn(i));
        if isOcc(g(1), g(2))
            xn(i) = px(i); yn(i) = py(i);
        end
    end
    px = xn; py = yn; pth = thn;

    % Sensor update (log domain)
    logw = zeros(N,1);
    normHit = 1/(sqrt(2*pi)*sigmaHit);
    unif = 1/rMax;
    invSig2 = 1/(sigmaHit^2);
    ray = @(x,y,th) rayCast(x,y,th,rMax,res,worldToGrid,isOcc);

    for bi = 1:numel(beamAngles)
        zexp = zeros(N,1);
        for i = 1:N
            zexp(i) = ray(px(i), py(i), pth(i) + beamAngles(bi));
        end
        dz = z(bi) - zexp;
        pHit  = zHit * normHit .* exp(-0.5*(dz.^2)*invSig2);
        pRand = zRand * unif;
        logw = logw + log(pHit + pRand + 1e-12);
    end

    logw = logw - max(logw);
    w = exp(logw);
    wsum = sum(w);
    pw = (wsum>0 && isfinite(wsum)) * (w/wsum) + (wsum<=0 || ~isfinite(wsum)) * (ones(N,1)/N);

    Neff = 1/sum(pw.^2);
    if Neff < 0.55*N
        idx2 = systematicResample(pw);
        px = px(idx2); py = py(idx2); pth = pth(idx2);
        pw = ones(N,1)/N;

        k = round(0.03*N);
        ridx = randi(numel(fx), k, 1);
        px(1:k) = (fx(ridx) - 0.5) * res;
        py(1:k) = (fy(ridx) - 0.5) * res;
        pth(1:k) = (rand(k,1)*2*pi - pi);
    end

    ex = sum(pw .* px);
    ey = sum(pw .* py);
    eth = atan2(sum(pw .* sin(pth)), sum(pw .* cos(pth)));

    truePose = next;
    trueHist(:,t) = truePose;
    estHist(:,t)  = [ex; ey; eth];
end

figure; hold on; axis equal;
imagesc([0 W*res], [0 H*res], flipud(occ)); colormap(gray);
set(gca,'YDir','normal');
plot(trueHist(1,:), trueHist(2,:), 'LineWidth', 1.5);
plot(estHist(1,:), estHist(2,:), 'LineWidth', 1.5);
xlabel('x [m]'); ylabel('y [m]');
title('MCL on an Occupancy Grid Map');
legend('map', 'true', 'MCL estimate');

function a = wrapAngle(a), a = mod(a + pi, 2*pi) - pi; end

function pose2 = simulateStep(pose, v, w, dt)
  pose2 = pose;
  pose2(1) = pose(1) + v*cos(pose(3))*dt;
  pose2(2) = pose(2) + v*sin(pose(3))*dt;
  pose2(3) = wrapAngle(pose(3) + w*dt);
end

function odom = odomFromTrue(p1, p2)
  dx = p2(1)-p1(1); dy = p2(2)-p1(2);
  dTrans = sqrt(dx^2 + dy^2);
  dir = atan2(dy, dx);
  dRot1 = (dTrans > 1e-9) * wrapAngle(dir - p1(3));
  dRot2 = wrapAngle(p2(3) - p1(3) - dRot1);
  odom = [dRot1; dTrans; dRot2];
end

function z = rangeScan(pose, beamAngles, rMax, noiseSigma, ray)
  z = zeros(numel(beamAngles),1);
  for i = 1:numel(beamAngles)
    r = ray(pose(1), pose(2), pose(3) + beamAngles(i)) + noiseSigma*randn();
    z(i) = min(max(r, 0), rMax);
  end
end

function idx = systematicResample(w)
  N = numel(w);
  u0 = rand()/N;
  cdf = cumsum(w);
  idx = zeros(N,1);
  j = 1;
  for i = 1:N
    u = u0 + (i-1)/N;
    while u > cdf(j) && j < N, j = j + 1; end
    idx(i) = j;
  end
end

function r = rayCast(x,y,th,rMax,res,worldToGrid,isOcc)
  step = 0.02;
  r = 0.0;
  while r < rMax
    wx = x + r*cos(th);
    wy = y + r*sin(th);
    g = worldToGrid(wx, wy);
    if isOcc(g(1), g(2)), return; end
    r = r + step;
  end
  r = rMax;
end
