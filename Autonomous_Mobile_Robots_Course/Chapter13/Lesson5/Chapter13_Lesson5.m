% Chapter13_Lesson5.m
% MATLAB utilities for VIO/Visual SLAM lab:
%  1) ATE evaluation (TUM format) with optional similarity scale.
%  2) Allan variance-based IMU noise parameter estimation (gyro/accel).
%
% Usage examples:
%   out = ate_eval_tum("groundtruth.txt","estimated.txt",0.02,true);
%   imu = readmatrix("imu.csv");  % columns: t, ax, ay, az, gx, gy, gz
%   av  = allan_imu(imu(:,1), imu(:,2:4), imu(:,5:7));
%
% NOTE: For monocular VO/SLAM, set allowScale=true.
function demo_Chapter13_Lesson5()
  disp("Run functions: ate_eval_tum, allan_imu");
end

function out = ate_eval_tum(gtPath, estPath, maxDt, allowScale)
  gt = read_tum(gtPath);
  est = read_tum(estPath);

  pairs = associate_by_time(est.t, gt.t, maxDt);
  if size(pairs,1) < 3
    error("Too few associated poses.");
  end

  A = est.p(pairs(:,1), :);
  B = gt.p(pairs(:,2), :);

  [s,R,t] = umeyama(A, B, allowScale);
  A_al = (s*(R*A') + t)';
  e = A_al - B;
  se = sum(e.^2,2);
  out.n_pairs = size(pairs,1);
  out.scale = s;
  out.R = R;
  out.t = t;
  out.rmse = sqrt(mean(se));
  out.median = median(sqrt(se));
  out.max = max(sqrt(se));
end

function tr = read_tum(path)
  fid = fopen(path,'r');
  if fid < 0, error("Cannot open file"); end
  t = [];
  p = [];
  while true
    line = fgetl(fid);
    if ~ischar(line), break; end
    line = strtrim(line);
    if isempty(line) || startsWith(line,"#"), continue; end
    parts = split(line);
    if numel(parts) < 8, continue; end
    tt = str2double(parts{1});
    tx = str2double(parts{2});
    ty = str2double(parts{3});
    tz = str2double(parts{4});
    t(end+1,1) = tt; %#ok<AGROW>
    p(end+1,:) = [tx ty tz]; %#ok<AGROW>
  end
  fclose(fid);
  tr.t = t;
  tr.p = p;
end

function pairs = associate_by_time(ta, tb, maxDt)
  pairs = [];
  j = 1;
  used = false(numel(tb),1);
  for i = 1:numel(ta)
    t = ta(i);
    while (j+1) <= numel(tb) && tb(j) < t
      j = j+1;
    end
    cand = [j, j-1];
    best = -1;
    bestErr = inf;
    for c = cand
      if c >= 1 && c <= numel(tb) && ~used(c)
        err = abs(tb(c)-t);
        if err < bestErr
          bestErr = err;
          best = c;
        end
      end
    end
    if best > 0 && bestErr <= maxDt
      pairs(end+1,:) = [i, best]; %#ok<AGROW>
      used(best) = true;
    end
  end
end

function [s,R,t] = umeyama(A, B, withScale)
  % A,B are Nx3
  n = size(A,1);
  muA = mean(A,1);
  muB = mean(B,1);
  X = A - muA;
  Y = B - muB;
  Sigma = (Y'*X)/n;

  [U,D,V] = svd(Sigma);
  S = eye(3);
  if det(U)*det(V) < 0
    S(3,3) = -1;
  end
  R = U*S*V';
  if withScale
    varA = sum(sum(X.^2))/n;
    s = trace(D*S)/(varA + 1e-12);
  else
    s = 1.0;
  end
  t = muB' - s*R*muA';
end

function av = allan_imu(t, acc, gyro)
  % Compute Allan deviation for 3-axis accel and gyro.
  % t: Nx1 seconds (monotonic). acc: Nx3 (m/s^2). gyro: Nx3 (rad/s).
  % Output av contains taus and sigma(taus) per axis.
  dt = median(diff(t));
  if ~isfinite(dt) || dt <= 0, error("Invalid dt"); end

  maxM = floor(numel(t)/10);
  Ms = unique(round(logspace(0, log10(maxM), 50)));
  taus = Ms * dt;

  av.taus = taus;
  av.acc = cell(1,3);
  av.gyro = cell(1,3);

  for ax = 1:3
    av.acc{ax} = allan_dev_1d(acc(:,ax), Ms);
    av.gyro{ax} = allan_dev_1d(gyro(:,ax), Ms);
  end

  % Rough noise parameters (white + random walk) from log-log slopes:
  % white noise ~ slope -1/2, random walk ~ slope +1/2.
  % Users typically pick two tau regions and fit manually for robust estimates.
end

function sigma = allan_dev_1d(x, Ms)
  x = x(:);
  N = numel(x);
  sigma = nan(numel(Ms),1);
  for k = 1:numel(Ms)
    m = Ms(k);
    K = floor(N/(2*m));
    if K < 2, continue; end
    % cluster averages
    y = zeros(2*K,1);
    for i = 1:(2*K)
      idx = (i-1)*m + (1:m);
      y(i) = mean(x(idx));
    end
    d = diff(y);
    sigma(k) = sqrt(0.5*mean(d.^2));
  end
end
