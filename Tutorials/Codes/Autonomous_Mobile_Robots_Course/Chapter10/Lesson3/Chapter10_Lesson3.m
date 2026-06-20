% Chapter10_Lesson3.m
% Correlative Scan Matching (2D) — educational MATLAB implementation.
%
% Requires (recommended): Image Processing Toolbox for bwdist.
% If bwdist is unavailable, the script falls back to a crude distance proxy.

function Chapter10_Lesson3()
    rng(7);

    res = 0.05;
    size_m = 10.0;
    W = round(size_m / res);
    H = W;

    occ = zeros(H, W, 'uint8');
    occ(1,:) = 1; occ(end,:) = 1; occ(:,1) = 1; occ(:,end) = 1;

    % pillar
    cx = floor(W/2); cy = floor(H/2);
    rad = round(0.4 / res);
    occ((cy-rad):(cy+rad),(cx-rad):(cx+rad)) = 1;

    % likelihood field from distance transform
    sigma_m = 0.10;
    if exist('bwdist','file') == 2
        dist_m = bwdist(occ > 0) * res; % distance to nearest occupied cell
    else
        % crude fallback: iterative relaxation (Manhattan)
        dist = ones(H,W) * 1e9;
        dist(occ > 0) = 0;
        for it=1:200
            dist = min(dist, circshift(dist,[ 1, 0]) + 1);
            dist = min(dist, circshift(dist,[-1, 0]) + 1);
            dist = min(dist, circshift(dist,[ 0, 1]) + 1);
            dist = min(dist, circshift(dist,[ 0,-1]) + 1);
        end
        dist_m = dist * res;
    end
    field0 = exp(-(dist_m.^2) / (2*sigma_m^2));

    % pyramid (max-pooling)
    field1 = maxpool2(field0);
    field2 = maxpool2(field1);

    % true pose and initial guess
    ptrue = [2.4, 3.2, deg2rad(18.0)];
    p0    = [2.55, 3.05, deg2rad(10.0)];

    pts_r = simulate_scan_endpoints(occ, res, ptrue, 6.0, 0.01);

    seeds = struct('p', p0, 's', 0.0);

    seeds = search_level(field2, res*4.0, seeds, pts_r, 0.6, deg2rad(12), res*4.0, deg2rad(2.0), 50);
    seeds = search_level(field1, res*2.0, seeds, pts_r, 0.3, deg2rad(6),  res*2.0, deg2rad(1.0), 50);
    seeds = search_level(field0, res,     seeds, pts_r, 0.15,deg2rad(3),  res,     deg2rad(0.5), 50);

    best = seeds(1);
    for i=2:numel(seeds)
        if seeds(i).s > best.s
            best = seeds(i);
        end
    end

    fprintf('True pose:     x=%.3f y=%.3f th(deg)=%.2f\n', ptrue(1), ptrue(2), rad2deg(ptrue(3)));
    fprintf('Initial guess: x=%.3f y=%.3f th(deg)=%.2f\n', p0(1), p0(2), rad2deg(p0(3)));
    fprintf('Matched pose:  x=%.3f y=%.3f th(deg)=%.2f score=%.2f\n', best.p(1), best.p(2), rad2deg(best.p(3)), best.s);
end

function out = maxpool2(g)
    [H,W] = size(g);
    H2 = floor(H/2); W2 = floor(W/2);
    g = g(1:2*H2, 1:2*W2);
    out = zeros(H2,W2);
    for r=1:H2
        for c=1:W2
            block = g((2*r-1):(2*r),(2*c-1):(2*c));
            out(r,c) = max(block(:));
        end
    end
end

function pts_r = simulate_scan_endpoints(occ, res, pose_w_r, max_range, noise_std)
    % Sample occupied map points within range and express them in robot frame.
    [ys,xs] = find(occ(1:2:end,1:2:end) > 0);
    xs = (xs-1)*2; ys = (ys-1)*2;
    pts_w = [ (xs+0.5)*res, (ys+0.5)*res ];
    x = pose_w_r(1); y = pose_w_r(2); th = pose_w_r(3);

    ct = cos(th); st = sin(th);
    dx = pts_w(:,1) - x;
    dy = pts_w(:,2) - y;
    xr =  ct*dx + st*dy;
    yr = -st*dx + ct*dy;
    d  = hypot(xr, yr);
    keep = (d > 0.3) & (d < max_range);
    xr = xr(keep); yr = yr(keep);

    n = numel(xr);
    if n > 600
        idx = randperm(n, 600);
        xr = xr(idx); yr = yr(idx);
    end
    pts_r = [xr, yr] + randn(size([xr,yr]))*noise_std;
end

function score = score_pose(field, res_level, pts_r, p)
    x=p(1); y=p(2); th=p(3);
    ct = cos(th); st = sin(th);
    xw = ct*pts_r(:,1) - st*pts_r(:,2) + x;
    yw = st*pts_r(:,1) + ct*pts_r(:,2) + y;
    gx = floor(xw / res_level) + 1;
    gy = floor(yw / res_level) + 1;
    [H,W] = size(field);
    inside = (gx >= 1) & (gx <= W) & (gy >= 1) & (gy <= H);
    gx = gx(inside); gy = gy(inside);
    score = sum(field(sub2ind([H,W], gy, gx)));
end

function seeds_out = search_level(field, res_level, seeds_in, pts_r, winXY, winTh, stepXY, stepTh, topK)
    cands = [];
    for si=1:numel(seeds_in)
        pc = seeds_in(si).p;
        xs = (pc(1)-winXY):stepXY:(pc(1)+winXY);
        ys = (pc(2)-winXY):stepXY:(pc(2)+winXY);
        ths = (pc(3)-winTh):stepTh:(pc(3)+winTh);
        for xi=1:numel(xs)
            for yi=1:numel(ys)
                for ti=1:numel(ths)
                    p = [xs(xi), ys(yi), ths(ti)];
                    s = score_pose(field, res_level, pts_r, p);
                    cands = [cands; p, s]; %#ok<AGROW>
                end
            end
        end
    end
    % keep topK
    [~,ord] = sort(cands(:,4), 'descend');
    ord = ord(1:min(topK, numel(ord)));
    cands = cands(ord,:);
    seeds_out = repmat(struct('p',[0,0,0],'s',0), size(cands,1), 1);
    for i=1:size(cands,1)
        seeds_out(i).p = cands(i,1:3);
        seeds_out(i).s = cands(i,4);
    end
end
