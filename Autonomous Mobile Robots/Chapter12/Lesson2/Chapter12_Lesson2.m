% Chapter12_Lesson2.m
% Loop Closure Detection Concepts — MATLAB/Simulink-oriented demo
%
% This script:
%   1) Builds a toy square-loop trajectory (ground truth) and a drifting odometry.
%   2) Creates simple "place descriptors" (bag-of-words style).
%   3) Performs cosine retrieval + chi-square geometric gating.
%   4) (Optional) Creates a minimal Simulink model with a MATLAB Function block
%      that calls the loop-closure detector for streaming use.
%
% Tested with base MATLAB. If you have Statistics & ML Toolbox, you can replace
% brute-force retrieval with KD-tree / knnsearch.

clear; clc; rng(0);

N = 420; SIDE = 25.0; MIN_SEP = 40; V = 300; N_PLACES = 90;

%% 1) Ground truth loop (square)
perim = 4*SIDE;
s = linspace(0, perim, N+1); s(end) = [];
gt = zeros(N,3);

for i = 1:N
    si = s(i);
    seg = floor(si / SIDE);
    u = mod(si, SIDE);
    if seg==0
        gt(i,:) = [u, 0, 0];
    elseif seg==1
        gt(i,:) = [SIDE, u, pi/2];
    elseif seg==2
        gt(i,:) = [SIDE-u, SIDE, pi];
    else
        gt(i,:) = [0, SIDE-u, -pi/2];
    end
end

wrap = @(a) mod(a+pi,2*pi)-pi;

%% 2) Drifting odometry
odo = zeros(N,3);
odo(1,:) = gt(1,:);
sigma_xy = 0.03; sigma_th = 0.003;
drift = [0.002, -0.001, 0.0002];

for k = 2:N
    d = gt(k,:) - gt(k-1,:);
    d(3) = wrap(d(3));
    noise = [sigma_xy*randn, sigma_xy*randn, sigma_th*randn];
    inc = d + noise + drift;
    odo(k,:) = odo(k-1,:) + inc;
    odo(k,3) = wrap(odo(k,3));
end

%% 3) Place prototypes and descriptors
prot = zeros(N_PLACES, V);
for p = 1:N_PLACES
    idx = randperm(V, 20);
    prot(p,idx) = 0.5 + 1.5*rand(1,20);
    prot(p,:) = prot(p,:) ./ (norm(prot(p,:)) + 1e-12);
end

cellSize = 3.2;
qx = floor(gt(:,1)/cellSize);
qy = floor(gt(:,2)/cellSize);
pid = mod(qx*73856093 + qy*19349663, N_PLACES) + 1;

C = zeros(N,V);
for i=1:N
    base = prot(pid(i),:);
    lam = 8.0*base + 0.15;
    C(i,:) = poissrnd(lam);
    % dropout (viewpoint changes)
    drop = rand(1,V) < 0.02;
    C(i,drop) = 0;
end

% TF normalization (IDF omitted for brevity)
X = C ./ (sum(C,2) + 1e-12);

%% 4) Ground-truth loop edges
gtLoop = false(N,N);
gtCount = 0;
for i=1:N
    for j=1:(i-MIN_SEP)
        if norm(gt(i,1:2)-gt(j,1:2)) < 1.4
            gtLoop(i,j) = true;
            gtCount = gtCount + 1;
        end
    end
end

%% 5) Retrieval + geometric gate
CHI2_3_0995 = 12.838; % approx chi2inv(0.995,3)
sigma_gate_xy = 0.20;
sigma_gate_th = 0.07;

det = 0; tp = 0; fp = 0;

for i=1:N
    jmax = i - MIN_SEP;
    if jmax <= 0, continue; end

    % brute-force cosine retrieval
    bestSim = -inf; bestJ = -1;
    xi = X(i,:);
    for j=1:jmax
        sim = dot(xi, X(j,:)) / (norm(xi)*norm(X(j,:)) + 1e-12);
        if sim > bestSim
            bestSim = sim;
            bestJ = j;
        end
    end

    if bestSim < 0.75, continue; end

    % geometric measurement z_ij around ground-truth
    pred = odo(i,:) - odo(bestJ,:);
    pred(3) = wrap(pred(3));
    z = (gt(i,:) - gt(bestJ,:)) + [sigma_gate_xy*randn, sigma_gate_xy*randn, sigma_gate_th*randn];
    z(3) = wrap(z(3));

    r = z - pred;
    r(3) = wrap(r(3));

    d2 = (r(1)^2)/(sigma_gate_xy^2) + (r(2)^2)/(sigma_gate_xy^2) + (r(3)^2)/(sigma_gate_th^2);
    if d2 > CHI2_3_0995, continue; end

    det = det + 1;
    if gtLoop(i,bestJ), tp = tp + 1; else, fp = fp + 1; end
end

prec = tp / (tp + fp + 1e-12);
rec  = tp / (gtCount + 1e-12);

fprintf('N=%d\n', N);
fprintf('GT loops=%d\n', gtCount);
fprintf('Detected=%d\n', det);
fprintf('Precision=%.3f  Recall=%.3f\n', prec, rec);

%% 6) Optional Simulink model scaffold (streaming)
% This creates a simple model with:
%   From Workspace -> MATLAB Function (LoopClosureStep) -> Display
% Disable if you do not have Simulink.
try
    mdl = 'Chapter12_Lesson2_LoopClosureModel';
    if bdIsLoaded(mdl), close_system(mdl,0); end
    new_system(mdl); open_system(mdl);

    add_block('simulink/Sources/From Workspace', [mdl '/Descriptors'], 'Position', [50 60 160 100]);
    add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/LoopClosureStep'], 'Position', [240 45 410 120]);
    add_block('simulink/Sinks/Display', [mdl '/Decision'], 'Position', [480 65 560 95]);

    add_line(mdl, 'Descriptors/1', 'LoopClosureStep/1');
    add_line(mdl, 'LoopClosureStep/1', 'Decision/1');

    % Insert code into MATLAB Function block (simple threshold on similarity)
    blk = [mdl '/LoopClosureStep'];
    code = [
        "function isLoop = LoopClosureStep(desc)\n" + ...
        "% desc: vector (1xV) descriptor at current time\n" + ...
        "% NOTE: Replace with a persistent database + nearest neighbor search.\n" + ...
        "persistent db;\n" + ...
        "if isempty(db)\n" + ...
        "  db = desc;\n" + ...
        "  isLoop = false;\n" + ...
        "  return;\n" + ...
        "end\n" + ...
        "sim = (desc*db')/(norm(desc)*norm(db)+1e-12);\n" + ...
        "isLoop = sim > 0.85;\n" + ...
        "db = [db; desc];\n" + ...
        "end\n"
    ];
    set_param(blk, 'Script', code);

    save_system(mdl);
    fprintf('\nSimulink model created: %s.slx\n', mdl);

catch ME
    fprintf('\nSimulink scaffold skipped: %s\n', ME.message);
end
