% Define 6-DOF rigidBodyTree robot (e.g., from URDF)
robot = importrobot("my6dof.urdf");
robot.DataFormat = "row";
robot.Gravity = [0 0 -9.81];

% Joint limits
qlim = robot.homeConfiguration;
nJ = numel(qlim);
qMin = zeros(1, nJ);
qMax = zeros(1, nJ);
for i = 1:nJ
    qMin(i) = qlim(i).JointPositionLimits(1);
    qMax(i) = qlim(i).JointPositionLimits(2);
end

% Obstacles (example: sphere at position p with radius r)
obsCenter = [0.5 0.0 0.3];
obsRadius = 0.15;

function coll = inCollision(robot, q, obsCenter, obsRadius)
    % Simple point-sampling collision check on end-effector
    eePose = getTransform(robot, q, robot.BodyNames{end});
    p = eePose(1:3, 4).';
    coll = norm(p - obsCenter) <= obsRadius;
end

function ok = edgeCollisionFree(robot, q1, q2, obsCenter, obsRadius, step)
    if nargin < 6, step = 0.05; end
    dq = q2 - q1;
    L = norm(dq, 2);
    nSteps = ceil(L / step);
    ok = true;
    for k = 0:nSteps
        alpha = k / max(nSteps, 1);
        q = q1 + alpha * dq;
        if inCollision(robot, q, obsCenter, obsRadius)
            ok = false;
            return;
        end
    end
end

% PRM parameters
nSamples = 800;
kNeighbors = 10;
samples = zeros(nSamples, nJ);
validMask = false(nSamples, 1);

for i = 1:nSamples
    q = qMin + (qMax - qMin).*rand(1, nJ);
    if ~inCollision(robot, q, obsCenter, obsRadius)
        samples(i, :) = q;
        validMask(i) = true;
    end
end
samples = samples(validMask, :);
n = size(samples, 1);

% Build adjacency
adj = cell(n, 1);
for i = 1:n
    qi = samples(i, :);
    d = vecnorm((samples - qi).');
    [~, idx] = sort(d);
    cnt = 0;
    for j = idx(2:end)
        if cnt >= kNeighbors, break; end
        if edgeCollisionFree(robot, qi, samples(j, :), obsCenter, obsRadius)
            w = norm(samples(j, :) - qi);
            adj{i}(end+1, :) = [j, w]; %#ok<AGROW>
            adj{j}(end+1, :) = [i, w]; %#ok<AGROW>
            cnt = cnt + 1;
        end
    end
end

% Start/goal and graph search omitted for brevity:
% - Insert q_start and q_goal as additional nodes
% - Connect to neighbors via edgeCollisionFree
% - Run Dijkstra to get sequence of joint configurations pathQ

% To execute pathQ in Simulink:
% 1. Create a timeseries object tsQ = timeseries(pathQ, t);
% 2. Feed tsQ into a "Joint-Space Trajectory" block driving the robot model.
      
