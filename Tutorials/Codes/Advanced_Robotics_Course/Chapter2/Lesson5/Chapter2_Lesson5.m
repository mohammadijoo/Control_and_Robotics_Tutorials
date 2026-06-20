function path = planManipAStar(startQ, goalQ, rigidBodyTree)
% PLANMANIPASTAR Lattice A* in joint space for a manipulator.
%   startQ, goalQ: 1xn joint vectors (radians)
%   rigidBodyTree: robotics.RigidBodyTree for collision checking

n = numel(startQ);
delta = 0.1 * ones(1, n);  % resolution

% joint limits from model
qlim = rigidBodyTree.homeConfiguration;
qMin = zeros(1, n);
qMax = zeros(1, n);
for i = 1:n
    qMin(i) = rigidBodyTree.Bodies{i}.Joint.PositionLimits(1);
    qMax(i) = rigidBodyTree.Bodies{i}.Joint.PositionLimits(2);
end

discretize = @(q) round((q - qMin) ./ delta);
undisc = @(k) qMin + k .* delta;

startK = discretize(startQ);
goalK  = discretize(goalQ);

open = java.util.PriorityQueue();
key = @(kvec) sprintf('%d_', kvec);
gScore = containers.Map();
parent = containers.Map('KeyType','char','ValueType','char');

gScore(key(startK)) = 0.0;
open.add({0.0, startK});  % {f, k}

while ~open.isEmpty()
    item = open.remove();
    f = item{1}; %#ok<NASGU>
    k = item{2};
    if all(k == goalK)
        % reconstruct
        pathK = k;
        kstr = key(k);
        while parent.isKey(kstr)
            kstr = parent(kstr);
            toks = sscanf(kstr, '%d_');
            pathK = [toks.'; pathK]; %#ok<AGROW>
        end
        % map to joint space
        path = zeros(size(pathK,1), n);
        for i = 1:size(pathK,1)
            path(i,:) = undisc(pathK(i,:));
        end
        return;
    end

    % neighbors: single joint +/-1 step
    for i = 1:n
        for step = [-1, 1]
            kk = k;
            kk(i) = kk(i) + step;
            if kk(i) <= round((qMin(i) - qMin(i)) / delta(i)) || ...
               kk(i) >= round((qMax(i) - qMin(i)) / delta(i))
                continue;
            end
            q = undisc(kk);
            % collision check via Robotics System Toolbox
            inCollision = checkCollision(rigidBodyTree, q, ...
                                         'IgnoreSelfCollision','on');
            if inCollision
                continue;
            end
            gOld = inf;
            kKey = key(kk);
            if isKey(gScore, kKey)
                gOld = gScore(kKey);
            end
            gNew = gScore(key(k)) + sum(abs(undisc(kk) - undisc(k)));
            if gNew < gOld
                gScore(kKey) = gNew;
                parent(kKey) = key(k);
                % heuristic: Euclidean joint distance
                h = norm(undisc(kk) - undisc(goalK));
                open.add({gNew + h, kk});
            end
        end
    end
end

error('No path found.');
end
      
