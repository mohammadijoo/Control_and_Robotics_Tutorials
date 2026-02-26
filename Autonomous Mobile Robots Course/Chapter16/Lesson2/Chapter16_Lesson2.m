% Chapter16_Lesson2.m
% Autonomous Mobile Robots — Chapter 16, Lesson 2: Velocity Obstacles (VO) Sampling Demo
%
% Educational reference implementation:
% - Finite-horizon collision check in relative motion
% - Sampling-based velocity selection outside VO
%
% Simulink note:
%   You can paste chooseVelocityVO() into a MATLAB Function block and feed
%   current state + neighbor states each control tick.

function Chapter16_Lesson2()
    rng(1);

    N = 8;
    steps = 400;
    dt = 0.05;
    T  = 2.5;
    R0 = 5.0;

    agents = struct();
    for i=1:N
        ang = 2*pi*(i-1)/N;
        p = [R0*cos(ang); R0*sin(ang)];
        agents(i).p = p;
        agents(i).v = [0;0];
        agents(i).goal = -p;
        agents(i).radius = 0.35;
        agents(i).vmax = 1.2;
    end

    for k=1:steps
        nextV = zeros(2,N);
        for i=1:N
            neigh = agents([1:i-1, i+1:N]);
            nextV(:,i) = chooseVelocityVO(agents(i), neigh, dt, T);
        end

        for i=1:N
            agents(i).v = nextV(:,i);
            agents(i).p = agents(i).p + dt * agents(i).v;
        end

        if mod(k,50)==0
            fprintf('Step %d: agent1 p=(%.3f, %.3f)\n', k, agents(1).p(1), agents(1).p(2));
        end
    end

    disp('Done.');
end

function v = chooseVelocityVO(a, neighbors, dt, T)
    % Preferred velocity to goal
    toGoal = a.goal - a.p;
    d = norm(toGoal);
    if d < 1e-9
        vPref = [0;0];
    else
        spd = min(a.vmax, d / max(dt, 1e-3));
        vPref = toGoal / (d + 1e-12) * spd;
    end

    % Candidate velocities
    cand = sampleDisk(a.vmax, 1200);
    cand = [cand, clampNorm(vPref, a.vmax)];

    bestCost = inf;
    v = [0;0];

    for c = 1:size(cand,2)
        vc = cand(:,c);
        feasible = true;
        minTTC = inf;

        for j = 1:numel(neighbors)
            nb = neighbors(j);
            pRel = nb.p - a.p;
            vRel = vc - nb.v;
            R = a.radius + nb.radius;

            [coll, ~] = ttcInHorizon(pRel, vRel, R, T);
            if coll
                feasible = false;
                break;
            end

            [collAny, tAny] = ttcInHorizon(pRel, vRel, R, 1e6);
            if collAny
                minTTC = min(minTTC, tAny);
            end
        end

        if ~feasible
            continue;
        end

        prefCost = norm(vc - vPref)^2;
        safetyCost = 0.0;
        if isfinite(minTTC)
            safetyCost = 1.0 / (minTTC + 1e-6);
        end

        cost = 1.0*prefCost + 2.0*safetyCost;
        if cost < bestCost
            bestCost = cost;
            v = vc;
        end
    end

    if isinf(bestCost)
        v = 0.2 * clampNorm(a.v, a.vmax); % failsafe slow down
    end
end

function [collides, tHit] = ttcInHorizon(pRel, vRel, R, T)
    if norm(pRel) <= R
        collides = true;
        tHit = 0.0;
        return;
    end

    a = dot(vRel,vRel);
    b = 2*dot(pRel,vRel);
    c = dot(pRel,pRel) - R^2;

    if a < 1e-12
        collides = false;
        tHit = inf;
        return;
    end

    disc = b^2 - 4*a*c;
    if disc < 0
        collides = false;
        tHit = inf;
        return;
    end

    sdisc = sqrt(disc);
    t1 = (-b - sdisc)/(2*a);
    t2 = (-b + sdisc)/(2*a);

    tHit = inf;
    if t1 >= 0
        tHit = t1;
    elseif t2 >= 0
        tHit = 0.0;
    end

    collides = (tHit >= 0) && (tHit <= T);
    if ~collides
        tHit = inf;
    end
end

function V = sampleDisk(vmax, n)
    V = zeros(2,n);
    for i=1:n
        ang = 2*pi*rand();
        r = sqrt(rand())*vmax;
        V(:,i) = [r*cos(ang); r*sin(ang)];
    end
end

function v = clampNorm(v, vmax)
    n = norm(v);
    if n <= vmax
        return;
    end
    v = v * (vmax/(n + 1e-12));
end
