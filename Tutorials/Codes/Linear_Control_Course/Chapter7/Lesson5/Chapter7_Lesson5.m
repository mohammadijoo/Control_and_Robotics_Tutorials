% Characteristic polynomial: (s^2 + 1)^2
den1 = [1 0 2 0 1];
r1 = roots(den1);

% Approximate multiplicities
tol = 1e-6;
clusters = {};
used = false(size(r1));
for i = 1:length(r1)
    if used(i), continue; end
    idx = find(abs(r1 - r1(i)) < tol);
    used(idx) = true;
    clusters{end+1} = r1(idx); %#ok<AGROW>
end

% Classify
unstable = false; marginal = false;
for k = 1:numel(clusters)
    c = clusters{k};
    s_avg = mean(c);
    m = numel(c);
    if real(s_avg) > tol
        unstable = true;
    elseif abs(real(s_avg)) <= tol
        if m >= 2, unstable = true; else marginal = true; end
    end
end

if unstable
    disp('unstable');
elseif marginal
    disp('marginally stable');
else
    disp('asymptotically stable');
end

% Simple robot joint model: inertia J, friction b
J = 0.01; b = 0.0;
num_joint = 1;
den_joint = [J 0 0];  % J s^2
G_joint = tf(num_joint, den_joint);
figure; step(G_joint); grid on;
title('Step response of double-integrator joint (unstable)');

% In Robotics System Toolbox or Peter Corke''s Robotics Toolbox,
% G_joint can serve as the linearized plant for a single revolute joint.
