
function u = scalar_joint_wrapper(x, a, k, cScale, learnerNet)
% scalar_joint_wrapper: stability-preserving wrapper for a scalar joint
% Inputs:
%   x          - joint position error
%   a, k       - dynamics and nominal gain
%   cScale     - 0 < cScale < 1, scaling for Lyapunov bound
%   learnerNet - (optional) trained network, e.g., SeriesNetwork
%
% Output:
%   u          - wrapped control input

if nargin < 5 || isempty(learnerNet)
    u_l = 0;
else
    % Assume learnerNet takes a 1-by-1 input
    u_l = predict(learnerNet, x);
end

u_b = -k * x;
c   = cScale * (a + k);
bound = c * abs(x);

if abs(u_l) > bound
    u_l = sign(u_l) * bound;
end

u = u_b + u_l;
end
