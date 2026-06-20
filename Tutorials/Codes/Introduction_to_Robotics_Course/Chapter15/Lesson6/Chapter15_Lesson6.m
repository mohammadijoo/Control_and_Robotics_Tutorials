function [safe, d_required, d_available] = safetyEnvelope1D(distance, v, Tr, amax, dng, margin)
% safetyEnvelope1D  Speed-distance safety envelope check.
%   distance  - current distance to human [m]
%   v         - speed toward the human [m/s]
%   Tr        - reaction time [s]
%   amax      - max braking deceleration [m/s^2]
%   dng       - no-go distance [m]
%   margin    - extra safety margin [m]

if nargin < 6
    margin = 0.05;
end

if v < 0
    % Moving away from the human w.r.t. this constraint.
    safe = true;
    d_required = 0;
    d_available = inf;
    return;
end

% d_stop = v*Tr + v^2/(2*a_max)
d_required = v * Tr + v.^2 / (2 * amax);
d_available = max(distance - dng - margin, 0);

safe = (d_required <= d_available);
end
      
