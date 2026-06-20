function solutions = ik2R(x, y, l1, l2, jointLimits)
%IK2R Analytic IK for planar 2R arm.
%   solutions is an N-by-2 matrix of [theta1, theta2].

    if nargin < 5
        jointLimits = [];
    end

    r2 = x.^2 + y.^2;
    c2 = (r2 - l1^2 - l2^2) / (2.0 * l1 * l2);

    if abs(c2) > 1.0
        solutions = zeros(0, 2); % no real solution
        return;
    end

    s2_pos = sqrt(max(0.0, 1.0 - c2^2));
    s2_candidates = [s2_pos; -s2_pos];

    solutions = [];
    for k = 1:numel(s2_candidates)
        s2 = s2_candidates(k);
        theta2 = atan2(s2, c2);
        k1 = l1 + l2 * c2;
        k2 = l2 * s2;
        theta1 = atan2(y, x) - atan2(k2, k1);

        t1 = wrapToPi(theta1);
        t2 = wrapToPi(theta2);

        if ~isempty(jointLimits)
            lower = jointLimits(1, :);
            upper = jointLimits(2, :);
            if any(t1 < lower(1) | t1 > upper(1)) || ...
               any(t2 < lower(2) | t2 > upper(2))
                continue;
            end
        end

        solutions(end+1, :) = [t1, t2]; %#ok<AGROW>
    end
end

function ang = wrapToPi(angle)
%WRAPTOPI Wrap angle to (-pi, pi].
    ang = mod(angle + pi, 2.0 * pi);
    idx = ang < 0.0;
    ang(idx) = ang(idx) + 2.0 * pi;
    ang = ang - pi;
end
      
