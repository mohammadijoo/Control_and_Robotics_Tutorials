function d = configDistance(q1, q2, jointTypes, weights)
%CONFIGDISTANCE Weighted configuration-space distance for mixed joints.
%   q1, q2       : column vectors (n-by-1) or row vectors (1-by-n)
%   jointTypes   : cell array of strings: 'R' (revolute) or 'P' (prismatic)
%   weights      : numeric vector of length n

    q1 = q1(:);
    q2 = q2(:);
    n  = numel(q1);

    assert(numel(q2) == n, 'Dimension mismatch q2');
    assert(numel(jointTypes) == n, 'Dimension mismatch jointTypes');
    assert(numel(weights) == n, 'Dimension mismatch weights');

    deltas = zeros(n,1);
    for i = 1:n
        delta = q2(i) - q1(i);
        if jointTypes{i} == 'R'
            % Wrap to (-pi, pi]
            delta = wrapToPi(delta);
        end
        deltas(i) = weights(i) * delta;
    end

    d = norm(deltas, 2);
end

function ang = wrapToPi(ang)
%WRAPTOPI Wrap angle in radians to (-pi, pi]
    ang = mod(ang + pi, 2*pi);
    if ang < 0
        ang = ang + 2*pi;
    end
    ang = ang - pi;
end

% Example usage:
% q1 = [0; 0.5; 0.1];
% q2 = [pi-0.2; 0.4; 0.15];
% jointTypes = {'R','R','P'};
% weights = [1/pi; 1/pi; 2];
% d = configDistance(q1, q2, jointTypes, weights);
      
