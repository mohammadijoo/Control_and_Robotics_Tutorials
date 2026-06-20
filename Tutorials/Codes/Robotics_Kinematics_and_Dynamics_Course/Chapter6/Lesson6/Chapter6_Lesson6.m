function [qStar, success, iters] = ikDLS2R(xDes, q0, L1, L2, varargin)
%IKDLS2R Damped least-squares IK for 2R planar arm.
%   xDes: [2x1] desired end-effector position
%   q0:   [2x1] initial guess
%   L1,L2: link lengths

p = inputParser;
addParameter(p, 'lambda', 1e-2);
addParameter(p, 'epsTask', 1e-4);
addParameter(p, 'maxIters', 100);
addParameter(p, 'stepMax', 0.2);
addParameter(p, 'qMin', [-pi; -pi]);
addParameter(p, 'qMax', [ pi;  pi]);
parse(p, varargin{:});
lambda   = p.Results.lambda;
epsTask  = p.Results.epsTask;
maxIters = p.Results.maxIters;
stepMax  = p.Results.stepMax;
qMin     = p.Results.qMin(:);
qMax     = p.Results.qMax(:);

q = min(max(q0(:), qMin), qMax);
success = false;

for k = 1:maxIters
    [x, J] = fkJac2R(q, L1, L2);
    e = xDes(:) - x;
    err = norm(e);

    if err <= epsTask
        success = true;
        break;
    end

    H = J.' * J + (lambda^2) * eye(2);
    g = J.' * e;
    delta_q = H \ g;

    stepNorm = norm(delta_q);
    if stepNorm > stepMax
        delta_q = delta_q * (stepMax / stepNorm);
    end

    qNew = q + delta_q;
    qNew = min(max(qNew, qMin), qMax);
    q = qNew;
end

qStar = q;
iters = k;
end

function [x, J] = fkJac2R(q, L1, L2)
q1 = q(1); q2 = q(2);
x = [L1*cos(q1) + L2*cos(q1+q2);
     L1*sin(q1) + L2*sin(q1+q2)];
s1 = sin(q1); c1 = cos(q1);
s12 = sin(q1+q2); c12 = cos(q1+q2);
J = [-L1*s1 - L2*s12, -L2*s12;
      L1*c1 + L2*c12,  L2*c12];
end
      
