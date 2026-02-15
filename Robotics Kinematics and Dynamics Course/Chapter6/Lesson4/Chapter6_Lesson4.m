function [q, success, iters] = ik_newton_2r(xd, l1, l2, q0, tol, maxIter, alpha)
%IK_NEWTON_2R Newton IK for 2R planar manipulator.
%   xd       : desired [x_d; y_d]
%   l1, l2   : link lengths
%   q0       : initial guess [q1; q2]
%   tol      : tolerance on position error norm
%   maxIter  : maximum iterations
%   alpha    : step-size

if nargin < 5, tol = 1e-6; end
if nargin < 6, maxIter = 50; end
if nargin < 7, alpha = 1.0; end

q = q0(:);
xd = xd(:);
success = false;

for k = 1:maxIter
    x = fk_2r(q, l1, l2);
    r = x - xd;
    err = norm(r);
    if err < tol
        success = true;
        iters = k - 1;
        return;
    end
    J = jacobian_2r(q, l1, l2);
    dq = -J \ r;   % solve J*dq = -r
    q = q + alpha * dq;
end

iters = maxIter;
end


function [q, success, iters] = ik_dls_2r(xd, l1, l2, q0, tol, maxIter, alpha, lambda)
%IK_DLS_2R Damped least-squares IK for 2R planar manipulator.

if nargin < 5, tol = 1e-6; end
if nargin < 6, maxIter = 50; end
if nargin < 7, alpha = 1.0; end
if nargin < 8, lambda = 1e-2; end

q = q0(:);
xd = xd(:);
success = false;

for k = 1:maxIter
    x = fk_2r(q, l1, l2);
    r = x - xd;
    err = norm(r);
    if err < tol
        success = true;
        iters = k - 1;
        return;
    end
    J = jacobian_2r(q, l1, l2);
    JTJ = J.' * J;
    A = JTJ + (lambda^2) * eye(2);
    g = J.' * r;
    dq = -A \ g;
    q = q + alpha * dq;
end

iters = maxIter;
end


function x = fk_2r(q, l1, l2)
q1 = q(1); q2 = q(2);
x = [ l1*cos(q1) + l2*cos(q1+q2);
      l1*sin(q1) + l2*sin(q1+q2) ];
end


function J = jacobian_2r(q, l1, l2)
q1 = q(1); q2 = q(2);
s1  = sin(q1);     c1  = cos(q1);
s12 = sin(q1+q2);  c12 = cos(q1+q2);

J = [ -l1*s1 - l2*s12,  -l2*s12;
       l1*c1 + l2*c12,   l2*c12 ];
end
      
