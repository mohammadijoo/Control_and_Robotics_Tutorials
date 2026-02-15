
function [e, edot, ISE] = joint_space_error_matlab(qd, q, dt)
% qd, q : N-by-n arrays (desired and measured joint positions)
% dt    : sample time

arguments
    qd double
    q  double
    dt (1,1) double {mustBePositive}
end

assert(isequal(size(qd), size(q)), 'Size mismatch between qd and q');

e = qd - q;
edot           = zeros(size(e));
edot(2:end,:)  = diff(e) / dt;
edot(1,:)      = edot(2,:);

ISE = sum(sum(e.^2, 2)) * dt;
end

% Example usage:
% t  = 0:0.001:2;
% qd = [0.5*sin(2*pi*t') 0.3*sin(4*pi*t')];
% q  = [0.48*sin(2*pi*(t'-0.01)) 0.29*sin(4*pi*(t'-0.01))];
% [e, edot, ISE] = joint_space_error_matlab(qd, q, 0.001);
