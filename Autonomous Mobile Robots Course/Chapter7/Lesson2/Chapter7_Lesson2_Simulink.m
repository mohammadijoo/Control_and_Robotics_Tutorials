% Chapter7_Lesson2_Simulink.m
% Simulink-oriented UKF step function for a MATLAB Function block.
%
% Usage (typical):
%   1) Create a Simulink model and add a "MATLAB Function" block.
%   2) Replace the function code with this file's "ukf_step" function.
%   3) Provide inputs:
%        u     : control input [v; w]
%        z     : measurement [range; bearing] to ONE selected landmark
%        lm    : landmark position [lx; ly]
%        reset : boolean reset signal (1 to reinitialize)
%   4) Outputs:
%        xhat  : estimated state [x; y; theta]
%
% Notes:
%   - For multiple landmarks, call ukf_step multiple times with different z/lm,
%     or stack measurements and extend the update equation.
%   - This code is "codegen-friendly" in principle (no dynamic sizing),
%     but you may need to adapt it depending on your Simulink Coder settings.

function xhat = ukf_step(u, z, lm, reset)
%#codegen

persistent x P params Q R

if isempty(x) || reset
    % Initialization
    x = [0.5; -0.5; -0.3];
    P = diag([0.8^2, 0.8^2, (20*pi/180)^2]);

    params.alpha = 0.35;
    params.beta  = 2.0;
    params.kappa = 0.0;

    sigma_xy = 0.02;
    sigma_th = 1.0*pi/180;
    Q = diag([sigma_xy^2, sigma_xy^2, sigma_th^2]);

    sigma_r = 0.15;
    sigma_b = 2.0*pi/180;
    R = diag([sigma_r^2, sigma_b^2]);
end

dt = 0.1;

% UKF predict with current input u
[x, P] = ukf_predict(x, P, Q, params, @(xx) f_motion(xx, u, dt));

% UKF update with current measurement z and landmark lm
[x, P] = ukf_update(x, P, R, params, z, @(xx) h_rangeBearing(xx, lm));

xhat = x;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Local functions (same as Chapter7_Lesson2.m)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function y = f_motion(x, u, dt)
px = x(1); py = x(2); th = x(3);
v  = u(1); w  = u(2);
y = [ px + dt*v*cos(th);
      py + dt*v*sin(th);
      wrapAngle(th + dt*w) ];
end

function z = h_rangeBearing(x, lm)
px = x(1); py = x(2); th = x(3);
dx = lm(1) - px;
dy = lm(2) - py;
r  = sqrt(dx^2 + dy^2);
b  = wrapAngle(atan2(dy, dx) - th);
z = [r; b];
end

function a = wrapAngle(a)
a = atan2(sin(a), cos(a));
end

function mu = meanAngle(angles, w)
s = sum(w .* sin(angles));
c = sum(w .* cos(angles));
mu = wrapAngle(atan2(s, c));
end

function [X, Wm, Wc] = sigmaPoints(x, P, params)
n = numel(x);
alpha = params.alpha; beta = params.beta; kappa = params.kappa;
lambda = alpha^2*(n+kappa) - n;

S = chol((n+lambda)*P, 'lower');

X = zeros(n, 2*n+1);
X(:,1) = x;
for i = 1:n
    X(:, 1+i)   = x + S(:,i);
    X(:, 1+i+n) = x - S(:,i);
end
X(3,:) = arrayfun(@wrapAngle, X(3,:));

Wm = ones(2*n+1,1) * (1/(2*(n+lambda)));
Wc = Wm;
Wm(1) = lambda/(n+lambda);
Wc(1) = Wm(1) + (1 - alpha^2 + beta);
end

function xbar = stateMean(X, Wm)
xbar = zeros(3,1);
xbar(1) = sum(Wm .* X(1,:)');
xbar(2) = sum(Wm .* X(2,:)');
xbar(3) = meanAngle(X(3,:)', Wm);
end

function dx = stateResidual(a, b)
dx = a - b;
dx(3) = wrapAngle(dx(3));
end

function [xPred, PPred] = ukf_predict(x, P, Q, params, f)
[X, Wm, Wc] = sigmaPoints(x, P, params);

Xp = zeros(size(X));
for i = 1:size(X,2)
    Xp(:,i) = f(X(:,i));
    Xp(3,i) = wrapAngle(Xp(3,i));
end

xPred = stateMean(Xp, Wm);

PPred = zeros(3,3);
for i = 1:size(Xp,2)
    dx = stateResidual(Xp(:,i), xPred);
    PPred = PPred + Wc(i) * (dx*dx');
end
PPred = PPred + Q;
end

function [xNew, PNew] = ukf_update(xPred, PPred, R, params, z, h)
[X, Wm, Wc] = sigmaPoints(xPred, PPred, params);

Z = zeros(2, size(X,2));
for i = 1:size(X,2)
    zi = h(X(:,i));
    zi(2) = wrapAngle(zi(2));
    Z(:,i) = zi;
end

zbar = zeros(2,1);
zbar(1) = sum(Wm .* Z(1,:)');
zbar(2) = meanAngle(Z(2,:)', Wm);

S = zeros(2,2);
Pxz = zeros(3,2);
for i = 1:size(X,2)
    dz = Z(:,i) - zbar;
    dz(2) = wrapAngle(dz(2));
    dx = stateResidual(X(:,i), xPred);
    S = S + Wc(i) * (dz*dz');
    Pxz = Pxz + Wc(i) * (dx*dz');
end
S = S + R;

K = Pxz / S;
innov = z - zbar;
innov(2) = wrapAngle(innov(2));

xNew = xPred + K*innov;
xNew(3) = wrapAngle(xNew(3));

PNew = PPred - K*S*K';
end
