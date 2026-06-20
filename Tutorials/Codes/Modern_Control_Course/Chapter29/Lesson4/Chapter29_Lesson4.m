% Chapter29_Lesson4.m
% Numerical controllability and observability Gramians for a 2-state LTV system.
% Run in MATLAB or GNU Octave.

clear; clc;

t0 = 0.0;
tf = 6.0;
nSteps = 4000;
h = (tf - t0) / nSteps;

times = linspace(t0, tf, nSteps + 1);
phis = zeros(2, 2, nSteps + 1);
Phi = eye(2);
phis(:, :, 1) = Phi;

for k = 1:nSteps
    t = times(k);
    Phi = rk4StepPhi(Phi, t, h);
    phis(:, :, k + 1) = Phi;
end

PhiTfT0 = phis(:, :, end);
Wc = zeros(2, 2);
Wo = zeros(2, 2);

for k = 1:(nSteps + 1)
    s = times(k);
    weight = 1.0;
    if k == 1 || k == nSteps + 1
        weight = 0.5;
    end

    PhiST0 = phis(:, :, k);
    PhiTfS = PhiTfT0 / PhiST0;

    bs = Bmat(s);
    cs = Cmat(s);

    Wc = Wc + weight * h * (PhiTfS * bs * bs' * PhiTfS');
    Wo = Wo + weight * h * (PhiST0' * cs' * cs * PhiST0);
end

disp('Controllability Gramian Wc:');
disp(Wc);
disp('eig(Wc):');
disp(eig((Wc + Wc') / 2));
disp('det(Wc):');
disp(det(Wc));
disp('rank(Wc):');
disp(rank(Wc, 1e-8));

disp('Observability Gramian Wo:');
disp(Wo);
disp('eig(Wo):');
disp(eig((Wo + Wo') / 2));
disp('det(Wo):');
disp(det(Wo));
disp('rank(Wo):');
disp(rank(Wo, 1e-8));

% Simulink note:
% Build the LTV plant using a MATLAB Function block that outputs A(t), B(t), C(t),
% then integrate x_dot = A(t)x + B(t)u using an Integrator block. To estimate
% Gramians, log A(t), B(t), C(t), and reproduce the integration used above.

function M = Amat(t)
    M = [0.0, 1.0;
        -(2.0 + 0.4 * sin(t)), -(0.25 + 0.10 * cos(2.0 * t))];
end

function M = Bmat(t)
    M = [0.0; 1.0 + 0.25 * sin(0.5 * t)];
end

function M = Cmat(t)
    M = [1.0, 0.30 * cos(t)];
end

function dPhi = phiDerivative(t, Phi)
    dPhi = Amat(t) * Phi;
end

function PhiNext = rk4StepPhi(Phi, t, h)
    k1 = phiDerivative(t, Phi);
    k2 = phiDerivative(t + 0.5 * h, Phi + 0.5 * h * k1);
    k3 = phiDerivative(t + 0.5 * h, Phi + 0.5 * h * k2);
    k4 = phiDerivative(t + h, Phi + h * k3);
    PhiNext = Phi + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
end
