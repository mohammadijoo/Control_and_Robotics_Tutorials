% Define Laplace variable and transfer function
s = tf('s');

% Example: L(s) = 50 / (s * (s + 5)) (Type 1 system)
L = 50 / (s * (s + 5));

% Extract numerator and denominator
[num, den] = tfdata(L, 'v');  % row vectors, descending powers of s

% Count trailing zeros in denominator to estimate integrator order
tol = 1e-9;
type = 0;
for k = 1:length(den)
    if abs(den(end - k + 1)) < tol
        type = type + 1;
    else
        break;
    end
end

% Static error constants (approximate for Kv, Ka)
Kp = dcgain(L);            % Kp = L(0) for Type 0; infinite for higher types
s_eps = 1e-6;
Ls = evalfr(L, s_eps);
Kv = s_eps^1 * Ls;
Ka = s_eps^2 * Ls;

fprintf('System type: %d\n', type);
fprintf('Kp = %g, Kv ≈ %g, Ka ≈ %g\n', Kp, Kv, Ka);

% Simulink note:
% Build a unity-feedback loop with:
%   - "Step" or "Ramp" block as reference
%   - "Sum" block for error
%   - "Transfer Fcn" block with numerator num and denominator den
%   - Feedback path of gain 1
% Then measure e(t) using a "Scope" block and compare steady-state error
% against 1/(1 + Kp), 1/Kv, 1/Ka predicted here.

% Robotics note:
% For a robot arm model 'robot' from Robotics System Toolbox, use:
% linSys = linearize('yourSimulinkModel', 'IOset');
% and then apply the same logic to the SISO loop representing a given joint.
