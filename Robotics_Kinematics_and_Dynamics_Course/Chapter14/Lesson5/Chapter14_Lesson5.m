% Pendulum parameters as uncertain reals (10% relative uncertainty)
I = ureal('I', 0.05, 'Percentage', 10);
b = ureal('b', 0.01, 'Percentage', 20);
k = ureal('k', 0.5,  'Percentage', 10);

% Regressor at a given state (q, dq, ddq)
q   = deg2rad(30);
dq  = 0.5;
ddq = 1.0;
Y   = [ddq, dq, sin(q)];

theta_hat = [0.05; 0.01; 0.5];
tau_hat   = Y * theta_hat;

% Uncertain parameter vector
theta_unc = [I; b; k];
tau_unc   = Y * theta_unc;   % This is an uncertain scalar (uss)

% Sample the uncertain parameters and compute tau
Ns = 1000;
samples = usample(theta_unc, Ns);  % Ns-by-1 struct of samples
tau_samples = zeros(Ns, 1);
for i = 1:Ns
    th = [samples(i).I; samples(i).b; samples(i).k];
    tau_samples(i) = Y * th;
end

max_err = max(abs(tau_samples - tau_hat));
fprintf('Max |tau - tau_hat| over samples: %g\n', max_err);

% Simulink modeling idea (conceptual):
%  - Build a Simulink block diagram for q, dq, ddq dynamics.
%  - Implement the dynamics I*ddq + b*dq + k*sin(q) = tau using Gain and Sum blocks.
%  - Use the Uncertain State Space blocks or mask the gains with uncertain
%    parameters I, b, k to perform worst-case and Monte Carlo simulations.
      
