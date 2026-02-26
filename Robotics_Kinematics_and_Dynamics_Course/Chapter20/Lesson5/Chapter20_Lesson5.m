% Nominal parameters
n = 6;                    % DOF
m_nom = 5 * ones(n,1);    % nominal link masses

% Uncertain masses (interval +-20 percent)
m_unc = ureal('m_unc', m_nom(1), 'Percentage', 20);
% For simplicity, treat all links as having the same uncertain mass here.

% Example state
q  = zeros(n,1);
dq = zeros(n,1);
ddq = ones(n,1);

% Placeholder nominal inverse dynamics (to be replaced by your Lagrange / Newton-Euler code)
function tau = invdyn_nominal(q, dq, ddq, m_val)
    n = numel(q);
    M = eye(n) * m_val;    % crude mass matrix
    h = zeros(n,1);
    tau = M * ddq + h;
end

tau_samples = [];
for k = 1:100
    % Sample an uncertain mass value
    m_sample = usample(m_unc);
    tau_k = invdyn_nominal(q, dq, ddq, m_sample);
    tau_samples = [tau_samples, tau_k];
end

% Analyze spread of torques due to uncertainty
tau_mean = mean(tau_samples, 2);
tau_std  = std(tau_samples, 0, 2);
disp(tau_mean);
disp(tau_std);
      
