% Closed-loop model T0(s): underdamped second-order
wn   = 10;    % rad/s
zeta = 0.2;

s = tf('s');
T0 = wn^2 / (s^2 + 2*zeta*wn*s + wn^2);

% Desired critically damped model M(s)
M = wn^2 / (s + wn)^2;

% Model-matching pre-filter
F_match = minreal(M / T0);

% First-order low-pass pre-filter
tau_f = 0.1;
F_lp  = 1 / (tau_f*s + 1);

% Tracking transfer functions
T_match = minreal(F_match * T0);
T_lp    = minreal(F_lp * T0);

figure;
step(T0, T_match, T_lp);
legend('T0(s)', 'F\_match*T0(s)', 'F\_lp*T0(s)');
grid on;
title('Reference Pre-Filters: MATLAB Simulation');
xlabel('Time [s]');
ylabel('Output y(t)');

% Simulink implementation sketch:
%   - Place Transfer Fcn block with numerator/denominator of F_match or F_lp
%   - Feed reference signal into this block
%   - Output of pre-filter goes to summing junction of existing feedback loop
