Ts   = 0.002;
tauf = 0.01;

s = tf('s');
F   = 1 / (1 + tauf*s);      % continuous-time filter
Fd  = c2d(F, Ts, 'tustin');  % discrete-time version

% Bode and step responses to check magnitude and phase effects
figure;
bode(F, Fd);
legend('F(s)', 'F(z)');

% In Simulink:
% - Insert a "Transfer Fcn" block with numerator [1], denominator [tauf 1]
%   in the measurement path for continuous simulation.
% - For code generation and hardware deployment, use "Discrete Transfer Fcn"
%   and set its numerator and denominator to the coefficients of Fd.
