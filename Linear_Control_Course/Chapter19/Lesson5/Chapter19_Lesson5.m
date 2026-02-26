% MATLAB implementation of lead-compensated motion servo
s = tf('s');
Gm = 1 / (s * (s + 1));

alpha = 1/3;
T = sqrt(3) / 2;
Kc = 2.58;

Cle = Kc * (T * s + 1) / (alpha * T * s + 1);
L = Cle * Gm;
Tcl = feedback(L, 1);   % unity feedback

figure;
step(Tcl);
grid on;
title('Lead-compensated position servo (MATLAB)');

figure;
margin(L);              % Bode and margins
grid on;

% Simulink implementation:
% 1. Create a new model.
% 2. Place blocks: Step, Sum, Transfer Fcn (for Gm), Transfer Fcn (for Cle),
%    and Scope.
% 3. Set the plant transfer function numerator and denominator to [1]
%    and [1 1 0] corresponding to 1 / (s (s + 1)).
% 4. Set the lead compensator numerator to [Kc*T Kc] and denominator to [alpha*T 1].
% 5. Run the simulation and compare the step response with the MATLAB plot.
