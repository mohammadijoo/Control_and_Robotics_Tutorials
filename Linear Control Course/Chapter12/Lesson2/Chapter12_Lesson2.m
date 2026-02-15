% First-order plant G(s) = K / (T s + 1)
K = 20;
T = 0.1;
s = tf('s');
G = K / (T*s + 1);

% Desired crossover and phase margin
omega_c_des = 40;           % rad/s
pm_des = 60;                % degrees

% Choose Ti and Td relative to plant time constant
Ti = T;          % integral time
Td = T/4;        % derivative time

% Compute Kp from the magnitude equation at omega_c_des
w = omega_c_des;
magG = abs(freqresp(G, w));
factor = sqrt(1 + (w*Td - 1/(w*Ti))^2);
Kp = sqrt(1 + (w*T)^2) / (K * factor);
Ki = Kp / Ti;
Kd = Kp * Td;

C = Kp + Ki/s + Kd*s;
L = C*G;

[gm, pm, wcg, wcp] = margin(L);
fprintf('Designed Kp=%.3f Ki=%.3f Kd=%.3f\n', Kp, Ki, Kd);
fprintf('Achieved phase margin %.2f deg at %.2f rad/s\n', pm, wcp);

figure;
margin(L); grid on; title('Loop frequency response with PID');

% In Simulink:
%  - Insert a Transfer Fcn block with numerator [K] and denominator [T 1].
%  - Insert a PID Controller block and set Kp, Ki, Kd as computed.
%  - Use the Bode plot and linear analysis tools to verify margins.
