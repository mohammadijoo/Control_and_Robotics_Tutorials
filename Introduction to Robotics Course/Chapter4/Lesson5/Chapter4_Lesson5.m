
Kt = 0.08;   % N*m/A
Ke = 0.08;   % V*s/rad
R  = 0.6;    % Ohm
Vmax = 24;   % V

w = linspace(0, 300, 200); % rad/s
M = (Kt/R) * (Vmax - Ke*w); % torque line

figure; plot(w, M);
xlabel('omega (rad/s)'); ylabel('Motor torque (N*m)');
title('Speed–torque limit from datasheet constants');
grid on;
      