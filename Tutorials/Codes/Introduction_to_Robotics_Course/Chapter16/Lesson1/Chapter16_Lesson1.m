% Plant and controller (example)
s = tf('s');
G = 1 / (0.1 * s + 1);          % simple first-order plant
Kp = 5;
C = Kp;                         % proportional controller

T_cl = feedback(C * G, 1);      % closed-loop transfer function

% Step response info
info = stepinfo(T_cl);

Mp_allowed = 10;    % percent
Ts_allowed = 0.5;   % seconds

req_overshoot = (info.Overshoot <= Mp_allowed);
req_settling  = (info.SettlingTime <= Ts_allowed);

if req_overshoot && req_settling
    disp('Requirements satisfied: overshoot and settling time within bounds.');
else
    disp('Requirements violated:');
    fprintf('  Overshoot = %.2f%% (allowed %.2f%%)\n', info.Overshoot, Mp_allowed);
    fprintf('  Settling time = %.3f s (allowed %.3f s)\n', info.SettlingTime, Ts_allowed);
end
      
