% Plant and controller
s = tf('s');
G = 10 / (s*(s + 1)*(s + 5));
K = 40;
C = K;
L = C*G;

% 1) Bode design and margins
figure;
margin(L); % Bode plus gain/phase margins

% 2) Nyquist plot
figure;
nyquist(L);

% 3) Nichols plot (open-loop Nichols)
figure;
nichols(L); grid on;

% Example: enforce a minimum phase margin and bandwidth by interactive tuning
% nichols(L) can be used with 'nicholsedit' in some releases, or use sisotool:
% sisotool(G);

% Robotics connection:
% - Use Robotics System Toolbox or Robotics Toolbox for MATLAB to build a
%   manipulator model.
% - Linearize the robot joint dynamics about a pose using LINEARIZE or linmod.
% - Use Bode/Nyquist/Nichols plots of the joint loop to design the servo gains.
