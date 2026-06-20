
Ad = [0.95  0.01;
      -0.2  0.90];
Ts = 0.005;

sysd = ss(Ad, [], eye(2), [], Ts);
z = eig(Ad);

disp('Discrete poles:');
disp(z);

if all(abs(z) < 1)
    disp('System is Schur-stable.');
else
    disp('System is NOT Schur-stable.');
end

% Example: from continuous-time PD design for a joint
J = 0.5; b = 0.1;
Kp = 25; Kd = 4;

Ac = [0  1;
      -Kp/J  -(Kd + b)/J];

Tc = 0.005;
sysc = ss(Ac, [], eye(2), []);
sysd_pd = c2d(sysc, Tc, 'zoh');
eig(sysd_pd)   % discrete-time poles of PD controlled joint

% In Simulink, one uses Discrete-Time blocks with Ts and scopes to
% visualize settling time and overshoot of joint position.
