% Discrete-time linear model x_{k+1} = A x_k + B u_k, y_k = C x_k
A = [1.0 0.1; 0 1.0];
B = [0.0; 0.1];
C = [1 0];
Ts = 0.1;

sys = ss(A, B, C, 0, Ts);

% Prediction horizon and control horizon
Hp = 40;   % prediction length (long horizon)
Hc = 10;   % control horizon

mpcobj = mpc(sys, Ts, Hp, Hc);

% Weights: position tracking vs control effort
mpcobj.Weights.OutputVariables = 1;
mpcobj.Weights.ManipulatedVariablesRate = 0.01;

% Constraints on control input
mpcobj.MV.Min = -1;
mpcobj.MV.Max =  1;

% Reference trajectory (long-horizon step to x = 10)
Tfinal = 20;
r = ones(Tfinal / Ts, 1) * 10;  % desired position

% Simulate in MATLAB
x0 = [0; 0];
mpcstate = mpcstate(mpcobj);
x = x0;
Y = [];
U = [];
for k = 1:length(r)
    y = C * x;
    u = mpcmove(mpcobj, mpcstate, y, r(k));
    x = A * x + B * u;
    Y(end+1,1) = y; %#ok<AGROW>
    U(end+1,1) = u; %#ok<AGROW>
end

% Plot results
figure;
subplot(2,1,1);
plot(0:Ts:(length(Y)-1)*Ts, Y, 'LineWidth', 1.5);
hold on; plot(0:Ts:(length(r)-1)*Ts, r, '--');
xlabel('Time [s]'); ylabel('Position');
title('Long-horizon tracking with MPC');

subplot(2,1,2);
plot(0:Ts:(length(U)-1)*Ts, U, 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Control input');
title('MPC control effort');

% The corresponding Simulink model can include:
% - "State-Space" block with matrices A, B, C, 0
% - "MPC Controller" block using mpcobj
% - Measurement noise and reference generator blocks
      
