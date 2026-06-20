
% MinimalRobot in MATLAB
x_r = 0;         % robot state
x_e = 10;        % environment target
dt  = 0.1;
alpha = 0.2;

for k = 1:50
    y = x_e - x_r;           % sensing: y = h(x_r, x_e)
    phi = 0.5 * y;           % internal reactive rule
    u = alpha*0 + (1-alpha)*phi;  % blended autonomy
    x_r = x_r + u*dt;        % actuation/body dynamics
end
disp(x_r)

% Simulink idea:
% Use blocks: Sum (x_e - x_r) -> Gain (0.5) -> Blend -> Integrator (x_r_dot=u)
      