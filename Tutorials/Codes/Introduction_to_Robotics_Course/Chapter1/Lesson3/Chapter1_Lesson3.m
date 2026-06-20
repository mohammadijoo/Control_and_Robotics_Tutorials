
% Shared autonomy for x_dot = u
x = 2.0; k = 1.5; alpha = 0.7;
T = 5.0; dt = 0.01;
nSteps = floor(T/dt);

xs = zeros(nSteps+1,1); xs(1)=x;

for i=1:nSteps
    t  = (i-1)*dt;
    uA = -k*x;
    uH = 0.5*sin(2*pi*t);  % bounded human command
    u  = alpha*uA + (1-alpha)*uH;

    x  = x + dt*u;         % plant integration
    xs(i+1)=x;
end

disp(['Final state: ', num2str(xs(end))]);

% Simulink note:
% Use blocks: Sum, Gain(-k), Sine Wave, Weighted Sum(alpha),
% Integrator for x_dot = u.
      