% First-order low-pass for tremor suppression
dt = 0.005; tau = 0.08;
alpha = dt/(tau+dt);

x_m = [0 0.1 0.3 0.2 0.5 0.4];
y = zeros(size(x_m));
for k = 2:length(x_m)
    y(k) = alpha*x_m(k) + (1-alpha)*y(k-1);
end
disp(y);

% Continuous-time view
s = tf('s');
H = 1/(1 + s*tau);
G = 5/((s+2)*(s+8));  % example slave model
CL = feedback(H*G, 1); % filtered tracking
step(CL);
      