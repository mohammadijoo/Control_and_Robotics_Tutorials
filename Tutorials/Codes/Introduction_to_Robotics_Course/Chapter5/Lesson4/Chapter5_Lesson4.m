
Ts = 2.0; w0 = 400.0; eta = 0.85;
r = 0.6; L = 0.8; E = 70e9; I = 2.5e-7; g = 9.81;

jointTorque = @(N,wj) eta*N*max(Ts*(1 - (N*wj)/w0), 0);
payloadLimit = @(N,wj) jointTorque(N,wj)/(r*g);

Ns = [20 40 80 120];
for N = Ns
    mp0 = payloadLimit(N,0);
    wjmax = w0/N;
    fprintf('N=%g mp_max(0)=%.2f kg, wj_max=%.2f rad/s\n', N, mp0, wjmax);
end

k = 3*E*I/L^3;
fprintf('k = %.3e N/m\n', k);

% In Simulink you can implement jointTorque and payloadLimit as MATLAB Function blocks,
% and sweep N to see payload-speed curves.
      