% SEA linear simulation using state-space
Jm=0.01; Jl=0.05; bm=0.02; bl=0.05; N=50; ks=200;

% State x = [theta_m; omega_m; theta_l; omega_l]
A = [ 0 1 0 0;
     -N^2*ks/Jm -bm/Jm  N*ks/Jm 0;
      0 0 0 1;
      N*ks/Jl 0 -ks/Jl -bl/Jl];

B = [0; 1/Jm; 0; 0];
C_tau = [N*ks 0 -ks 0];  % tau_s = ks*(N*theta_m - theta_l)
D = 0;

sys = ss(A,B,C_tau,D);

t = 0:1e-3:2;
tau_d = 2*(t>0.2);

% Simple torque PID (discrete)
Kp=5; Ki=40; Kd=0.02;
x = zeros(4,1); eint=0; eprev=0;
tau_s_log=zeros(size(t));

for k=1:length(t)
    tau_s = C_tau*x;
    e = tau_d(k) - tau_s;
    eint = eint + e*1e-3;
    de = (e-eprev)/1e-3; eprev=e;
    tau_m = Kp*e + Ki*eint + Kd*de;

    xdot = A*x + B*tau_m;
    x = x + 1e-3*xdot;
    tau_s_log(k)=tau_s;
end

plot(t,tau_d,'--',t,tau_s_log,'LineWidth',1.2);
xlabel('Time (s)'); ylabel('Torque (N*m)'); grid on;
legend('Desired','SEA torque');
