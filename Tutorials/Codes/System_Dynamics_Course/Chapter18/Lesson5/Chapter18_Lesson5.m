% Chapter18_Lesson5.m
% Port-Hamiltonian mechatronic model (electrical + rotational + elastic)
clear; clc;

P.L = 0.35; P.Ra = 1.8; P.J = 0.02; P.b = 0.08; P.k = 4.0; P.Kt = 0.22;
x0 = [0;0;0];
[t,x] = ode45(@(t,x) fph(t,x,P), [0 4], x0);

phi = x(:,1); p = x(:,2); q = x(:,3);
i = phi / P.L; omega = p / P.J;
H = 0.5*phi.^2/P.L + 0.5*p.^2/P.J + 0.5*P.k*q.^2;
u = arrayfun(@uin, t);
Pin = u .* i;
Pdiss = P.Ra*i.^2 + P.b*omega.^2;

lhs = H(end)-H(1);
rhs = trapz(t, Pin-Pdiss);
fprintf('Energy residual = %.3e\n', lhs-rhs);

T = table(t,phi,p,q,H,Pin,Pdiss);
writetable(T,'Chapter18_Lesson5_matlab_results.csv');

plot(t, i, t, omega, t, q); grid on;
legend('i(t)','omega(t)','q(t)'); title('Port-Hamiltonian States');

% Simulink note: implement xdot = (J-R)gradH + G*u using Gain/Sum/Integrator blocks.

function dx = fph(t,x,P)
g = [x(1)/P.L; x(2)/P.J; P.k*x(3)];
dx = [P.Kt*g(2) - P.Ra*g(1) + uin(t);
     -P.Kt*g(1) - g(3) - P.b*g(2);
      g(2)];
end

function v = uin(t)
if t < 0.8
    v = 8.0;
elseif t < 1.6
    v = 3.0;
else
    v = 0.0;
end
end
