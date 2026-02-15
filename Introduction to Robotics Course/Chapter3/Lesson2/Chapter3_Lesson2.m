% Moving-base arm on cart (ode45)
I=0.2; M=1.0; m=0.5; ell=0.6; g=9.81;
Kp=15; Kd=3;

f = @(t,s) dyn(t,s,I,M,m,ell,g,Kp,Kd);

tspan = [0 5];
s0 = [0;0;0.5;0]; % [x; xdot; q; qdot]
sol = ode45(f, tspan, s0);

xf = sol.y(1,end);
qf = sol.y(3,end);
disp(['Final q=', num2str(qf), ' Final x=', num2str(xf)]);

function ds = dyn(~, s, I, M, m, ell, g, Kp, Kd)
  x=s(1); xdot=s(2); q=s(3); qdot=s(4);

  tau = -Kp*q - Kd*qdot;
  ub = 0; % unactuated base

  M11 = M + m;
  M12 = m*ell*cos(q);
  M22 = I + m*ell^2;

  Mass = [M11 M12; M12 M22];

  h1 = -m*ell*sin(q)*qdot^2;
  h2 =  m*g*ell*sin(q);

  acc = Mass \ [ub - h1; tau - h2];
  xddot = acc(1);
  qddot = acc(2);

  ds = [xdot; xddot; qdot; qddot];
end
      