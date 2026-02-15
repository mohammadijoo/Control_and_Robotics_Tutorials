
function lab_adaptive_vs_fixed
  theta_true = [2.0; 0.4; 5.0];
  theta_nom  = [1.5; 0.2; 4.0];
  lam = 5.0; k_s = 10.0;
  Gamma = diag([5.0 5.0 5.0]);

  x0_fixed = [0; 0];                % [q; dq]
  x0_adapt = [0; 0; theta_nom];     % [q; dq; th1_hat; th2_hat; th3_hat]

  tspan = [0 20];

  [tF, xF] = ode45(@(t,x) rhs_fixed(t,x,theta_true,theta_nom,lam,k_s), tspan, x0_fixed);
  [tA, xA] = ode45(@(t,x) rhs_adapt(t,x,theta_true,Gamma,lam,k_s),      tspan, x0_adapt);

  qF = xF(:,1);
  qA = xA(:,1);

  [qdF, ~, ~] = desired_traj(tF);
  [qdA, ~, ~] = desired_traj(tA);

  eF = qF - qdF;
  eA = qA - qdA;

  figure; plot(tF, eF, tA, eA);
  xlabel('t'); ylabel('e');
  legend('fixed-model','adaptive');
end

function [qd, dqd, ddqd] = desired_traj(t)
  A = 0.5; omega = 1.0;
  qd   = A .* sin(omega .* t);
  dqd  = A .* omega .* cos(omega .* t);
  ddqd = -A .* omega.^2 .* sin(omega .* t);
end

function dx = rhs_fixed(t,x,theta_true,theta_nom,lam,k_s)
  q  = x(1); dq = x(2);
  [qd,dqd,ddqd] = desired_traj(t);
  e  = q - qd;
  de = dq - dqd;
  dq_r  = dqd - lam * e;
  ddq_r = ddqd - lam * de;
  s   = dq - dq_r;
  Y   = [ddq_r; dq_r; sin(q)];
  tau = Y' * theta_nom - k_s * s;
  dx  = [dq; plant_accel(theta_true,q,dq,tau)];
end

function dx = rhs_adapt(t,x,theta_true,Gamma,lam,k_s)
  q  = x(1); dq = x(2);
  th_hat = x(3:5);
  [qd,dqd,ddqd] = desired_traj(t);
  e  = q - qd;
  de = dq - dqd;
  dq_r  = dqd - lam * e;
  ddq_r = ddqd - lam * de;
  s   = dq - dq_r;
  Y   = [ddq_r; dq_r; sin(q)];
  tau = Y' * th_hat - k_s * s;
  ddq = plant_accel(theta_true,q,dq,tau);
  dth = -Gamma * (Y * s);
  dx  = [dq; ddq; dth];
end

function ddq = plant_accel(theta,q,dq,tau)
  theta1 = theta(1); theta2 = theta(2); theta3 = theta(3);
  ddq = (tau - theta2 * dq - theta3 * sin(q)) / theta1;
end
