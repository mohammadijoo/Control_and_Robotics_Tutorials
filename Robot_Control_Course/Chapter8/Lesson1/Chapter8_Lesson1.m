
function adaptive_joint_1dof
  % True parameters
  m_true = 2.0; b_true = 0.4; g0_true = 5.0;

  % Initial parameter estimates
  theta_hat0 = [1.0; 0.2; 3.0];  % [m_hat; b_hat; g0_hat]
  x0 = [0; 0; theta_hat0];       % [q; dq; theta_hat]

  Tspan = [0 5];
  [T,X] = ode45(@(t,x) dynamics(t,x,m_true,b_true,g0_true), Tspan, x0);

  q = X(:,1);
  theta_hat = X(:,3:5);

  figure; subplot(2,1,1);
  plot(T,q); ylabel('q(t)');
  subplot(2,1,2);
  plot(T,theta_hat); ylabel('theta\_hat'); xlabel('t');

  function dx = dynamics(t,x,m_true,b_true,g0_true)
      q = x(1); dq = x(2);
      theta_hat = x(3:5);
      m_hat = theta_hat(1); b_hat = theta_hat(2); g0_hat = theta_hat(3);

      [q_d, dq_d, ddq_d] = desired_traj(t);
      e = q - q_d; de = dq - dq_d;
      k_p = 20; k_d = 8;

      ddq_r = ddq_d - k_d*de - k_p*e;
      dq_r = dq_d - k_p*e;
      Y = [ddq_r, dq_r, sin(q)];
      tau_hat = Y * theta_hat;
      tau = tau_hat;

      ddq = (tau - b_true*dq - g0_true*sin(q)) / m_true;

      gamma = 0.1;
      error = tau - tau_hat;
      theta_hat_dot = -gamma * (Y.' * error);

      dx = [dq; ddq; theta_hat_dot];
  end

  function [q_d, dq_d, ddq_d] = desired_traj(t)
      q_d = 0.5 * sin(0.5 * t);
      dq_d = 0.25 * cos(0.5 * t);
      ddq_d = -0.125 * sin(0.5 * t);
  end
end
