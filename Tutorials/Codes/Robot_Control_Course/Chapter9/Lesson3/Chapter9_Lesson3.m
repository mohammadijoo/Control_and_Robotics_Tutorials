
function [X, U] = ilqr_pendulum_matlab(x0, N, maxIter)
  m = 1.0; l = 1.0; g = 9.81; dt = 0.01;
  Q = diag([10, 1]);
  R = 0.1;
  P_N = diag([50, 5]);

  X = zeros(2, N+1);
  U = zeros(1, N);
  X(:,1) = x0(:);
  xref = zeros(2, N+1);

  % initial rollout
  for k = 1:N
    X(:,k+1) = dynamics(X(:,k), U(k), m, l, g, dt);
  end

  for it = 1:maxIter
    Vx = P_N * (X(:,N+1) - xref(:,N+1));
    Vxx = P_N;

    K = zeros(1, 2, N);
    k_ff = zeros(1, N);
    lambda = 1e-6;

    for k = N:-1:1
      xk = X(:,k);
      uk = U(k);
      xr = xref(:,k);

      [fx, fu] = linearize_dynamics(xk, uk, m, l, g, dt);

      dx = xk - xr;
      lx = Q * dx;
      lu = R * uk;
      lxx = Q;
      luu = R;

      Qx = lx + fx' * Vx;
      Qu = lu + fu' * Vx;
      Qxx = lxx + fx' * Vxx * fx;
      Quu = luu + fu' * Vxx * fu + lambda;
      Qux = fu' * Vxx * fx;

      k_local = -Quu \ Qu;
      K_local = -Quu \ Qux;

      k_ff(k) = k_local;
      K(1,:,k) = K_local;

      Vx = Qx + K_local' * Quu * k_local + K_local' * Qu + Qux' * k_local;
      Vxx = Qxx + K_local' * Quu * K_local + K_local' * Qux + Qux' * K_local;
      Vxx = 0.5 * (Vxx + Vxx');
    end

    % forward pass
    Xnew = zeros(size(X));
    Unew = zeros(size(U));
    Xnew(:,1) = x0(:);

    alpha_list = [1.0, 0.5, 0.25, 0.1];
    Jold = cost_total(X, U, xref, Q, R, P_N, N);

    improved = false;
    for a = alpha_list
      for k = 1:N
        du = a * k_ff(k) + squeeze(K(1,:,k)) * (Xnew(:,k) - X(:,k));
        Unew(k) = U(k) + du;
        Xnew(:,k+1) = dynamics(Xnew(:,k), Unew(k), m, l, g, dt);
      end
      Jnew = cost_total(Xnew, Unew, xref, Q, R, P_N, N);
      if Jnew < Jold
        X = Xnew; U = Unew;
        improved = true;
        break;
      end
    end

    if ~improved
      break;
    end
  end
end

function xnext = dynamics(x, u, m, l, g, dt)
  theta = x(1); dtheta = x(2);
  ddtheta = (-g / l) * sin(theta) + u / (m * l^2);
  xnext = [theta + dt * dtheta;
           dtheta + dt * ddtheta];
end

function [fx, fu] = linearize_dynamics(x, u, m, l, g, dt)
  theta = x(1);
  ddtheta_dtheta = (-g / l) * cos(theta);
  ddtheta_du = 1.0 / (m * l^2);

  fx = eye(2);
  fx(1,2) = dt;
  fx(2,1) = dt * ddtheta_dtheta;
  fx(2,2) = 1.0;

  fu = [0; dt * ddtheta_du];
end

function J = cost_total(X, U, xref, Q, R, P_N, N)
  J = 0;
  for k = 1:N
    dx = X(:,k) - xref(:,k);
    J = J + 0.5 * dx' * Q * dx + 0.5 * R * U(k)^2;
  end
  dxN = X(:,N+1) - xref(:,N+1);
  J = J + 0.5 * dxN' * P_N * dxN;
end
