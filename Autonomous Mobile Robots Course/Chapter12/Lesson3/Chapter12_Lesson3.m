% Chapter12_Lesson3.m
% Pose Graph Optimization in SE(2) via Gauss-Newton (MATLAB, sparse)
%
% Run:
%   Chapter12_Lesson3
%
% Notes:
% - Educational from-scratch implementation using sparse normal equations.
% - Simulink note: you can wrap the "one GN step" into a MATLAB Function block
%   to co-simulate optimization as part of a SLAM pipeline.

function Chapter12_Lesson3()
  [X0, edges] = make_synthetic_pose_graph(30);
  disp('Optimizing pose graph (Gauss-Newton, anchor pose 1)...');

  X = gauss_newton(X0, edges, 15);

  disp('First 5 poses (x,y,theta) initial vs optimized:');
  for k = 1:5
    fprintf('k=%02d  X0=[%.4f %.4f %.4f]  Xopt=[%.4f %.4f %.4f]\n', ...
      k-1, X0(k,1), X0(k,2), X0(k,3), X(k,1), X(k,2), X(k,3));
  end

  % Optional: programmatically create a tiny Simulink skeleton (no data files).
  % create_simulink_skeleton();
end

% -------------------- Core solver --------------------

function X = gauss_newton(X0, edges, iters)
  X = X0;
  for it = 1:iters
    [H, b, chi2] = build_normal_equations(X, edges);
    dx = -H \ b;   % sparse solve

    X = apply_increment(X, dx);

    fprintf('iter=%02d  chi2=%.6f  |dx|=%.3e\n', it-1, chi2, norm(dx));
    if norm(dx) < 1e-8
      break;
    end
  end
end

function [H, b, chi2] = build_normal_equations(X, edges)
  % Anchor pose 1 (MATLAB 1-index) => variables are poses 2..N
  N = size(X,1);
  dim = 3*(N-1);

  H = sparse(dim, dim);
  b = zeros(dim, 1);
  chi2 = 0.0;

  for k = 1:length(edges)
    e = edges(k);
    xi = X(e.i, :).';
    xj = X(e.j, :).';
    z  = e.z(:);

    r = se2_error(xi, xj, z);
    [A, Bm] = se2_jacobians(xi, xj, z);

    Omega = e.Omega;
    chi2 = chi2 + r.' * Omega * r;

    AtOmega = A.' * Omega;
    BtOmega = Bm.' * Omega;

    Hii = AtOmega * A;
    Hij = AtOmega * Bm;
    Hji = BtOmega * A;
    Hjj = BtOmega * Bm;

    bi = AtOmega * r;
    bj = BtOmega * r;

    % scatter, skipping anchored node i=1
    addBlock(e.i, e.i, Hii);
    addBlock(e.i, e.j, Hij);
    addBlock(e.j, e.i, Hji);
    addBlock(e.j, e.j, Hjj);

    if e.i ~= 1
      ii = idx(e.i);
      b(ii:ii+2) = b(ii:ii+2) + bi;
    end
    if e.j ~= 1
      jj = idx(e.j);
      b(jj:jj+2) = b(jj:jj+2) + bj;
    end
  end

  function base = idx(pid)
    base = 3*(pid-2) + 1; % pose 2 -> 1
  end

  function addBlock(p, q, M)
    if (p == 1) || (q == 1), return; end
    ip = idx(p);
    iq = idx(q);
    H(ip:ip+2, iq:iq+2) = H(ip:ip+2, iq:iq+2) + M;
  end
end

function X = apply_increment(X, dx)
  N = size(X,1);
  for k = 2:N
    d = dx(3*(k-2)+1 : 3*(k-2)+3);
    X(k,1) = X(k,1) + d(1);
    X(k,2) = X(k,2) + d(2);
    X(k,3) = wrap_angle(X(k,3) + d(3));
  end
end

% -------------------- SE(2) utilities --------------------

function a = wrap_angle(a)
  a = mod(a + pi, 2*pi) - pi;
end

function R = rot2(th)
  c = cos(th); s = sin(th);
  R = [c -s; s c];
end

function zhat = predict_relative(xi, xj)
  ti = xi(1:2);
  tj = xj(1:2);
  Ri = rot2(xi(3));
  dt = Ri.' * (tj - ti);
  dth = wrap_angle(xj(3) - xi(3));
  zhat = [dt; dth];
end

function e = se2_error(xi, xj, z)
  zhat = predict_relative(xi, xj);
  Rz = rot2(z(3));
  terr = Rz.' * (zhat(1:2) - z(1:2));
  therr = wrap_angle(zhat(3) - z(3));
  e = [terr; therr];
end

function [A, Bm] = se2_jacobians(xi, xj, z)
  ti = xi(1:2);
  tj = xj(1:2);
  Ri = rot2(xi(3));
  Rz = rot2(z(3));
  A2 = Rz.' * Ri.';

  S = [0 -1; 1 0];
  dt = (tj - ti);

  A = zeros(3,3);
  Bm = zeros(3,3);

  A(1:2,1:2) = -A2;
  Bm(1:2,1:2) =  A2;

  dti_dtheta = Rz.' * (-Ri.' * (S * dt));
  A(1:2,3) = dti_dtheta;

  A(3,3) = -1;
  Bm(3,3) =  1;
end

% -------------------- Synthetic graph --------------------

function [X0, edges] = make_synthetic_pose_graph(N)
  Xtrue = zeros(N,3);
  for k = 2:N
    Xtrue(k,1) = Xtrue(k-1,1) + 0.5*cos(0.1*k);
    Xtrue(k,2) = Xtrue(k-1,2) + 0.5*sin(0.1*k);
    Xtrue(k,3) = wrap_angle(0.05*k);
  end

  noise_xy = 0.02;
  noise_th = 0.01;
  Sigma = diag([noise_xy^2, noise_xy^2, noise_th^2]);
  Omega = inv(Sigma);

  edges = struct('i', {}, 'j', {}, 'z', {}, 'Omega', {});
  rng(7);

  % odometry edges
  for k = 1:N-1
    z = predict_relative(Xtrue(k,:).', Xtrue(k+1,:).');
    z = z + [noise_xy; noise_xy; noise_th] .* randn(3,1);
    z(3) = wrap_angle(z(3));
    edges(end+1) = struct('i', k, 'j', k+1, 'z', z, 'Omega', Omega); %#ok<AGROW>
  end

  % loop closure 1->N
  zlc = predict_relative(Xtrue(1,:).', Xtrue(N,:).');
  zlc = zlc + [noise_xy; noise_xy; noise_th] .* randn(3,1);
  zlc(3) = wrap_angle(zlc(3));
  edges(end+1) = struct('i', 1, 'j', N, 'z', zlc, 'Omega', Omega);

  % initial guess: integrate noisy odometry only
  X0 = zeros(N,3);
  for k = 1:N-1
    z = edges(k).z;
    Rk = rot2(X0(k,3));
    X0(k+1,1:2) = X0(k,1:2) + (Rk * z(1:2)).';
    X0(k+1,3) = wrap_angle(X0(k,3) + z(3));
  end
end

% -------------------- Simulink skeleton (optional) --------------------
function create_simulink_skeleton()
  mdl = 'PoseGraphGN_Skeleton';
  if bdIsLoaded(mdl), close_system(mdl, 0); end
  new_system(mdl);
  open_system(mdl);

  add_block('simulink/Sources/Constant', [mdl '/edges_placeholder'], 'Value', '0');
  add_block('simulink/Sources/Constant', [mdl '/X0_placeholder'], 'Value', '0');

  add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/GN_step']);
  set_param([mdl '/GN_step'], 'Script', sprintf([ ...
    'function X = GN_step(X0, edges)\n' ...
    '%% Replace placeholders with real data; call gauss_newton for one iteration.\n' ...
    'X = X0; %% demo only\n' ...
    'end\n']));

  add_block('simulink/Sinks/Display', [mdl '/Display']);

  add_line(mdl, 'X0_placeholder/1', 'GN_step/1');
  add_line(mdl, 'edges_placeholder/1', 'GN_step/2');
  add_line(mdl, 'GN_step/1', 'Display/1');

  save_system(mdl);
end
