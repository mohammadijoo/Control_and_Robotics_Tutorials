
% Assume rigidBodyTree object 'humanoid' from Robotics System Toolbox
% and kinematics helpers have been defined.
function qdd_star = wholeBodyQP_step(humanoid, q, qd, supportBody, swingBody)

  % Compute dynamics
  [M, C, G] = manipulatorDynamics(humanoid, q, qd);
  h = C * qd.' + G;

  % CoM Jacobian and position
  Jcom = centerOfMassJacobian(humanoid, q);
  pcom = centerOfMass(humanoid, q);

  pcomRef = [0, 0, 0.8];
  Kp_com = 50; Kd_com = 10;

  vcom = Jcom * qd.';
  comErr = pcomRef - pcom;
  comVelErr = -vcom;
  comAccDes = Kp_com * comErr + Kd_com * comVelErr;
  JdotComQd = zeros(1, size(Jcom, 2)); % placeholder

  Acom = Jcom;
  bcom = comAccDes - JdotComQd;

  % Swing foot task
  swingId = bodyIndex(humanoid, swingBody);
  [Tswing, Jswing] = forwardKinematics(humanoid, q, swingId);
  xswing = tform2trvec(Tswing);
  vswing = Jswing * qd.';

  swingRef = [0.1, 0.1, 0.4];
  Kp_s = 150; Kd_s = 20;
  xErr = swingRef - xswing;
  vErr = -vswing;
  accSwingDes = Kp_s * xErr + Kd_s * vErr;
  JdotSwingQd = zeros(1, size(Jswing, 2)); % placeholder

  Aswing = Jswing(1:3, :);
  bswing = accSwingDes - JdotSwingQd(1:3);

  % Posture task
  q0 = q; % nominal posture
  Kp_q = 20; Kd_q = 5;
  qErr = q0 - q;
  qdErr = -qd;
  accPostDes = Kp_q * qErr + Kd_q * qdErr;

  Apost = eye(numel(q));
  bpost = accPostDes;

  % Stack tasks
  w_com = 1.0; w_swing = 1.0; w_post = 0.1; w_reg = 1e-4;

  At = [sqrt(w_com) * Acom;
        sqrt(w_swing) * Aswing;
        sqrt(w_post) * Apost];

  bt = [sqrt(w_com) * bcom.';
        sqrt(w_swing) * bswing.';
        sqrt(w_post) * bpost.'];

  H = At.' * At + w_reg * eye(size(At, 2));
  f = -At.' * bt;

  % Contact acceleration constraint (holonomic, stance foot)
  supportId = bodyIndex(humanoid, supportBody);
  [Tsup, Jsup] = forwardKinematics(humanoid, q, supportId);
  Jc = Jsup(1:3, :);
  JcdotQd = zeros(3, 1); % placeholder

  Aeq = Jc;
  beq = -JcdotQd;

  % Acceleration bounds
  n = numel(q);
  qddMax = 20 * ones(n, 1);
  qddMin = -20 * ones(n, 1);
  Aineq = [eye(n); -eye(n)];
  bineq = [qddMax; -qddMin];

  opts = optimoptions('quadprog', 'Display', 'off');
  qdd_star = quadprog(H, f, Aineq, bineq, Aeq, beq, [], [], [], opts).';
end
