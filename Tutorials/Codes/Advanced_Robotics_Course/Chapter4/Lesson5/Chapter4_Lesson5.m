% Define a 2-DOF planar manipulator in MATLAB
robot = rigidBodyTree("DataFormat","row","MaxNumBodies",3);

L1 = 0.7; L2 = 0.7;

body1 = rigidBody("link1");
joint1 = rigidBodyJoint("joint1","revolute");
setFixedTransform(joint1,trvec2tform([0 0 0]));
body1.Joint = joint1;
addBody(robot,body1,"base");

body2 = rigidBody("link2");
joint2 = rigidBodyJoint("joint2","revolute");
setFixedTransform(joint2,axang2tform([0 0 1 0]) * trvec2tform([L1 0 0]));
body2.Joint = joint2;
addBody(robot,body2,"link1");

tool = rigidBody("tool");
setFixedTransform(tool.Joint,trvec2tform([L2 0 0]));
addBody(robot,tool,"link2");

% Obstacle as collision object
obs = collisionSphere(0.25);
obs.Pose = trvec2tform([0.5 0 0]);

% Trajectory discretization
N = 40;
qStart = [-0.5 1.0];
qGoal  = [ 1.0 -0.5];
q = zeros(N+1,2);
for k = 1:(N+1)
    alpha = (k-1)/N;
    q(k,:) = (1-alpha)*qStart + alpha*qGoal;
end

lambdaSmooth = 1.0;
lambdaCol = 10.0;
alphaStep = 0.01;
clearance = 0.05;

for it = 1:200
    grad = zeros(size(q));
    J_s = 0;
    J_c = 0;

    % Smoothness term
    for k = 2:N
        ddq = q(k+1,:) - 2*q(k,:) + q(k-1,:);
        J_s = J_s + 0.5 * ddq * ddq.';
        grad(k-1,:) = grad(k-1,:) - ddq;
        grad(k,:)   = grad(k,:)   + 2*ddq;
        grad(k+1,:) = grad(k+1,:) - ddq;
    end

    % Collision term
    for k = 2:N
        config = q(k,:);
        [isColliding,sepDist,~,~] = checkCollision(robot,config,obs, ...
            "IgnoreSelfCollision","on","Exhaustive","off");
        d = sepDist;  % signed distance; < 0 if penetrating

        if d < clearance
            phi = 0.5*(clearance - d)^2;
            J_c = J_c + phi;

            % Approximate gradient using numerical differentiation on joint space
            epsFD = 1e-5;
            gq = zeros(1,2);
            for j = 1:2
                dq = zeros(1,2); dq(j) = epsFD;
                [~,dPlus]  = checkCollision(robot,config + dq,obs);
                [~,dMinus] = checkCollision(robot,config - dq,obs);
                dd_dq = (dPlus - dMinus) / (2*epsFD);
                gq(j) = -(clearance - d) * dd_dq;
            end
            grad(k,:) = grad(k,:) + gq;
        end
    end

    % Gradient step on internal nodes
    for k = 2:N
        q(k,:) = q(k,:) - alphaStep * (lambdaSmooth * grad(k,:) + lambdaCol * grad(k,:));
    end

    % Fix boundary conditions
    q(1,:)   = qStart;
    q(end,:) = qGoal;

    if mod(it,20) == 0
        fprintf("Iter %d: J_s = %.4f, J_c = %.4f\n",it,J_s,J_c);
    end
end
      
