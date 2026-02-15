% Define a 2-DOF planar arm using Robotics System Toolbox
L1 = 1.0; L2 = 0.7;
robot = rigidBodyTree('DataFormat','row','MaxNumBodies',3);
body1 = rigidBody('link1');
jnt1  = rigidBodyJoint('joint1','revolute');
setFixedTransform(jnt1,trvec2tform([0 0 0]));
body1.Joint = jnt1;
addBody(robot,body1,'base');

body2 = rigidBody('link2');
jnt2  = rigidBodyJoint('joint2','revolute');
setFixedTransform(jnt2,trvec2tform([L1 0 0]));
body2.Joint = jnt2;
addBody(robot,body2,'link1');

% Task generator: random end-effector poses within workspace
numTasks = 100;
qStart = zeros(numTasks,2);
qGoal  = zeros(numTasks,2);
difficulty = zeros(numTasks,1);

ik = inverseKinematics('RigidBodyTree',robot);
weights = [1 1 0 0 0 0];
endEff = 'link2';

for i = 1:numTasks
    % sample reachable XY position
    r = (L1 + L2) * 0.8 * sqrt(rand());
    theta = 2*pi*rand();
    targetPos = [r*cos(theta), r*sin(theta), 0];

    % start and goal poses
    Tstart = trvec2tform([0 0 0]);
    Tgoal  = trvec2tform(targetPos);

    [qStart(i,:), ~] = ik(endEff,Tstart,weights,robot.homeConfiguration);
    [qGoal(i,:),  ~] = ik(endEff,Tgoal, weights,robot.homeConfiguration);

    difficulty(i) = norm(qGoal(i,:) - qStart(i,:)); % joint distance
end

% Stratify tasks by difficulty tertiles for later benchmark design
edges = quantile(difficulty,[0 1/3 2/3 1]);
stratumId = discretize(difficulty, edges);

taskDataset = table(qStart, qGoal, difficulty, stratumId);
disp(head(taskDataset));
      
