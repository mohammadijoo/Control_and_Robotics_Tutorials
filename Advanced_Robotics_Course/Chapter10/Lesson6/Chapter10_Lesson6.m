% Assume: robot is a robotics.RigidBodyTree
%        T_bc (4x4), T_co (4x4), grasps_og cell array of 4x4 matrices

function T_bg = composeGrasp(T_bc, T_co, T_og)
    T_bg = T_bc * T_co * T_og;
end

% Example IK call
function qStar = solveIK(robot, T_bg_target)
    ik = inverseKinematics("RigidBodyTree", robot);
    weights = [1 1 1 1 1 1];
    initialGuess = robot.homeConfiguration;
    [configSol, ~] = ik(robot.EndEffector, T_bg_target, weights, initialGuess);
    qStar = [configSol.JointPosition];
end

% High-level loop (to be called from a Simulink MATLAB Function block)
function [qCmd, success] = poseGraspStep( ...
    robot, T_bc, T_co, grasps_og, maxAttempts, k)

    persistent T_co_init
    if isempty(T_co_init)
        T_co_init = eye(4);
    end

    % In practice, T_co would come from a ROS topic or sensor block
    % Here we take T_co as input to this step

    T_bg = composeGrasp(T_bc, T_co, grasps_og{1});

    qStar = solveIK(robot, T_bg);

    % Plan a simple joint-space interpolation (for illustration)
    qCurrent = [robot.homeConfiguration.JointPosition];
    nSteps = 50;
    qTrajectory = zeros(numel(qStar), nSteps);
    for i = 1:numel(qStar)
        qTrajectory(i, :) = linspace(qCurrent(i), qStar(i), nSteps);
    end

    qCmd = qTrajectory(:, end);  % last point as instantaneous command
    success = true;

    T_co_init = T_co;
end

% Sketch of Simulink integration (to be built graphically):
% 1) ROS Subscribe block for point cloud
% 2) MATLAB Function block "PoseEstimationICP" that outputs T_co
% 3) MATLAB Function block "PoseGraspStep" (above) that outputs qCmd
% 4) ROS Publish block for joint trajectory or joint targets
      
