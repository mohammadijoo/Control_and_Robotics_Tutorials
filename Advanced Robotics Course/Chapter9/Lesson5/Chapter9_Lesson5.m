function hybridPlan = tamp_pick_and_place(world, skeleton, qInit, objectPoseInit, goalRegionPose)
% world: struct with handles for collision, kinematics, etc.

qCurrent = qInit;
objectPose = objectPoseInit;
hybridPlan = {};

for k = 1:numel(skeleton)
    action = skeleton{k};
    name = action.name;
    params = action.params;

    switch name
        case 'move'
            qGoal = named_config(params{2});
            path = rrt_connect_matlab(world, qCurrent, qGoal, 'transit', objectPose);
            if isempty(path)
                hybridPlan = {};
                return;
            end
            hybridPlan{end + 1} = struct('type', 'move', 'path', {path});
            qCurrent = qGoal;

        case 'pick'
            [graspPose, qGrasp] = sample_grasp_and_ik(world, objectPose, qCurrent);
            if isempty(qGrasp)
                hybridPlan = {};
                return;
            end
            pathToGrasp = rrt_connect_matlab(world, qCurrent, qGrasp, 'transit', objectPose);
            if isempty(pathToGrasp)
                hybridPlan = {};
                return;
            end
            hybridPlan{end + 1} = struct('type', 'pick', ...
                                         'path', {pathToGrasp}, ...
                                         'graspPose', graspPose);
            qCurrent = qGrasp;

        case 'place'
            objectPose = goalRegionPose;
            qPlace = named_config('place_pose');
            pathTransfer = rrt_connect_matlab(world, qCurrent, qPlace, 'transfer', objectPose);
            if isempty(pathTransfer)
                hybridPlan = {};
                return;
            end
            hybridPlan{end + 1} = struct('type', 'place', ...
                                         'path', {pathTransfer}, ...
                                         'goalPose', goalRegionPose);
            qCurrent = qPlace;
    end
end
end

function qGoal = named_config(name)
% Map a symbolic name to a configuration, e.g. loaded from a MAT file
qGoal = []; % TODO
end

function path = rrt_connect_matlab(world, qStart, qGoal, mode, objectPose)
% Thin MATLAB wrapper around your Chapter 3 planner
path = {}; % TODO
end

function [graspPose, qGrasp] = sample_grasp_and_ik(world, objectPose, qSeed)
% Sample a few top-down grasps and call IK
graspPose = [];
qGrasp = [];
end
      
