% Chapter5_Lesson1.m
% Wheel Odometry Computation — Differential Drive (planar SE(2))
%
% Run:
%   Chapter5_Lesson1
%
% Requires: none (base MATLAB). For robotics workflows, Robotics System Toolbox
% provides standardized odometry messages and frames, but this file is core-math.

function Chapter5_Lesson1()
    params.rL = 0.05;         % [m]
    params.rR = 0.05;         % [m]
    params.b  = 0.30;         % [m]
    params.ticksPerRev = 2048;
    params.gearRatio   = 1.0;

    pose = [0; 0; 0]; % [x; y; theta]

    for k = 1:200
        pose = odo_update_from_ticks(pose, 40, 60, params);
    end

    fprintf('Final pose: x=%.6f y=%.6f theta=%.6f rad\n', pose(1), pose(2), pose(3));
end

function theta = normalize_angle(theta)
    theta = atan2(sin(theta), cos(theta));
end

function dphi = ticks_to_wheel_angle(dN, params)
    denom = double(params.ticksPerRev) * double(params.gearRatio);
    dphi = 2*pi * (double(dN) / denom);
end

function pose = odo_update_from_ticks(pose, dN_L, dN_R, params)
    dphiL = ticks_to_wheel_angle(dN_L, params);
    dphiR = ticks_to_wheel_angle(dN_R, params);

    dsL = params.rL * dphiL;
    dsR = params.rR * dphiR;

    deltaS = 0.5*(dsR + dsL);
    deltaTheta = (dsR - dsL)/params.b;

    x = pose(1); y = pose(2); th = pose(3);

    eps = 1e-12;
    if abs(deltaTheta) < eps
        x = x + deltaS*cos(th);
        y = y + deltaS*sin(th);
        th = normalize_angle(th + deltaTheta);
    else
        R = deltaS/deltaTheta;
        x = x + R*(sin(th + deltaTheta) - sin(th));
        y = y - R*(cos(th + deltaTheta) - cos(th));
        th = normalize_angle(th + deltaTheta);
    end

    pose = [x; y; th];
end
