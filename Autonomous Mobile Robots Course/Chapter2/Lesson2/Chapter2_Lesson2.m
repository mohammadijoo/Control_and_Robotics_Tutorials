% Chapter 2 - Lesson 2: Differential Drive Kinematics (MATLAB/Simulink)
%
% This script demonstrates:
%   - wheel rates -> (v,w)
%   - exact discrete-time integration
%   - optional use of robotics.DifferentialDriveKinematics (Robotics System Toolbox)

clear; clc;

r = 0.05;   % wheel radius [m]
L = 0.30;   % axle length [m]

phiDotL = 5.0;   % rad/s
phiDotR = 8.0;   % rad/s
dt = 0.1;        % s
N = 50;

pose = [0; 0; 0]; % [x; y; theta]

% Forward map: wheel rates -> (v,w)
vL = r * phiDotL;
vR = r * phiDotR;
v  = 0.5*(vR + vL);
w  = (vR - vL)/L;
fprintf('v = %.6f m/s, w = %.6f rad/s\n', v, w);

% Simulate with exact integration
traj = zeros(3, N+1);
traj(:,1) = pose;
for k = 1:N
    pose = integratePoseExact(pose, v, w, dt);
    traj(:,k+1) = pose;
end
fprintf('Final pose [x,y,theta] = [%.6f, %.6f, %.6f]\n', traj(1,end), traj(2,end), traj(3,end));

% Inverse map: (v,w) -> wheel rates
phiDotR2 = (v + 0.5*L*w)/r;
phiDotL2 = (v - 0.5*L*w)/r;
fprintf('Inverse check [phiDotL, phiDotR] = [%.6f, %.6f]\n', phiDotL2, phiDotR2);

% Optional: Robotics System Toolbox
% If installed, you can use the differential-drive kinematics object:
if exist('robotics.DifferentialDriveKinematics','class') == 8
    kin = robotics.DifferentialDriveKinematics('TrackWidth', L, 'WheelRadius', r);
    % The object expects body speeds [v, w]
    poseObj = [0 0 0];
    for k = 1:N
        poseObj = poseObj + derivative(kin, poseObj, [v w])' * dt; %#ok<AGROW>
    end
    fprintf('Toolbox (Euler) pose [x,y,theta] = [%.6f, %.6f, %.6f]\n', poseObj(1), poseObj(2), poseObj(3));
end

% ---- Simulink note (programmatic model build) ----
% For a Simulink model, represent:
%   v = 0.5*r*(phiDotR + phiDotL)
%   w = (r/L)*(phiDotR - phiDotL)
% then integrate:
%   x_dot = v*cos(theta), y_dot = v*sin(theta), theta_dot = w
% You can create this with blocks (Gain, Sum, Trig, Product, Integrator)
% or via a MATLAB Function block calling integratePoseExact in discrete time.

function poseNext = integratePoseExact(pose, v, w, dt)
    x = pose(1); y = pose(2); th = pose(3);
    if abs(w) < 1e-12
        poseNext = [x + v*dt*cos(th);
                    y + v*dt*sin(th);
                    th];
        return;
    end
    th2 = th + w*dt;
    x2 = x + (v/w)*(sin(th2) - sin(th));
    y2 = y - (v/w)*(cos(th2) - cos(th));
    poseNext = [x2; y2; th2];
end
