% Chapter11_Lesson4.m
% Chapter 11 (SLAM I) — Lesson 4: Data Association Challenges
% MATLAB demo: chi-square gating (NIS) + NN association + programmatic Simulink model.

function Chapter11_Lesson4()
clc;

poseTrue = [2.0; 1.0; 0.4];
lmsTrue  = [5 2; 4 -1.5; 1 4; 7 5; 6 -2];

poseEst  = poseTrue + [0.1; -0.05; 0.03];
lmsEst   = lmsTrue + repmat([0.2 -0.1], size(lmsTrue,1), 1);
N = size(lmsEst,1);

% demo SPD covariance
dim = 3 + 2*N;
rng(7); A = randn(dim, dim);
P = A*A'; P = P ./ max(diag(P));
S = diag([0.2 0.2 0.05 repmat(0.5, 1, 2*N)]);
P = S*P*S;

sigma_r = 0.15; sigma_b = deg2rad(2.0);
R = diag([sigma_r^2, sigma_b^2]);

% generate 3 noisy measurements from landmarks 1,3,5
ids = [1 3 5]; rng(1);
Z = zeros(2, numel(ids));
for i=1:numel(ids)
    z = predictZ(poseTrue, lmsTrue(ids(i),:)');
    z = z + mvnrnd([0 0], R)'; %#ok<MVNRND>
    z(2) = wrapToPi(z(2));
    Z(:,i) = z;
end

disp('Measurements [range, bearing]:'), disp(Z')

disp('NN association with gating:')
for i=1:size(Z,2)
    j = associateNN(poseEst, lmsEst, P, Z(:,i), R, 0.99);
    fprintf('  meas %d -> landmark %d\n', i, j);
end

buildSimulinkNN('Chapter11_Lesson4_Simulink', N);
disp('Created Simulink model: Chapter11_Lesson4_Simulink.slx')
end


% ---------- model ----------
function z = predictZ(pose, lm)
dx = lm(1) - pose(1); dy = lm(2) - pose(2);
z = [hypot(dx,dy); wrapToPi(atan2(dy,dx) - pose(3))];
end

function a = wrapToPi(a)
a = mod(a + pi, 2*pi) - pi;
end

function H = jacobianH(pose, lm, j, N)
dx = lm(1) - pose(1); dy = lm(2) - pose(2);
q = dx^2 + dy^2; r = sqrt(q);
H = zeros(2, 3 + 2*N);
H(1,1)=-dx/r; H(1,2)=-dy/r;
H(2,1)= dy/q; H(2,2)=-dx/q; H(2,3)=-1;
idx = 3 + 2*(j-1) + 1;
H(1,idx)=dx/r; H(1,idx+1)=dy/r;
H(2,idx)=-dy/q; H(2,idx+1)=dx/q;
end

function [nu,S] = innovAndS(pose, lms, P, z, j, R)
N = size(lms,1);
lm = lms(j,:)';
zhat = predictZ(pose, lm);
nu = z - zhat; nu(2)=wrapToPi(nu(2));
H = jacobianH(pose, lm, j, N);
S = H*P*H' + R;
end

function gamma = chi2invApprox(p, dof)
% toolbox-free chi2 inverse (Wilson–Hilferty)
z = sqrt(2)*erfinv(2*p - 1);
k = dof;
gamma = k*(1 - 2/(9*k) + z*sqrt(2/(9*k)))^3;
end

function bestJ = associateNN(pose, lms, P, z, R, gateProb)
N = size(lms,1);
gamma = chi2invApprox(gateProb, 2);
bestJ = 0; bestD2 = inf;
for j=1:N
    [nu,S] = innovAndS(pose, lms, P, z, j, R);
    d2 = nu'*(S\nu);
    if d2 <= gamma && d2 < bestD2
        bestD2 = d2; bestJ = j;
    end
end
end


% ---------- Simulink ----------
function buildSimulinkNN(modelName, N)
if bdIsLoaded(modelName), close_system(modelName, 0); end
if exist([modelName '.slx'], 'file') == 2, delete([modelName '.slx']); end

new_system(modelName); open_system(modelName);

add_block('simulink/Sources/Constant', [modelName '/z'], 'Position',[30 50 120 90], 'Value','[1;0]');
add_block('simulink/Sources/Constant', [modelName '/zhat'], 'Position',[30 120 120 160], 'Value',sprintf('zeros(%d,1)',2*N));
add_block('simulink/Sources/Constant', [modelName '/Sdiag'], 'Position',[30 190 120 230], 'Value',sprintf('ones(%d,1)',2*N));

add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/NN_Gate'], 'Position',[220 80 380 200]);
add_block('simulink/Sinks/Display', [modelName '/Display'], 'Position',[430 120 500 160]);

code = sprintf([ ...
'function idx = f(z, zhat, Sdiag)\n' ...
'N = length(zhat)/2;\n' ...
'gamma = chi2invApprox(0.99, 2);\n' ...
'best = 1e9; idx = 0;\n' ...
'for j=1:N\n' ...
'  nu = z - zhat(2*j-1:2*j);\n' ...
'  nu(2) = wrapToPi(nu(2));\n' ...
'  S = diag(Sdiag(2*j-1:2*j));\n' ...
'  d2 = nu''*(S\\nu);\n' ...
'  if d2 <= gamma && d2 < best\n' ...
'    best = d2; idx = j;\n' ...
'  end\n' ...
'end\n' ...
'end\n\n' ...
'function a = wrapToPi(a)\n' ...
'a = mod(a + pi, 2*pi) - pi;\n' ...
'end\n\n' ...
'function g = chi2invApprox(p, dof)\n' ...
'z = sqrt(2)*erfinv(2*p - 1);\n' ...
'k = dof;\n' ...
'g = k*(1 - 2/(9*k) + z*sqrt(2/(9*k)))^3;\n' ...
'end\n' ...
]);

set_param([modelName '/NN_Gate'], 'Script', code);

add_line(modelName, 'z/1', 'NN_Gate/1');
add_line(modelName, 'zhat/1', 'NN_Gate/2');
add_line(modelName, 'Sdiag/1', 'NN_Gate/3');
add_line(modelName, 'NN_Gate/1', 'Display/1');

save_system(modelName);
end
