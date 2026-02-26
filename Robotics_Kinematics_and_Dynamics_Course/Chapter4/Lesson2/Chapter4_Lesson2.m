% Number of links (including base)
nLinks = 4;
parent = -ones(1, nLinks);   % parent(1) = -1 (base)
for i = 2:nLinks
    parent(i) = i - 1;
end

% Joint transforms as function handles
linkLengths = [1.0, 1.0, 1.0];
jointTF = cell(1, nLinks);
jointTF{1} = [];  % no joint to base
for i = 2:nLinks
    a = linkLengths(i - 1);
    jointTF{i} = @(theta) planarR(theta, a);
end

function T = planarR(theta, a)
    c = cos(theta); s = sin(theta);
    T = eye(4);
    T(1,1) = c;  T(1,2) = -s;
    T(2,1) = s;  T(2,2) =  c;
    T(1,4) = a;
end

function T = forwardTransforms(q, parent, jointTF)
    nLinks = numel(parent);
    T = repmat(eye(4), 1, 1, nLinks);
    for i = 2:nLinks
        p = parent(i);
        T_pi = jointTF{i}(q(i - 1));
        T(:, :, i) = T(:, :, p) * T_pi;
    end
end

% Example
q = [0.2; 0.4; -0.3];
T = forwardTransforms(q, parent, jointTF);
T_ee = T(:, :, end)

% Programmatic Simulink (Simscape Multibody) sketch:
modelName = "serial_chain_model";
new_system(modelName);
open_system(modelName);

% Add Simscape Multibody environment and 3 revolute joints with links
% (actual block names may vary by MATLAB version)
% add_block('sm_lib/Body Elements/Body', [modelName '/Link1']);
% add_block('sm_lib/Joints/Revolute Joint', [modelName '/Joint1']);
% ...
      
