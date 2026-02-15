function q = epsilon_quality_matlab(G, numDirs)
% G: 6 x p matrix of primitive wrenches
if nargin < 2
    numDirs = 200;
end
epsVal = inf;
for k = 1:numDirs
    v = randn(6,1);
    v = v / norm(v);
    proj = v' * G;           % 1 x p
    support = max(proj);
    epsVal = min(epsVal, support);
end
q = max(epsVal, 0.0);
end

% Example usage:
p = 16;
G = randn(6, p);
for j = 1:p
    G(:,j) = G(:,j) / norm(G(:,j));
end
q_example = epsilon_quality_matlab(G, 300);
fprintf('Approximate epsilon quality: %.4f\n', q_example);
      
