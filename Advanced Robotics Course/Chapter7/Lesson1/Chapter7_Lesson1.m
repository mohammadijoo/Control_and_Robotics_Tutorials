function W = planarContactWrenchGenerators(p_obj, mu, mEdges)
%PLANARCONTACTWRENCHGENERATORS  2D friction cone wrench generators.
%   p_obj: [2x1] contact position [x; y] in object frame.
%   mu: scalar friction coefficient.
%   mEdges: number of edges for polyhedral cone approximation.
%
%   W: [3 x mEdges] matrix, columns are wrench generators [fx; fy; tau_z].

if mu <= 0
    error('mu must be positive');
end

alpha = atan(mu);
angles = linspace(-alpha, alpha, mEdges);

fx = sin(angles);
fy = cos(angles);

px = p_obj(1);
py = p_obj(2);
tau_z = px .* fy - py .* fx;

W = [fx; fy; tau_z];

end

% Example usage:
pL = [-0.05; 0.0];
pR = [0.05; 0.0];
mu = 0.8;
mEdges = 8;

WL = planarContactWrenchGenerators(pL, mu, mEdges);
WR = planarContactWrenchGenerators(pR, mu, mEdges);
G = [WL, WR];  % 3 x (2*mEdges)

% In Simulink, one may:
% 1) Use Simscape Multibody blocks to simulate actual contacts.
% 2) Use MATLAB Function blocks that call planarContactWrenchGenerators
%    to compute linearized cones for model-predictive control or planning.
      
