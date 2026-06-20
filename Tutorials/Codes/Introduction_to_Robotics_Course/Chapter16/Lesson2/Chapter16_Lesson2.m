% Candidate concepts: rows = concepts, columns = [cost, mass, energy, tracking_error]
F = [ 15.0  40.0  800.0  1.5;
      18.0  50.0  700.0  1.2;
      25.0  55.0  950.0  0.8 ];

names = { 'A\_wheeled'; 'B\_tracked'; 'C\_legged' };

MAX_MASS   = 55.0;
MAX_ENERGY = 1000.0;

% Feasibility mask
isFeasible = (F(:,2) <= MAX_MASS) & (F(:,3) <= MAX_ENERGY);
F = F(isFeasible, :);
names = names(isFeasible);

% Normalize each column for minimization (1 = best, 0 = worst)
Fmin = min(F, [], 1);
Fmax = max(F, [], 1);
Z = (Fmax - F) ./ (Fmax - Fmin);   % implicit row-wise broadcast

% weights: cost, mass, energy, tracking_error
w = [0.3; 0.2; 0.2; 0.3];

U = Z * w;    % utility values for each concept

[Ubest, idxBest] = max(U);
fprintf('Best concept: %s, U = %.3f\n', names{idxBest}, Ubest);

% --- Simulink mapping idea ---
% In Simulink:
% 1) Represent Z as a vector signal (e.g., from a Constant block).
% 2) Use a Gain block with parameter w' (1-by-4) to compute U = w' * Z.
% 3) Optionally sweep w in time to visualize sensitivity of concept ranking.
      
