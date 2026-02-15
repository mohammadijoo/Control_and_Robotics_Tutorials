% Vector of severities and probabilities (1..4 scale)
S = [3; 2; 4];   % severities for 3 hazards
P = [3; 2; 3];   % probabilities

I = S .* P;  % risk indices

risk_level = strings(size(I));
risk_level(I <= 4) = "low";
risk_level(I > 4 & I <= 8) = "medium";
risk_level(I > 8) = "high";

disp(table(S, P, I, risk_level));

% Simulink sketch:
%  - Three constant blocks for S(i)
%  - Three constant blocks for P(i)
%  - Product blocks computing I(i) = S(i)*P(i)
%  - MATLAB Function block to map indices to string labels
      
