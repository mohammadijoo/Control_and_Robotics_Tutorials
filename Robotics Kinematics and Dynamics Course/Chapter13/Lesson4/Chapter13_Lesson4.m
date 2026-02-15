function result = checkLinkInertia(m, c, I_C)
%CHECKLINKINERTIA Physical consistency test for a single link.
%   m   : scalar mass
%   c   : 3x1 COM position
%   I_C : 3x3 inertia about COM

tol = 1e-9;
result.massPositive = (m > tol);

% Symmetrize
I_C = 0.5 * (I_C + I_C.');

% Eigenvalues
[eigVec, eigVal] = eig(I_C);
lambda = diag(eigVal);
result.I_C_SPD = all(lambda > tol);

% Triangle inequalities in principal frame
J1 = lambda(1); J2 = lambda(2); J3 = lambda(3);
result.triangleOK = (J1 <= J2 + J3 + tol) && ...
                    (J2 <= J1 + J3 + tol) && ...
                    (J3 <= J1 + J2 + tol);

% Spatial inertia SPD test
S = [  0,   -c(3),  c(2);
      c(3),  0,    -c(1);
     -c(2),  c(1),  0   ];
upperLeft  = I_C + m * (S * S.');
upperRight = m * S;
lowerLeft  = m * S.';
lowerRight = m * eye(3);

I_spatial = [upperLeft,  upperRight;
             lowerLeft,  lowerRight];

lambda_spatial = eig(I_spatial);
result.spatialSPD = all(lambda_spatial > tol);
end
      
