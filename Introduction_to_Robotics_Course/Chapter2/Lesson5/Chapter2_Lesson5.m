rng(0);

A_nom = [0.9 0.1; 0 0.85];
B_nom = [0.1; 0.2];
K = [1.2 0.4];
phi_scale = 0.1;

T = 50; M = 100;
finalStates = zeros(M,2);

for m=1:M
    A = A_nom + phi_scale*randn(size(A_nom));
    B = B_nom + phi_scale*randn(size(B_nom));
    x = [1;0];
    for t=1:T
        u = -K*x;
        x = A*x + B*u;
    end
    finalStates(m,:) = x';
end

mean(finalStates)
cov(finalStates)
      