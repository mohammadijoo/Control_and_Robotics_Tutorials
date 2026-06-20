A = sym([2 1 0;
         0 2 0;
         0 0 3]);

% Jordan decomposition: A = P*J*inv(P)
[P,J] = jordan(A);

disp('J ='); disp(J);
disp('Check A == P*J*inv(P):'); disp(isequal(A, P*J*inv(P)));

% Kernel-dimension method for lambda=2
lam = sym(2);
d1 = null(A - lam*eye(3));          % basis for ker(A-lam I)
d2 = null((A - lam*eye(3))^2);      % basis for ker((A-lam I)^2)
fprintf('nullity k=1: %d\n', size(d1,2));
fprintf('nullity k=2: %d\n', size(d2,2));
