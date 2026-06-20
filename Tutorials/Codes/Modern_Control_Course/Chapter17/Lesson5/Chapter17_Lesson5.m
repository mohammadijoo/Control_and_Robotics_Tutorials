% Chapter17_Lesson5.m
% Practical Considerations in Choosing Canonical Forms
% Requires: base MATLAB. Control System Toolbox is optional for ss/canon.

clear; clc;

A0 = [0 1 0; 0 0 1; -6 -11 -6];
B0 = [0; 0; 1];
C0 = [1 0 0];
D0 = 0;

% Scaled coordinates x = S*z0
S = diag([1, 0.05, 20]);
A = S*A0/S;
B = S*B0;
C = C0/S;
D = D0;

% Observable canonical target: transpose of controllable companion form
Ao = A0.';
Co = [0 0 1];
O = obsv_local(A, C);
Oo = obsv_local(Ao, Co);
T_ocf = O \ Oo;
A_ocf = T_ocf \ A * T_ocf;
B_ocf = T_ocf \ B;
C_ocf = C * T_ocf;

fprintf('Rank of observability matrix: %d\n', rank(O));
fprintf('OCF transform condition number: %.6g\n', cond(T_ocf));
disp('A in observable canonical coordinates:'); disp(A_ocf);
disp('C in observable canonical coordinates:'); disp(C_ocf);

% Modal transformation
[V,Lambda] = eig(A);
A_modal = V \ A * V;
B_modal = V \ B;
C_modal = C * V;

fprintf('Modal eigenvector condition number: %.6g\n', cond(V));
disp('A in modal coordinates:'); disp(A_modal);

% Optional Control System Toolbox checks
if exist('ss','file') == 2
    sys = ss(A,B,C,D);
    disp('State-space object created. Use canon(sys,''modal'') if Control System Toolbox supports it.');
    try
        sys_modal = canon(sys, 'modal'); %#ok<NASGU>
        disp('MATLAB canon(sys,''modal'') succeeded.');
    catch ME
        fprintf('canon(sys,''modal'') not available or failed: %s\n', ME.message);
    end
end

if min(cond(V), cond(T_ocf)) > 5000
    disp('Recommendation: both canonical transforms are poorly conditioned; keep physical/scaled coordinates or use an orthogonal Schur form.');
elseif cond(V) < cond(T_ocf)
    disp('Recommendation: modal form is numerically preferable for this example.');
else
    disp('Recommendation: OCF is acceptable for observer algebra in this example.');
end

function O = obsv_local(A,C)
    n = size(A,1);
    O = zeros(n,n);
    Ak = eye(n);
    for k = 1:n
        O(k,:) = C*Ak;
        Ak = Ak*A;
    end
end
