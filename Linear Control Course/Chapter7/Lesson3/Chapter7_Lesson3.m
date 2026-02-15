function [R, nRHP] = routh_array(coeffs)
% ROUTH_ARRAY Construct Routh array for polynomial with real coefficients.
%   coeffs: [a_n, ..., a_0], a_n > 0

n = numel(coeffs) - 1;
m = ceil((n + 1) / 2);
R = zeros(n + 1, m);

% First two rows
R(1,1:2:end) = coeffs(1:2:end);
R(2,1:2:end) = coeffs(2:2:end);

eps_val = 1e-9;

for i = 3:n+1
    if abs(R(i-1,1)) < eps_val
        R(i-1,1) = eps_val;
    end
    for j = 1:m-1
        a = R(i-1,1);
        b = R(i-2,1);
        c = R(i-2,j+1);
        d = R(i-1,j+1);
        R(i,j) = (a*c - b*d)/a;
    end
end

% Count sign changes
col = R(:,1);
col(abs(col) < eps_val) = eps_val;
s = sign(col);
nRHP = 0;
for i = 1:n
    if s(i)*s(i+1) < 0
        nRHP = nRHP + 1;
    end
end
end

%% Example: DC motor robot joint with proportional control
K = 10;
num = K;
den = [1 6 8 0];
G = tf(num, den);       % plant
Gcl = feedback(G, 1);   % unity feedback

[~, den_cl] = tfdata(Gcl, 'v');
[R, nRHP] = routh_array(den_cl);

disp('Routh array:');
disp(R);
fprintf('Number of RHP roots: %d\n', nRHP);
if nRHP == 0
    fprintf('Closed-loop is asymptotically stable.\n');
else
    fprintf('Closed-loop is unstable.\n');
end
