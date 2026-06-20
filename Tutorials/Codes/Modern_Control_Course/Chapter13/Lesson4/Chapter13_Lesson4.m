% Chapter13_Lesson4.m
% Relationship of Observability to Sensor Placement
% Uses MATLAB matrix operations. If Control System Toolbox is available,
% obsv(A,C) gives the same stacked observability matrix.

clear; clc;

A = [ 0.0  1.0  0.0  0.0;
     -2.0 -0.4  0.8  0.0;
      0.0  0.0 -1.0  1.0;
      0.6  0.0 -3.0 -0.5 ];

n = size(A,1);

fprintf("Sensor placement by rank of O = [C; C A; ...; C A^(n-1)]\n");

for r = 1:2
    subsets = nchoosek(1:n, r);
    for idx = 1:size(subsets,1)
        sensors = subsets(idx,:);
        C = zeros(r,n);
        for k = 1:r
            C(k,sensors(k)) = 1;
        end

        O = local_observability_matrix(A,C);
        rk = rank(O);
        G = O' * O;
        sigma_min = min(svd(O));
        logdet_proxy = log(det(G + 1e-9*eye(n)));

        fprintf("Sensors {%s}: rank=%d, sigma_min=%.3e, logdet=%.3f\n", ...
            num2str(sensors), rk, sigma_min, logdet_proxy);
    end
end

% Compare with MATLAB Control System Toolbox, if available:
% C = [1 0 0 0];
% O_toolbox = obsv(A,C);

function O = local_observability_matrix(A,C)
    n = size(A,1);
    O = [];
    Ak = eye(n);
    for k = 1:n
        O = [O; C*Ak]; %#ok<AGROW>
        Ak = Ak*A;
    end
end
