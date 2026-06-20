% Chapter11_Lesson4.m
% Structural controllability graph test for xdot = A x + B u.
% Pattern entries: 1 = free structural nonzero, 0 = fixed zero.
%
% Simulink note:
% If a linearized Simulink model is available as sys = linearize("modelName",io),
% then a structural pattern can be formed by
%   Abar = double(abs(sys.A) > tol);
%   Bbar = double(abs(sys.B) > tol);
% and then tested with structuralControllability(Abar,Bbar).

clear; clc;

A_chain = [0 0 0;
           1 0 0;
           0 1 0];
B_chain = [1; 0; 0];

A_dilation = [0 0 0;
              1 0 0;
              1 0 0];
B_dilation = [1; 0; 0];

disp("Chain example");
disp(structuralControllability(A_chain, B_chain));

disp("Dilation example");
disp(structuralControllability(A_dilation, B_dilation));

function result = structuralControllability(Abar, Bbar)
    [n, ~] = validatePattern(Abar, Bbar);
    reach = inputReachability(Abar, Bbar);
    matchingSize = maximumPatternMatchingSize(Abar, Bbar);

    result = struct();
    result.all_states_input_reachable = all(reach);
    result.reachable_state_flags = reach;
    result.maximum_matching_size = matchingSize;
    result.no_dilation_via_full_row_matching = (matchingSize == n);
    result.structurally_controllable = all(reach) && (matchingSize == n);
end

function [n, m] = validatePattern(Abar, Bbar)
    [n1, n2] = size(Abar);
    if n1 ~= n2
        error("Abar must be square.");
    end
    [bRows, m] = size(Bbar);
    if bRows ~= n1
        error("Bbar row count must match Abar.");
    end
    if m == 0
        error("Bbar must have at least one input column.");
    end
    n = n1;
end

function reach = inputReachability(Abar, Bbar)
    [n, m] = validatePattern(Abar, Bbar);
    N = n + m;
    adj = cell(N, 1);
    for v = 1:N
        adj{v} = [];
    end

    % Abar(i,j) means x_j influences xdot_i, so x_j -> x_i.
    for i = 1:n
        for j = 1:n
            if Abar(i,j) ~= 0
                adj{j} = [adj{j}, i];
            end
        end
    end

    % Bbar(i,k) means u_k influences xdot_i, so u_k -> x_i.
    for i = 1:n
        for k = 1:m
            if Bbar(i,k) ~= 0
                adj{n+k} = [adj{n+k}, i];
            end
        end
    end

    seen = false(N, 1);
    q = zeros(N, 1);
    head = 1; tail = 0;

    for k = 1:m
        tail = tail + 1;
        q(tail) = n + k;
        seen(n+k) = true;
    end

    while head <= tail
        v = q(head);
        head = head + 1;
        for w = adj{v}
            if ~seen(w)
                seen(w) = true;
                tail = tail + 1;
                q(tail) = w;
            end
        end
    end

    reach = seen(1:n).';
end

function matchingSize = maximumPatternMatchingSize(Abar, Bbar)
    [n, m] = validatePattern(Abar, Bbar);
    leftCount = n + m;
    rightCount = n;
    adjLeft = cell(leftCount, 1);
    for u = 1:leftCount
        adjLeft{u} = [];
    end

    for i = 1:n
        for j = 1:n
            if Abar(i,j) ~= 0
                adjLeft{j} = [adjLeft{j}, i];
            end
        end
    end

    for i = 1:n
        for k = 1:m
            if Bbar(i,k) ~= 0
                adjLeft{n+k} = [adjLeft{n+k}, i];
            end
        end
    end

    pairV = zeros(rightCount, 1);
    matchingSize = 0;

    for u = 1:leftCount
        visited = false(rightCount, 1);
        if augment(u, visited)
            matchingSize = matchingSize + 1;
        end
    end

    function ok = augment(u, visited)
        ok = false;
        for v = adjLeft{u}
            if visited(v)
                continue;
            end
            visited(v) = true;
            if pairV(v) == 0 || augment(pairV(v), visited)
                pairV(v) = u;
                ok = true;
                return;
            end
        end
    end
end
