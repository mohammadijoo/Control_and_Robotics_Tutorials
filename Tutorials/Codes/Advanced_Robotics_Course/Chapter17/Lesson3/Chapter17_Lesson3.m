function [b_next, a_qmdp] = deformable_qmdp_step(b, a_prev, o, T, Z, Q)
% b      : current belief column vector (Sx1)
% a_prev : last action index (1..A)
% o      : new observation index (1..O)
% T      : SxSxA transition probabilities (T(s_next, s, a))
% Z      : SxAxO observation probabilities (Z(s_next, a, o))
% Q      : SxA MDP Q-values for QMDP
%
% b_next : updated belief
% a_qmdp : next action from QMDP

S = size(T, 1);
A = size(T, 3);

% Prediction: b_pred(s') = sum_s T(s', s, a_prev) * b(s)
b_pred = zeros(S,1);
for sp = 1:S
    for s = 1:S
        b_pred(sp) = b_pred(sp) + T(sp,s,a_prev) * b(s);
    end
end

% Correction: multiply by likelihood Z(s', a_prev, o)
lik = Z(:,a_prev,o);
b_next = lik .* b_pred;
norm_const = sum(b_next);

if norm_const < 1e-12
    b_next = ones(S,1) / S;
else
    b_next = b_next / norm_const;
end

% QMDP action selection: maximize sum_s b_next(s) * Q(s,a)
Qa = zeros(A,1);
for a = 1:A
    Qa(a) = b_next' * Q(:,a);
end
[~, a_qmdp] = max(Qa);
      
