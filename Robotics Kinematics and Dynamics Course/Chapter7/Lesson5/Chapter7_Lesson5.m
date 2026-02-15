function Jnum = numericJacobianVec(fkHandle, q, h)
%NUMERICJACOBIANVEC Central-difference Jacobian for f: R^n -> R^m
%
% fkHandle : function handle, fkHandle(q) -> x (m-by-1)
% q        : column vector (n-by-1)
% h        : stepsize

    if nargin < 3
        h = 1e-6;
    end

    q = q(:);
    n = numel(q);
    x0 = fkHandle(q);
    m = numel(x0);

    Jnum = zeros(m, n);

    for i = 1:n
        dq = zeros(n,1);
        dq(i) = h;

        xp = fkHandle(q + dq);
        xm = fkHandle(q - dq);

        Jnum(:, i) = (xp - xm) / (2*h);
    end
end
      
