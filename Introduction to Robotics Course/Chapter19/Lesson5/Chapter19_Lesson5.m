Rmax = 1.0;      % safe radius for state
gamma = 0.5;     % allowed tracking error
for k = 1:length(t)
    xk = x(:,k);
    ek = r(k) - y(k);

    if norm(xk, 2) > Rmax
        warning('State left safe set: ||x||_2 = %.3f > Rmax', norm(xk, 2));
    end
    if abs(ek) > gamma
        warning('Tracking error too large at k=%d: e=%.3f', k, ek);
    end
end
      
