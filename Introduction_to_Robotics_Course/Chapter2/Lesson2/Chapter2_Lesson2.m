
function q = cubic_ptp(q0, qf, T, t)
    % Cubic point-to-point joint interpolation
    t = min(max(t,0),T);
    Delta = qf - q0;
    s = t / T;
    q = q0 + 3*Delta*s.^2 - 2*Delta*s.^3;
end

% Example usage
q0 = 0.2; qf = 1.0; T = 2.0;
ts = linspace(0,T,50);
qs = arrayfun(@(tt)cubic_ptp(q0,qf,T,tt), ts);
disp(qs(1:5));
      