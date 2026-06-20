% Parameters
zeta = 0.5;
wn   = 10;  % rad/s

% Second-order transfer function G(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
num = [wn^2];
den = [1, 2*zeta*wn, wn^2];
G = tf(num, den);

% Time-domain simulation
t = 0:0.001:3;
[y, t_out] = step(G, t);

% Built-in transient metrics (2% default)
info = stepinfo(G);

fprintf('Rise time (10-90%%)    : %.4f s\n', info.RiseTime);
fprintf('Peak time             : %.4f s\n', info.PeakTime);
fprintf('Overshoot             : %.2f %%\n', info.Overshoot);
fprintf('Settling time (2%%)   : %.4f s\n', info.SettlingTime);

% Manual metrics using sampled data (similar to earlier algorithms)
band = 0.02;
c_inf = mean(y(round(0.9*length(y)):end));
low = 0.1*c_inf;
high = 0.9*c_inf;

t10 = NaN; t90 = NaN;
for k = 1:length(t_out)
    if isnan(t10) && y(k) >= low
        t10 = t_out(k);
    end
    if isnan(t90) && y(k) >= high
        t90 = t_out(k);
        break;
    end
end
tr = t90 - t10;

[~, idx_max] = max(y);
tp = t_out(idx_max);
Mp = (y(idx_max) - c_inf) / c_inf;

tol = band * abs(c_inf);
ts = t_out(end);
for k = 1:length(t_out)
    if all(abs(y(k:end) - c_inf) <= tol)
        ts = t_out(k);
        break;
    end
end

fprintf('\nManual computation:\n');
fprintf('tr = %.4f, tp = %.4f, Mp = %.4f, ts = %.4f\n', tr, tp, Mp, ts);

% In Simulink, a second-order plant block and a Step block can be used,
% and "Scope" or "To Workspace" block can provide y(t) for the same metrics.
