% Assume t, r, y, u are column vectors of equal length
dt = [t(1); diff(t)];     % time steps
e = r - y;

E_rms = sqrt(mean(e.^2));
E_iae = sum(abs(e) .* dt);
U2 = sum(u.^2 .* dt);

% Basic overshoot and settling time for approximate step response
y_final = mean(y(end-50:end));   % average of final samples
y_max = max(y);
Mp = (y_max - y_final) / max(abs(y_final));   % relative overshoot

% Settling time: find last time index where |y - y_final| exceeds band
band = 0.02 * max(abs(y_final)); % 2 percent
idx = find(abs(y - y_final) > band, 1, "last");
if isempty(idx)
    Ts = 0.0;
else
    Ts = t(idx);
end

fprintf("RMS error: %g\n", E_rms);
fprintf("IAE:       %g\n", E_iae);
fprintf("U^2:       %g\n", U2);
fprintf("Overshoot: %g\n", Mp);
fprintf("Ts:        %g\n", Ts);
      
