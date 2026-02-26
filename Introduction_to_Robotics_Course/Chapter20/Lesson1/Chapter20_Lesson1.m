% Define requirements
req.maxSettlingTime_s = 2.0;
req.maxOvershoot      = 0.05;
req.maxRmsError       = 0.02;

% Assume you have already simulated your closed-loop system and have:
% t : time vector
% y : output vector
% r : reference vector (same size as y)

e = r - y;
rmsError = sqrt(trapz(t, e.^2) / (t(end) - t(1)));

% Simple settling time and overshoot estimation for a step response
y_final = y(end);
overshoot = (max(y) - y_final) / max(1e-9, abs(y_final));

% Settling time: first time after which |y - y_final| <= 0.02*|y_final|
idx = find(abs(y - y_final) <= 0.02 * abs(y_final), 1, 'first');
if ~isempty(idx)
    settlingTime = t(idx);
else
    settlingTime = Inf;
end

metrics.settlingTime_s = settlingTime;
metrics.overshoot      = overshoot;
metrics.rmsError       = rmsError;

feasibility.settlingTimeOk = metrics.settlingTime_s <= req.maxSettlingTime_s;
feasibility.overshootOk    = metrics.overshoot      <= req.maxOvershoot;
feasibility.rmsErrorOk     = metrics.rmsError       <= req.maxRmsError;

disp(feasibility);
      
