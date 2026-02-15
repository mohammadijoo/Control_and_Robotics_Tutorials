
function J = total_cost(q, qd, tau, Q, R, Su, dt, lambdaTrack, lambdaEffort, lambdaSmooth)
% q, qd: (N+1)-by-n
% tau:   N-by-n

e = q - qd;                           % (N+1)-by-n
J_track = 0.5 * dt * sum(diag(e * Q * e'));

J_effort = 0.5 * dt * sum(diag(tau * R * tau'));

dtau = diff(tau, 1, 1);               % (N-1)-by-n
J_smooth = 0.5 * sum(diag(dtau * Su * dtau'));

J = lambdaTrack * J_track + ...
    lambdaEffort * J_effort + ...
    lambdaSmooth * J_smooth;
end
