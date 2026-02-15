% First-order plant and unity feedback with proportional gain
s = tf('s');
G = 1/(s + 1);

gains = [1 2 5 10];
for K = gains
    C = K;
    T = feedback(C*G, 1);   % closed-loop from r to y
    S = feedback(1, C*G);   % sensitivity (from r to e)

    % Step response metrics
    info = stepinfo(T);
    ess = abs(1 - dcgain(T));         % steady-state error to unit step

    % Control signal for unit step: u = C * e
    [y, t] = step(T);
    e = 1 - y;
    u = K * e;
    umax = max(abs(u));

    fprintf('K = %4.1f | ess = %.4f, ts = %.3f, umax = %.3f\n', ...
            K, ess, info.SettlingTime, umax);
end

% In Simulink, the same loop can be built using:
%  - Sum block for e = r - y
%  - Gain block for C = K
%  - Transfer Fcn block for G(s) = 1/(s+1)
%  - Saturation block to model actuator limits
% and the Scope block to visualize y(t), e(t), and u(t).
