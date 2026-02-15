function traj = propagate_double_integrator(x0, u, dt, T_step, u_max)
% x0 = [x; v]
% u: scalar control
    if nargin < 3, dt = 0.01; end
    if nargin < 4, T_step = 0.5; end
    if nargin < 5, u_max = 1.0; end

    u_sat = max(-u_max, min(u, u_max));
    x = x0(:);
    N = floor(T_step / dt);
    traj = zeros(2, N+1);
    traj(:,1) = x;

    for k = 1:N
        xdot = [x(2); u_sat];   % [x_dot; v_dot]
        x = x + dt * xdot;
        traj(:,k+1) = x;
    end
end

% In Simulink:
% - Create an Integrator block chain for v and x.
% - Feed a Saturation block with constant u into the acceleration input.
% - Log x and v using "To Workspace" blocks for analysis.
      
