function rolling_disk_demo()
    R = 0.1;            % radius
    q0 = [0; 0; 0; 0];  % [x; y; theta; phi]
    Tspan = [0 5];
    u = 0.2;            % forward speed along body x-axis

    % Integrate kinematic model
    ode = @(t, q) rolling_disk_kinematics(t, q, u, R);
    [T, Q] = ode45(ode, Tspan, q0);

    figure; plot(Q(:,1), Q(:,2), 'LineWidth', 2);
    xlabel('x [m]'); ylabel('y [m]');
    axis equal; grid on;
    title('Rolling disk trajectory (non-holonomic)');

end

function qdot = rolling_disk_kinematics(~, q, u, R)
    x = q(1); %#ok<NASGU>
    y = q(2); %#ok<NASGU>
    theta = q(3);
    phi = q(4); %#ok<NASGU>

    v = u;
    xdot = v * cos(theta);
    ydot = v * sin(theta);
    thetadot = 0.0;
    phidot = v / R;

    qdot = [xdot; ydot; thetadot; phidot];
end
      
