% Chapter14_Lesson2.m
% Equilibrium points, phase portrait, and trajectories for a planar nonlinear system.
%
% Requires: MATLAB (Optimization Toolbox for fsolve is helpful but not strictly required).
% If you do not have fsolve, skip the equilibrium solver section and use the analytic equilibria.

function Chapter14_Lesson2()
    a = 1.0;

    % System definition: z = [x; y], z' = [f; g]
    f = @(x,y) x - x.^3 - y;
    g = @(x,y) x + a*y;
    F = @(t,z) [f(z(1), z(2)); g(z(1), z(2))];

    % ---- 1) Equilibria (analytic for this example) ----
    xp = sqrt((a+1)/a);
    eqs = [0, 0; +xp, -xp; -xp, +xp];

    % ---- 2) Local classification via Jacobian invariants ----
    fprintf('Equilibria and local classification (linearization):\n');
    for k = 1:size(eqs,1)
        x = eqs(k,1); y = eqs(k,2);
        J = [1 - 3*x^2, -1; 1, a];
        tr = trace(J);
        detJ = det(J);
        disc = tr^2 - 4*detJ;

        kind = classify2x2(tr, detJ, disc);
        fprintf('  z*=(% .6f, % .6f)  trace=% .6f det=% .6f -> %s\n', x, y, tr, detJ, kind);
    end

    % ---- 3) Phase portrait (vector field) ----
    figure; hold on; grid on;
    xlim([-2.5 2.5]); ylim([-2.5 2.5]);
    xlabel('x'); ylabel('y');
    title('Phase portrait: x''=x-x^3-y, y''=x+a y');

    [X,Y] = meshgrid(linspace(-2.5,2.5,25), linspace(-2.5,2.5,25));
    U = f(X,Y);
    V = g(X,Y);
    quiver(X,Y,U,V,'AutoScale','on');

    % Plot equilibria
    plot(eqs(:,1), eqs(:,2), 'o', 'MarkerSize', 7);

    % ---- 4) Trajectories via ode45 ----
    initials = [-2 -2; -2 0; -2 2; 0.5 -2; 0.5 2; 2 -2; 2 0; 2 2];
    for k = 1:size(initials,1)
        z0 = initials(k,:)';
        [t, z] = ode45(F, [0 12], z0);
        plot(z(:,1), z(:,2), 'LineWidth', 1.0);
    end
    hold off;

    % ---- 5) Simulink (programmatic build) ----
    % This builds a simple Simulink model with two Integrators and a MATLAB Function block.
    % Uncomment to generate and save an .slx model:
    % build_simulink_model(a);
end

function kind = classify2x2(tr, detJ, disc)
    eps = 1e-12;
    if detJ < -eps
        kind = 'saddle (hyperbolic)';
        return;
    end
    if abs(detJ) <= eps
        kind = 'degenerate (det ~ 0)';
        return;
    end

    if disc > eps
        if tr < -eps
            kind = 'stable node';
        elseif tr > eps
            kind = 'unstable node';
        else
            kind = 'improper/star node';
        end
    elseif disc < -eps
        if tr < -eps
            kind = 'stable spiral (focus)';
        elseif tr > eps
            kind = 'unstable spiral (focus)';
        else
            kind = 'center (linear); nonlinear decides';
        end
    else
        if tr < -eps
            kind = 'stable degenerate node';
        elseif tr > eps
            kind = 'unstable degenerate node';
        else
            kind = 'degenerate/center';
        end
    end
end

function build_simulink_model(a)
    model = 'Chapter14_Lesson2_Simulink';
    if bdIsLoaded(model); close_system(model, 0); end
    new_system(model); open_system(model);

    % Add blocks
    add_block('simulink/Sources/Constant', [model '/x0'], 'Value', '0.5');
    add_block('simulink/Sources/Constant', [model '/y0'], 'Value', '2.0');
    add_block('simulink/Continuous/Integrator', [model '/Int_x'], 'InitialCondition', 'x0');
    add_block('simulink/Continuous/Integrator', [model '/Int_y'], 'InitialCondition', 'y0');

    % MATLAB Function block computing derivatives
    add_block('simulink/User-Defined Functions/MATLAB Function', [model '/Dynamics']);
    set_param([model '/Dynamics'], 'Script', sprintf([ ...
        'function dz = fcn(z)\n' ...
        '%% z = [x; y]\n' ...
        'x = z(1); y = z(2);\n' ...
        'dx = x - x^3 - y;\n' ...
        'dy = x + %.15g*y;\n' ...
        'dz = [dx; dy];\n' ...
        'end\n'], a));

    % Mux/Demux
    add_block('simulink/Signal Routing/Mux', [model '/Mux'], 'Inputs', '2');
    add_block('simulink/Signal Routing/Demux', [model '/Demux'], 'Outputs', '2');

    % Scopes
    add_block('simulink/Sinks/XY Graph', [model '/XY']);
    add_block('simulink/Sinks/Scope', [model '/Scope']);

    % Wiring
    add_line(model, 'x0/1', 'Int_x/1');
    add_line(model, 'y0/1', 'Int_y/1');

    add_line(model, 'Int_x/1', 'Mux/1');
    add_line(model, 'Int_y/1', 'Mux/2');
    add_line(model, 'Mux/1', 'Dynamics/1');

    add_line(model, 'Dynamics/1', 'Demux/1');
    add_line(model, 'Demux/1', 'Int_x/1', 'autorouting', 'on');
    add_line(model, 'Demux/2', 'Int_y/1', 'autorouting', 'on');

    add_line(model, 'Int_x/1', 'XY/1');
    add_line(model, 'Int_y/1', 'XY/2');

    add_line(model, 'Mux/1', 'Scope/1');

    % Layout niceness
    set_param(model, 'StopTime', '12');
    save_system(model);
    fprintf('Saved Simulink model: %s.slx\n', model);
end
