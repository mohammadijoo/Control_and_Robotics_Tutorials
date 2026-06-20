% Chapter15_Lesson4.m
% Event handling and hybrid dynamics in MATLAB / Simulink-oriented style
% Example: Bouncing ball with ode45 Events + reset map loop

function Chapter15_Lesson4
    params.g = 9.81;
    params.e = 0.82;
    params.minBounceSpeed = 0.05;

    t0 = 0.0;
    tf = 8.0;
    y0 = [1.5; 0.0]; % [height; velocity]

    T_all = [];
    Y_all = [];
    impactTimes = [];

    opts = odeset('Events', @(t,y) groundEvent(t,y), ...
                  'RelTol', 1e-8, 'AbsTol', 1e-10, 'MaxStep', 0.03);

    tStart = t0;
    yStart = y0;

    while tStart < tf
        [T, Y, TE, YE, IE] = ode45(@(t,y) flow(t,y,params), [tStart tf], yStart, opts); %#ok<ASGLU>
        T_all = [T_all; T]; %#ok<AGROW>
        Y_all = [Y_all; Y]; %#ok<AGROW>

        if isempty(TE)
            break;
        end

        impactTimes(end+1,1) = TE(1); %#ok<AGROW>
        yMinus = YE(1,:).';

        if abs(yMinus(2)) < params.minBounceSpeed
            T_all = [T_all; TE(1)]; %#ok<AGROW>
            Y_all = [Y_all; 0.0, 0.0]; %#ok<AGROW>
            break;
        end

        yPlus = [0.0; -params.e * yMinus(2)];
        tStart = TE(1);
        yStart = yPlus + [0.0; 1e-12]; % avoid immediate retrigger
    end

    figure('Color', 'w');
    plot(T_all, Y_all(:,1), 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)');
    ylabel('Height (m)');
    title('Hybrid Simulation with Events (Bouncing Ball)');

    disp('Impact times:');
    disp(impactTimes);

    % Simulink note:
    % Use Integrator blocks with zero-crossing enabled, a Hit Crossing block
    % on h(t)=0, and a Triggered Subsystem / Stateflow chart to apply the reset
    % v^+ = -e v^- at each detected event.
end

function dydt = flow(~, y, params)
    dydt = [y(2); -params.g];
end

function [value, isterminal, direction] = groundEvent(~, y)
    value = y(1);      % event when height reaches zero
    isterminal = 1;    % stop integration and return control to script
    direction = -1;    % detect descending crossing only
end
