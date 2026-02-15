function swarm_lab_matlab()
    N     = 50;
    h     = 0.05;
    R     = 2.0;
    wAtt  = 0.2;
    wRep  = 0.05;
    dMin  = 0.5;
    vMax  = 0.5;
    steps = 500;

    % Initialize positions uniformly in a square
    rng(0);
    boxSize = 10.0;
    pos = boxSize * (rand(N, 2) - 0.5);

    traj = zeros(steps + 1, N, 2);
    traj(1, :, :) = pos;

    for k = 1:steps
        pos = step_swarm(pos, h, R, wAtt, wRep, dMin, vMax);
        traj(k + 1, :, :) = pos;
    end

    % Simple trajectory plot
    figure; hold on; grid on; axis equal;
    for i = 1:N
        xy = squeeze(traj(:, i, :));
        plot(xy(:, 1), xy(:, 2));
    end
    title('Swarm trajectories (MATLAB)');
    xlabel('x'); ylabel('y');
end

function pos_new = step_swarm(pos, h, R, wAtt, wRep, dMin, vMax)
    N = size(pos, 1);
    vel = zeros(N, 2);
    R2 = R^2;

    for i = 1:N
        p_i = pos(i, :);
        att = [0.0, 0.0];
        rep = [0.0, 0.0];
        for j = 1:N
            if j == i, continue; end
            diff = pos(j, :) - p_i;
            d2 = diff * diff';
            if d2 <= R2
                att = att + diff;
                if d2 > 1e-12
                    d = sqrt(d2);
                    if d <= dMin
                        rep = rep - diff / (d^3);
                    end
                end
            end
        end
        vel(i, :) = wAtt * att + wRep * rep;
    end

    % Saturate velocities
    speeds = sqrt(sum(vel.^2, 2));
    mask = speeds > vMax;
    vel(mask, :) = vel(mask, :) .* (vMax ./ speeds(mask));

    pos_new = pos + h * vel;
end
      
