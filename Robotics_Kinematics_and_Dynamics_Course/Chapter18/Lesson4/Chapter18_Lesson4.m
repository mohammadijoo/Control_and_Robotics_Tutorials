function time_parameterization_demo()
    T = 2.0;
    nSamples = 100;
    t = linspace(0, T, nSamples);

    s     = s_time(t, T);
    sdot  = sdot_time(t, T);
    sddot = sddot_time(t, T);

    q     = q_path(s);
    dqds  = dqds_path(s);
    d2qds2 = d2qds2_path(s);

    qdot  = dqds .* sdot.';
    qddot = d2qds2 .* (sdot.'.^2) + dqds .* sddot.';

    qdot_max = [1.0, 1.0];
    if any(abs(qdot) > qdot_max, "all")
        disp("Joint velocity limits exceeded.");
    else
        disp("Joint velocity limits satisfied.");
    end

    % Example plot
    figure; plot(t, q);
    xlabel("t [s]"); ylabel("q_i(t) [rad]");
    legend("q_1","q_2");
end

function q = q_path(s)
    q1 = cos(0.5 * pi * s);
    q2 = sin(0.5 * pi * s);
    q = [q1(:), q2(:)];
end

function dqds = dqds_path(s)
    dq1 = -0.5 * pi * sin(0.5 * pi * s);
    dq2 =  0.5 * pi * cos(0.5 * pi * s);
    dqds = [dq1(:), dq2(:)];
end

function d2qds2 = d2qds2_path(s)
    c = 0.5 * pi;
    d2q1 = -(c^2) * cos(c * s);
    d2q2 = -(c^2) * sin(c * s);
    d2qds2 = [d2q1(:), d2q2(:)];
end

function s = s_time(t, T)
    tau = t ./ T;
    s = 3 .* tau.^2 - 2 .* tau.^3;
end

function sdot = sdot_time(t, T)
    tau = t ./ T;
    sdot = (6 .* tau - 6 .* tau.^2) ./ T;
end

function sddot = sddot_time(t, T)
    tau = t ./ T;
    sddot = (6 - 12 .* tau) ./ (T^2);
end
      
