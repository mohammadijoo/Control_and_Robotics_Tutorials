% Parameter ranges
J_vals  = linspace(0.5, 2.0, 10);
B_vals  = linspace(0.2, 1.0, 10);
Kp_vals = linspace(5, 40, 8);
Kd_vals = linspace(0.1, 5, 8);

zeta_min = 0.5;
Mp_max   = 0.15;
ts_max   = 2.0;

stable_count = 0;
robust_count = 0;
total_count  = 0;

for J = J_vals
    for B = B_vals
        % Plant
        G = tf(1, [J B 0]);
        for Kp = Kp_vals
            for Kd = Kd_vals
                total_count = total_count + 1;

                % PD controller
                C = tf([Kd Kp], 1);

                T = feedback(C*G, 1);

                p = pole(T);
                if any(real(p) >= 0)
                    continue; % unstable
                end
                stable_count = stable_count + 1;

                S = stepinfo(T);
                Mp = S.Overshoot / 100;    % fraction
                ts = S.SettlingTime;

                % Approximate zeta from dominant pole
                [~, idx] = max(real(p));
                p_dom = p(idx);
                wn = abs(p_dom);
                zeta = -real(p_dom) / wn;

                if zeta >= zeta_min && Mp <= Mp_max && ts <= ts_max
                    robust_count = robust_count + 1;
                end
            end
        end
    end
end

fprintf("Stable fraction: %g\n", stable_count / total_count);
fprintf("Robust fraction: %g\n", robust_count / total_count);

% Visualization for nominal (J,B) using mesh plot
J_nom = 1.0; B_nom = 0.5;
G_nom = tf(1, [J_nom B_nom 0]);

[Kg, Kd_grid] = meshgrid(Kp_vals, Kd_vals);
sat = zeros(size(Kg));

for i = 1:numel(Kg)
    Kp = Kg(i);
    Kd = Kd_grid(i);
    C = tf([Kd Kp], 1);
    T = feedback(C*G_nom, 1);
    p = pole(T);
    if any(real(p) >= 0), continue; end
    S = stepinfo(T);
    Mp = S.Overshoot / 100;
    ts = S.SettlingTime;
    [~, idx] = max(real(p));
    p_dom = p(idx);
    wn = abs(p_dom);
    zeta = -real(p_dom) / wn;
    if zeta >= zeta_min && Mp <= Mp_max && ts <= ts_max
        sat(i) = 1;
    end
end

figure;
surf(Kp_vals, Kd_vals, sat');
xlabel("Kp"); ylabel("Kd"); zlabel("Spec OK (1/0)");
title("Spec-satisfying region at nominal J,B");
