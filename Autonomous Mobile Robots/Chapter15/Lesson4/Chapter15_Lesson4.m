% Chapter15_Lesson4.m
% Timed-Elastic-Band (TEB) local planning — minimal educational Gauss-Newton (numeric Jacobian).
% This script avoids specialized toolboxes (no Optimization Toolbox required).

function Chapter15_Lesson4()
    start = [0.0; 0.0; 0.0];      % [x;y;theta]
    goal  = [4.0; 2.5; 0.0];

    obstacles = [ 2.0 1.2 0.45;
                  2.8 2.0 0.35 ]; % each row: [cx cy r]

    N = 18;
    w = struct('w_smooth',10.0,'w_align',2.0,'w_time',1.0,'w_obst',50.0,'w_v',10.0,'w_w',5.0,'w_a',1.0,'w_alpha',1.0);
    lim = struct('v_max',0.8,'w_max',1.2,'a_max',0.8,'alpha_max',1.5,'dt_min',0.05,'dt_max',0.6,'d_min',0.35,'robot_radius',0.20);

    [xs, ys, th, dts] = initial_guess(start, goal, N, 0.22);

    z0 = [xs(2:end-1);
          ys(2:end-1);
          th(2:end-1);
          dts];

    residual = @(z) teb_residual(z, start, goal, N, obstacles, w, lim);

    z = gauss_newton(residual, z0, 35, 1e-3);

    [xs2, ys2, th2, dts2] = unpack(z, start, goal, N);
    fprintf('Optimized first 5 points:\n');
    for i = 1:min(5,N)
        fprintf('%02d: (%.3f, %.3f)\n', i-1, xs2(i), ys2(i));
    end
    fprintf('Total time [s]: %.3f\n', sum(dts2));

    figure; hold on; grid on; axis equal;
    plot(xs2, ys2, '-o', 'LineWidth', 2);
    plot(start(1), start(2), 'ks', 'MarkerSize', 10, 'LineWidth', 2);
    plot(goal(1), goal(2), 'k*', 'MarkerSize', 12, 'LineWidth', 2);
    thv = linspace(0, 2*pi, 200);
    for k = 1:size(obstacles,1)
        cx = obstacles(k,1); cy = obstacles(k,2); rr = obstacles(k,3);
        plot(cx + rr*cos(thv), cy + rr*sin(thv), '--');
    end
    title('Timed-Elastic-Band local planning (demo)');
end

function a = wrap_to_pi(a)
    a = mod(a + pi, 2*pi);
    if a < 0, a = a + 2*pi; end
    a = a - pi;
end

function y = hinge(x)
    y = max(0, x);
end

function [xs, ys, th, dts] = initial_guess(start, goal, N, dt0)
    xs = linspace(start(1), goal(1), N)';
    ys = linspace(start(2), goal(2), N)';
    th = zeros(N,1);
    th(1) = start(3); th(end) = goal(3);
    for i = 2:N-1
        dx = xs(i+1)-xs(i); dy = ys(i+1)-ys(i);
        th(i) = atan2(dy, dx);
    end
    dts = dt0*ones(N-1,1);
end

function [xs, ys, th, dts] = unpack(z, start, goal, N)
    nmid = N-2;
    xs = [start(1); z(1:nmid); goal(1)];
    ys = [start(2); z(nmid+1:2*nmid); goal(2)];
    th = [start(3); z(2*nmid+1:3*nmid); goal(3)];
    dts = z(3*nmid+1:end);
end

function r = teb_residual(z, start, goal, N, obstacles, w, lim)
    [xs, ys, th, dts] = unpack(z, start, goal, N);
    n_dt = N-1;

    r = [];

    % dt objective + bounds
    for i = 1:n_dt
        r(end+1,1) = sqrt(w.w_time) * dts(i);
        r(end+1,1) = sqrt(w.w_time) * hinge(lim.dt_min - dts(i));
        r(end+1,1) = sqrt(w.w_time) * hinge(dts(i) - lim.dt_max);
    end

    % smoothness
    for i = 2:N-1
        ddx = xs(i+1) - 2*xs(i) + xs(i-1);
        ddy = ys(i+1) - 2*ys(i) + ys(i-1);
        r(end+1,1) = sqrt(w.w_smooth) * ddx;
        r(end+1,1) = sqrt(w.w_smooth) * ddy;
    end

    % align
    for i = 1:N-1
        dx = xs(i+1)-xs(i); dy = ys(i+1)-ys(i);
        ang = atan2(dy, dx);
        r(end+1,1) = sqrt(w.w_align) * sin(wrap_to_pi(th(i) - ang));
    end

    % obstacle clearance
    for i = 1:N
        px = xs(i); py = ys(i);
        dmin = 1e9;
        for k = 1:size(obstacles,1)
            d = hypot(px-obstacles(k,1), py-obstacles(k,2)) - (obstacles(k,3) + lim.robot_radius);
            dmin = min(dmin, d);
        end
        r(end+1,1) = sqrt(w.w_obst) * hinge(lim.d_min - dmin);
    end

    % v,w bounds and store v,w
    v = zeros(N-1,1);
    ww = zeros(N-1,1);
    for i = 1:N-1
        dt = max(dts(i), 1e-6);
        ds = hypot(xs(i+1)-xs(i), ys(i+1)-ys(i));
        v(i) = ds/dt;
        ww(i) = wrap_to_pi(th(i+1)-th(i))/dt;

        r(end+1,1) = sqrt(w.w_v) * hinge(v(i) - lim.v_max);
        r(end+1,1) = sqrt(w.w_w) * hinge(abs(ww(i)) - lim.w_max);
    end

    % accel bounds
    for i = 1:N-2
        dtm = 0.5*(dts(i)+dts(i+1));
        dtm = max(dtm, 1e-6);
        a = (v(i+1)-v(i))/dtm;
        alpha = (ww(i+1)-ww(i))/dtm;

        r(end+1,1) = sqrt(w.w_a) * hinge(abs(a) - lim.a_max);
        r(end+1,1) = sqrt(w.w_alpha) * hinge(abs(alpha) - lim.alpha_max);
    end
end

function J = finite_diff_jac(fun, z, eps)
    r0 = fun(z);
    m = length(r0); n = length(z);
    J = zeros(m,n);
    for j = 1:n
        z1 = z; z1(j) = z1(j) + eps;
        r1 = fun(z1);
        J(:,j) = (r1-r0)/eps;
    end
end

function z = gauss_newton(fun, z0, maxIter, lam)
    z = z0;
    for it = 1:maxIter
        r = fun(z);
        cost = 0.5*(r.'*r);

        J = finite_diff_jac(fun, z, 1e-6);
        A = J.'*J + lam*eye(size(J,2));
        b = -(J.'*r);

        dz = A\b;

        step = 1.0;
        for ls = 1:10
            z_try = z + step*dz;
            r_try = fun(z_try);
            cost_try = 0.5*(r_try.'*r_try);
            if cost_try < cost
                z = z_try; break;
            end
            step = 0.5*step;
        end

        if norm(step*dz) < 1e-5
            break;
        end
    end
end
