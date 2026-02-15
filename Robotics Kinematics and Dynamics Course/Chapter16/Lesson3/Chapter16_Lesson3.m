function singularity_map_3RPR()
    % Geometry
    Rb = 1.0;
    Ab = [ Rb, 0.0;
           Rb*cos(2*pi/3), Rb*sin(2*pi/3);
           Rb*cos(4*pi/3), Rb*sin(4*pi/3) ];

    Rp    = 0.5;
    alpha = [0.0; 2*pi/3; 4*pi/3];
    b_loc = [Rp*cos(alpha), Rp*sin(alpha)];

    % Grid in x-y plane at fixed orientation
    phi  = 0.3;
    xs   = linspace(-0.3, 0.3, 40);
    ys   = linspace(-0.3, 0.3, 40);
    detA = zeros(length(ys), length(xs));

    % Nominal leg lengths (for example)
    rho_nom = [1.0; 1.0; 1.0];

    for ix = 1:length(xs)
        for iy = 1:length(ys)
            x_vec = [xs(ix); ys(iy); phi];
            [A, ~] = computeAB_3RPR(x_vec, rho_nom, Ab, b_loc);
            detA(iy, ix) = det(A);
        end
    end

    figure;
    imagesc(xs, ys, log10(abs(detA)));
    set(gca, "YDir", "normal");
    colorbar;
    title("log_{10}(|det(A)|) for 3-RPR");
    xlabel("x");
    ylabel("y");
end

function [A, B] = computeAB_3RPR(x_vec, rho, Ab, b_loc)
    x   = x_vec(1);
    y   = x_vec(2);
    phi = x_vec(3);

    R = [cos(phi), -sin(phi);
         sin(phi),  cos(phi)];

    p = zeros(3, 2);
    for i = 1:3
        p(i, :) = [x, y] + (R * b_loc(i, :).').';
    end

    A = zeros(3, 3);
    B = zeros(3, 3);

    Rp = norm(b_loc(1, :));  % same for each
    for i = 1:3
        d = p(i, :) - Ab(i, :);
        ang = phi + alpha(i); %#ok<NASGU> % can be precomputed

        dp_dxi = [1.0, 0.0, -Rp * sin(phi + alpha(i));
                  0.0, 1.0,  Rp * cos(phi + alpha(i))];

        A(i, :) = 2.0 * d * dp_dxi;
        B(i, i) = -2.0 * rho(i);
    end
end
      
