function q = grasp_quality_example()
    % Two contact points on a box
    p1 = [0.0;  0.03; 0.0];
    p2 = [0.0; -0.03; 0.0];

    G = buildGraspMap({p1, p2});
    s = svd(G);
    q = min(s);
    fprintf('sigma_min(G) = %f\n', q);
end

function S = skew(v)
    x = v(1); y = v(2); z = v(3);
    S = [ 0, -z,  y;
          z,  0, -x;
         -y,  x,  0];
end

function G = buildGraspMap(contacts)
    m = numel(contacts);
    G_top = zeros(3, 3*m);
    G_bottom = zeros(3, 3*m);
    for i = 1:m
        idx = 3*(i-1) + 1;
        G_top(:, idx:idx+2) = eye(3);
        G_bottom(:, idx:idx+2) = skew(contacts{i});
    end
    G = [G_top; G_bottom];
end
      
