function primitives = makeUnicyclePrimitives()
    % Primitive fields: v, omega, T, dt, disp, cost
    primitives(1) = simulatePrimitive(1.0,  0.0, 1.0, 0.05);
    primitives(2) = simulatePrimitive(1.0,  0.8, 1.0, 0.05);
    primitives(3) = simulatePrimitive(1.0, -0.8, 1.0, 0.05);
end

function prim = simulatePrimitive(v, omega, T, dt)
    x = 0.0; y = 0.0; th = 0.0;
    t = 0.0; cost = 0.0;
    while t < T
        x  = x  + v * cos(th) * dt;
        y  = y  + v * sin(th) * dt;
        th = th + omega * dt;
        t  = t + dt;
        cost = cost + dt;
    end
    th = wrapTo2Pi(th);
    prim.v = v;
    prim.omega = omega;
    prim.T = T;
    prim.dt = dt;
    prim.disp = [x; y; th];
    prim.cost = cost;
end

% Example: construct primitives and visualize endpoints
prims = makeUnicyclePrimitives();
figure; hold on; axis equal;
colors = lines(numel(prims));
for i = 1:numel(prims)
    dp = prims(i).disp;
    plot([0, dp(1)], [0, dp(2)], '-', 'Color', colors(i,:));
end
legend('straight', 'left', 'right');
title('Unicycle motion primitive endpoints');
      
