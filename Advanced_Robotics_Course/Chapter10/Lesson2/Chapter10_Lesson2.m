dt = 0.1;
F = [1 0 dt 0;
     0 1 0  dt;
     0 0 1  0;
     0 0 0  1];

H = [1 0 0 0;
     0 1 0 0];

q = 1.0;
dt2 = dt^2;
dt3 = dt^3;
dt4 = dt^4;
Q1d = [0.25 * dt4, 0.5 * dt3;
       0.5 * dt3,  dt2];
Q = q * blkdiag(Q1d, Q1d);

R = 4.0 * eye(2);

x = zeros(4, 1);
P = 1e3 * eye(4);

measurements = [0 0;
                0.9 0.1;
                2.1 0.2;
                3.0 0.1];

for k = 1:size(measurements, 1)
    % Prediction
    x = F * x;
    P = F * P * F.' + Q;

    % Update
    z = measurements(k, :).';
    y = z - H * x;
    S = H * P * H.' + R;
    K = P * H.' / S;
    x = x + K * y;
    P = (eye(4) - K * H) * P;

    fprintf("Estimated position: %.3f, %.3f\n", x(1), x(2));
end
      
