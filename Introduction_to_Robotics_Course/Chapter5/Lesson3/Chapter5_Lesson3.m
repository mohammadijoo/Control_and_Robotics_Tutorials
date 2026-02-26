
l1 = 1.0; l2 = 0.7;
q1_min = -pi; q1_max = pi;
q2_min = -pi; q2_max = pi;

N = 50000;
q1 = q1_min + (q1_max - q1_min)*rand(N,1);
q2 = q2_min + (q2_max - q2_min)*rand(N,1);

x = l1*cos(q1) + l2*cos(q1 + q2);
y = l1*sin(q1) + l2*sin(q1 + q2);

figure; scatter(x,y,1,'.');
axis equal; grid on;
title('Estimated workspace of planar 2R arm');
xlabel('x'); ylabel('y');
      