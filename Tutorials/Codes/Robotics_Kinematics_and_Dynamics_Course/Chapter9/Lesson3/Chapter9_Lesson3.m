function tau = torqueFromForce2R(q, f, l1, l2)
% torqueFromForce2R - Joint torques from end-effector force for a planar 2R arm.
% q  : [q1; q2]
% f  : [Fx; Fy]
% l1, l2 : link lengths

q1 = q(1); q2 = q(2);
Fx = f(1); Fy = f(2);

s1  = sin(q1);
c1  = cos(q1);
s12 = sin(q1 + q2);
c12 = cos(q1 + q2);

J = [ -l1 * s1 - l2 * s12, -l2 * s12;
       l1 * c1 + l2 * c12,  l2 * c12 ];

tau = J' * [Fx; Fy];
end

% Example usage (script):
l1 = 1; l2 = 1;
q = [0; 0];
f = [0; -10];
tau_example = torqueFromForce2R(q, f, l1, l2)
      
