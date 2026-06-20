function tau = joint_torques_from_force_2r(q, l1, l2, f)
% q  : [q1; q2]
% l1 : link 1 length
% l2 : link 2 length
% f  : [fx; fy]

q1 = q(1);
q2 = q(2);
fx = f(1);
fy = f(2);

s1  = sin(q1);
c1  = cos(q1);
s12 = sin(q1 + q2);
c12 = cos(q1 + q2);

J = [ -l1*s1 - l2*s12,  -l2*s12;
       l1*c1 + l2*c12,   l2*c12 ];

tau = J.' * f;
end

% Example call:
% q = deg2rad([45; 30]);
% l1 = 1.0; l2 = 0.8;
% f = [10; 0];
% tau = joint_torques_from_force_2r(q, l1, l2, f)
      
