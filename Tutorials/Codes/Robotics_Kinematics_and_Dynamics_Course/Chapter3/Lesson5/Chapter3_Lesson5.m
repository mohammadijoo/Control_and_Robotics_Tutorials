function [q_cont, eulZYX, nearGimbal] = robustOrientationUpdate(R, q_prev)

q_raw = rotm2quat(R);          % [w x y z]
if dot(q_prev, q_raw) < 0
    q_raw = -q_raw;
end
q_cont = quatnormalize(q_raw);

% ZYX Euler: [yaw pitch roll]
eulZYX = rotm2eul(R, "ZYX");
theta = eulZYX(2);
nearGimbal = abs(cos(theta)) < 1e-6;

end
      
