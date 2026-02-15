body = rigidBody("link1");
% inertia vector: [Ixx Iyy Izz Ixy Iyz Izx]
inertiaVec = [0.1 0.2 0.15 0 0 0];
setFixedTransform(body.Joint, trvec2tform([0 0 0])); % example
body.Inertia = [inertiaVec, m, c'];  % [Ixx Iyy Izz Ixy Iyz Izx m cx cy cz]
      
