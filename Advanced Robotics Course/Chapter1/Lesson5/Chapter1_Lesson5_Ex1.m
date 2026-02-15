function coll = cspaceCollision(theta1, theta2)
%#codegen
persistent robot env
if isempty(robot)
    % Initialize persistent robot and environment (similar to script above)
    % ...
end
q = [theta1 theta2];
coll = checkCollision(robot, q, env, 'IgnoreSelfCollision','on');
end
      
