% Fibonacci action client
ac = ros2actionclient(node, "/fib", "example_interfaces/Fibonacci");
goal = ros2message(ac);
goal.order = int32(8);

% Send goal and wait for result
result = sendGoalAndWait(ac, goal);
disp("sequence:");
disp(result.sequence);
      
