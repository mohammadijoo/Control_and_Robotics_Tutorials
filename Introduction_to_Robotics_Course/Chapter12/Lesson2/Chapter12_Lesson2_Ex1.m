% Service client for /add
cli = ros2svcclient(node, "/add", "example_interfaces/AddTwoInts");
req = ros2message(cli);
req.a = int64(3);
req.b = int64(7);

res = call(cli, req);
disp(["sum = ", num2str(res.sum)]);
      
