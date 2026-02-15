import example_interfaces.srv.AddTwoInts;
import org.ros2.rcljava.client.Client;

Client<AddTwoInts> cli =
  node.createClient(AddTwoInts.class, "/add");

// build request
AddTwoInts.Request req = new AddTwoInts.Request();
req.setA(3);
req.setB(7);

// async call
cli.asyncSendRequest(req).thenAccept(res -> {
  System.out.println("sum=" + res.getSum());
});
      
