// Pseudocode for Java Bullet bindings: set up world and step
DynamicsWorld world = new DiscreteDynamicsWorld(...);
world.setGravity(new Vector3(0f, 0f, -9.81f));

// Create a rigid body (box)
RigidBody box = RigidBodyFactory.createBox(mass, size, initialTransform);
world.addRigidBody(box);

float dt = 1f/240f;
for(int k=0; k<1000; k++){
    world.stepSimulation(dt, 10);
    Transform tr = box.getWorldTransform();
    System.out.println("t=" + k*dt + " z=" + tr.origin.z);
}
