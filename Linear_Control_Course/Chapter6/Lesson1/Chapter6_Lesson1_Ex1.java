// Sketch (requires appropriate WPILib dependencies):
//
// import edu.wpi.first.math.system.LinearSystem;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.math.numbers.N1;
//
// LinearSystem<N1, N1, N1> armPlant =
//     LinearSystemId.createSingleJointedArmSystem(
//         /* armMomentOfInertia */ J,
//         /* armLength */ L,
//         /* armMass */ m,
//         /* motor */ motor,        // motor model
//         /* gearing */ G);
//
// // This armPlant behaves approximately as a second-order system in arm angle.
// // WPILib then provides simulation and controller classes built on top of it.
