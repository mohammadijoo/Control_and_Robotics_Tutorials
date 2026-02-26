modelName = "capstone_architecture";
new_system(modelName);
open_system(modelName);

add_block("simulink/Commonly Used Blocks/In1", ...
          modelName + "/Sensors");
add_block("simulink/Commonly Used Blocks/Out1", ...
          modelName + "/Actuators");

add_block("simulink/User-Defined Functions/Subsystem", ...
          modelName + "/Perception");
add_block("simulink/User-Defined Functions/Subsystem", ...
          modelName + "/Estimator");
add_block("simulink/User-Defined Functions/Subsystem", ...
          modelName + "/Planner");
add_block("simulink/User-Defined Functions/Subsystem", ...
          modelName + "/Controller");

add_line(modelName, "Sensors/1", "Perception/1");
add_line(modelName, "Perception/1", "Estimator/1");
add_line(modelName, "Estimator/1", "Planner/1");
add_line(modelName, "Planner/1", "Controller/1");
add_line(modelName, "Controller/1", "Actuators/1");

save_system(modelName);
      
