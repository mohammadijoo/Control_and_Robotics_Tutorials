
% Define project specs
spec.contact       = false;
spec.constraints   = true;
spec.uncertainty   = "medium";
spec.computeBudget = "moderate";

% Select architecture type
if ~spec.contact
    if spec.uncertainty == "low"
        archType = "JointPD";
    else
        archType = "ComputedTorque";
    end
else
    archType = "Impedance";
end

% Set controller gains based on archType
switch archType
    case "JointPD"
        Kp = 5 * ones(6,1);
        Kd = 1 * ones(6,1);
    case "ComputedTorque"
        Kp = 15 * ones(6,1);
        Kd = 4  * ones(6,1);
    case "Impedance"
        Kp = 3 * ones(6,1);
        Kd = 0.8 * ones(6,1);
end

% Push parameters to Simulink model
modelName = "capstone_robot_architecture";
set_param(modelName, "SimulationCommand", "stop");

set_param([modelName "/Controller"], ...
          "ArchType", archType, ...
          "Kp", mat2str(Kp), ...
          "Kd", mat2str(Kd));

if spec.constraints
    set_param([modelName "/SafetyLayer"], "EnableSafety", "on");
else
    set_param([modelName "/SafetyLayer"], "EnableSafety", "off");
end

% Run simulation and collect metrics
simOut = sim(modelName, "StopTime", "5");

e_q   = simOut.logsout.getElement("e_q").Values.Data;
tau   = simOut.logsout.getElement("tau").Values.Data;
sigma = simOut.logsout.getElement("sigmaSafety").Values.Data;

rho_track   = max(vecnorm(e_q, 2, 2));
rho_effort  = sqrt(mean(vecnorm(tau, 2, 2).^2));
rho_safe    = min(sigma);

metrics = struct("track", rho_track, ...
                 "effort", rho_effort, ...
                 "safe", rho_safe);
