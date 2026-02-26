function metrics = deployment_metrics(totalUptimeHours, nFailures, mttrHours, missionHours)
    if totalUptimeHours <= 0
        error('totalUptimeHours must be positive');
    end
    if nFailures <= 0
        nFailures = 1; % conservative
    end

    lambdaHat = nFailures / totalUptimeHours;
    mtbfHat = 1 / lambdaHat;
    availabilityHat = mtbfHat / (mtbfHat + mttrHours);
    missionSuccess = exp(-lambdaHat * missionHours);

    metrics.lambda_hat = lambdaHat;
    metrics.MTBF_hat = mtbfHat;
    metrics.availability_hat = availabilityHat;
    metrics.mission_success_prob = missionSuccess;
end

% In Simulink, you could:
% - Add Constant blocks for totalUptimeHours, nFailures, mttrHours, missionHours
% - Use a MATLAB Function block implementing deployment_metrics
% - Route outputs to Display or Scope blocks for online monitoring
      
