function tamp_matlab_example()
    T = 20;
    dt = 0.1;
    qStart = [0; 0];
    qGoal  = [1; 1];

    x0 = zeros(2*(T+1),1);
    for k = 0:T
        alpha = k / T;
        x0(2*k+1:2*k+2) = (1-alpha)*qStart + alpha*qGoal;
    end

    function J = costFun(x)
        J = 0;
        for k = 1:T
            qk   = x(2*(k-1)+1:2*(k-1)+2);
            qkp1 = x(2*k+1:2*k+2);
            v = (qkp1 - qk) / dt;
            J = J + v.' * v;
        end
        qT = x(2*T+1:2*T+2);
        diffGoal = qT - qGoal;
        J = J + diffGoal.' * diffGoal;
    end

    function [c, ceq] = nonlinConstraints(x)
        % Example: start configuration fixed
        q0 = x(1:2);
        ceq = [q0 - qStart];
        % c contains inequality constraints (e.g. collision distances >= 0)
        c = [];
    end

    options = optimoptions('fmincon','Algorithm','sqp','Display','iter');
    lb = -inf(2*(T+1),1);
    ub =  inf(2*(T+1),1);

    [xOpt, Jstar] = fmincon(@costFun, x0, [], [], [], [], lb, ub, @nonlinConstraints, options);
    fprintf('Optimal cost: %f\n', Jstar);

    % In a full TAMP pipeline, nonlinConstraints would call collision checking
    % and geometric grasp/place constraints, and Simulink could be used to
    % simulate closed-loop tracking of the optimized trajectory.
end
      
