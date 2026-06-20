% Requirement vector
r = [0.7; 0.8; 0.5; 0.9];

classes = {"IndustrialArm","MobileBase","MobileManipulator","Humanoid"};

% Capability matrix (rows are classes)
C = [0.95 0.20 0.90 0.60;
     0.30 0.90 0.40 0.70;
     0.80 0.85 0.70 0.85;
     0.75 0.70 0.55 0.90];

% Feasible indices
feasible = find(all(C >= r', 2));
disp("Feasible:"); disp(classes(feasible));

% Dominance check
dominates = @(ci,cj) all(ci >= cj) && any(ci > cj);

pareto = [];
for i = feasible'
    dom = false;
    for j = feasible'
        if i==j, continue; end
        if dominates(C(j,:), C(i,:)), dom = true; break; end
    end
    if ~dom, pareto = [pareto; i]; end
end
disp("Pareto:"); disp(classes(pareto));

% Utility parameters
w     = [0.35 0.25 0.15 0.25];
alpha = [4 4 3 6];
beta  = [6 5 4 8];
cost  = [0.7 0.4 0.6 0.9];
lambda = 0.5;

U = zeros(length(classes),1);
for i=1:length(classes)
    s = zeros(1,4);
    for j=1:4
        if C(i,j) >= r(j)
            s(j) = 1 - exp(-alpha(j)*(C(i,j)-r(j)));
        else
            s(j) = -beta(j)*(r(j)-C(i,j));
        end
    end
    U(i) = w*s' - lambda*cost(i);
end

disp("Utility scores:"); disp(table(classes', U));
[~,best] = max(U);
disp("Chosen class:"); disp(classes(best));
      