robots(1) = struct('name','6R arm','dof_base',0,'dof_joints',6,'n_agents',1,'humanoid',false);
robots(2) = struct('name','Wheeled base','dof_base',3,'dof_joints',0,'n_agents',1,'humanoid',false);
robots(3) = struct('name','Humanoid','dof_base',6,'dof_joints',22,'n_agents',1,'humanoid',true);
robots(4) = struct('name','Swarm','dof_base',3,'dof_joints',0,'n_agents',50,'humanoid',false);

for k = 1:length(robots)
    r = robots(k);
    if r.n_agents > 1
        fam = 'Swarm';
    elseif r.humanoid
        fam = 'Humanoid';
    elseif r.dof_base > 0 && r.dof_joints == 0
        fam = 'Mobile robot';
    elseif r.dof_base == 0 && r.dof_joints > 0
        fam = 'Manipulator';
    elseif r.dof_base > 0 && r.dof_joints > 0
        fam = 'Mobile manipulator';
    else
        fam = 'Other';
    end
    fprintf('%s => %s (DoF %d)\\n', r.name, fam, r.dof_base + r.dof_joints);
end
      