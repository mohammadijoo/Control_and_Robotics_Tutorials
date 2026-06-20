function [NextObs, Reward, IsDone, LoggedSignals] = step(this, Action)
% Extract current state from this.State
eePos = this.State.eePos;   % 3x1
goal  = this.State.goal;    % 3x1

% Apply action through robot dynamics (not shown)
[nextEePos, successFlag] = robotDynamicsStep(eePos, Action);

% Build next observation
NextObs.eePos = nextEePos;
NextObs.goal  = goal;
NextObs.success = successFlag;

% Base reward components
dist = norm(eePos - goal);
alpha = 1.0;
ctrlCoeff = 0.01;
rTask = -alpha * dist;
rCtrl = -ctrlCoeff * (Action'*Action);
rTerm = 1.0 * double(successFlag);

% Potential-based shaping (Phi = -beta * ||eePos - goal||)
beta = 0.5;
gamma = 0.99;
phiPrev = -beta * norm(eePos - goal);
phiNext = -beta * norm(nextEePos - goal);
rShape = gamma * phiNext - phiPrev;

Reward = rTask + rCtrl + rTerm + rShape;

% Termination condition
IsDone = successFlag;

LoggedSignals = [];
end
      
