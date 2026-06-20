function [assignment, prices] = auctionTaskAllocation(V, epsilon, maxIters)
% V: nR x nT matrix of valuations v_ij
% epsilon: small positive scalar
% assignment: length-nR vector (robot -> task index or -1)
% prices: length-nT vector of task prices

if nargin < 2
    epsilon = 0.01;
end
if nargin < 3
    maxIters = 1000;
end

[nR, nT] = size(V);
prices = zeros(1, nT);
winners = -ones(1, nT);  % winner robot index per task
assignment = -ones(1, nR);

iter = 0;
while iter < maxIters && any(assignment == -1)
    iter = iter + 1;
    bidsPerTask = cell(1, nT);
    % local bidding
    for i = 1:nR
        if assignment(i) ~= -1
            continue;
        end
        reduced = V(i,:) - prices;
        [best, j1] = max(reduced);
        reduced(j1) = -inf;
        second = max(reduced);
        bidAmount = prices(j1) + (best - second) + epsilon;
        bidsPerTask{j1}(end+1, :) = [bidAmount, i]; %#ok<AGROW>
    end
    % resolve bids
    for j = 1:nT
        if isempty(bidsPerTask{j})
            continue;
        end
        B = bidsPerTask{j};
        [~, idx] = max(B(:,1));
        bestBid = B(idx,1);
        bestRobot = B(idx,2);
        prices(j) = bestBid;
        prevWinner = winners(j);
        if prevWinner ~= -1 && prevWinner ~= bestRobot
            assignment(prevWinner) = -1;
        end
        winners(j) = bestRobot;
        assignment(bestRobot) = j;
    end
end
      
