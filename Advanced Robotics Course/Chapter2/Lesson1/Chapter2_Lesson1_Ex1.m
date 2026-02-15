function [frontierOut, parentOut] = bfsStep(frontierIn, parentIn, A)
%#codegen
% A is adjacency matrix, A(i,j) = 1 if edge i->j exists.
% frontierIn is a logical vector for current frontier nodes.
% parentIn stores parent indices (0 for "none").

N = size(A,1);
frontierOut = false(N,1);
parentOut = parentIn;

for i = 1:N
    if frontierIn(i)
        for j = 1:N
            if A(i,j) ~= 0 && parentOut(j) == 0
                parentOut(j) = i;
                frontierOut(j) = true;
            end
        end
    end
end
      
