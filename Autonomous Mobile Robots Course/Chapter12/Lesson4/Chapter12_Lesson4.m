% Chapter12_Lesson4.m
% Graph-Based SLAM (2D pose graph) with robust kernels via IRLS (Huber).
% Educational MATLAB implementation using numeric Jacobians and Gauss-Newton.

function Chapter12_Lesson4()
    rng(1);
    N = 20;
    x = zeros(N,3);
    for k=2:N
        x(k,1) = x(k-1,1) + 0.3;
        x(k,2) = 0.0;
        x(k,3) = wrap(x(k-1,3) + 0.05);
    end

    sigmaXY = 0.05; sigmaTH = 0.02;
    Omega = diag([1/sigmaXY^2, 1/sigmaXY^2, 1/sigmaTH^2]);

    edges = {};
    for k=1:N-1
        e.i = k; e.j = k+1;
        e.z = [0.3; 0.0; 0.05];
        e.Omega = Omega;
        edges{end+1} = e;
    end
    % bad loop closure (outlier)
    e.i = 1; e.j = N; e.z = [2.0; -1.0; 1.0]; e.Omega = Omega;
    edges{end+1} = e;

    delta = 1.5;
    iters = 15;
    damping = 1e-6;

    % Fix node 1 => variables nodes 2..N
    dim = 3*(N-1);

    for it=1:iters
        H = zeros(dim,dim);
        b = zeros(dim,1);

        sumS = 0; sumRho = 0;

        for idx=1:numel(edges)
            ed = edges{idx};
            i = ed.i; j = ed.j;

            [eij, Ji, Jj] = numericJacobianEdge(x(i,:)', x(j,:)', ed.z, 1e-6);

            s = eij' * ed.Omega * eij;
            sumS = sumS + s;

            w = huberWeightFromS(s, delta);
            r = sqrt(max(s, 1e-12));
            if r <= delta
                rho = s;
            else
                rho = 2*delta*r - delta^2;
            end
            sumRho = sumRho + rho;

            W = w * ed.Omega;

            ci = nodeToCol(i, N);
            cj = nodeToCol(j, N);

            if ci > 0
                H(ci:ci+2, ci:ci+2) = H(ci:ci+2, ci:ci+2) + Ji' * W * Ji;
                b(ci:ci+2) = b(ci:ci+2) + Ji' * W * eij;
            end
            if cj > 0
                H(cj:cj+2, cj:cj+2) = H(cj:cj+2, cj:cj+2) + Jj' * W * Jj;
                b(cj:cj+2) = b(cj:cj+2) + Jj' * W * eij;
            end
            if (ci > 0) && (cj > 0)
                H(ci:ci+2, cj:cj+2) = H(ci:ci+2, cj:cj+2) + Ji' * W * Jj;
                H(cj:cj+2, ci:ci+2) = H(cj:cj+2, ci:ci+2) + Jj' * W * Ji;
            end
        end

        H = 0.5*(H+H') + damping*eye(dim);
        dx = -H \ b;

        % apply
        for node=2:N
            c = 3*(node-2) + 1;
            x(node,1) = x(node,1) + dx(c+0);
            x(node,2) = x(node,2) + dx(c+1);
            x(node,3) = wrap(x(node,3) + dx(c+2));
        end

        fprintf('iter %02d  sumS=%.3f  sumRho=%.3f  |dx|=%.3e\n', it, sumS, sumRho, norm(dx));
        if norm(dx) < 1e-8
            break;
        end
    end

    disp('Final pose of last node:');
    disp(x(end,:));

    % Simulink note:
    % - You can implement the IRLS loop with a MATLAB Function block that outputs dx,
    %   then integrate pose states with Unit Delay blocks, using a For-Iterator subsystem.
end

function c = nodeToCol(node, N)
    % node 1 fixed => return 0. Others map to 1-based column index.
    if node == 1
        c = 0;
    else
        c = 3*(node-2) + 1;
    end
end

function w = huberWeightFromS(s, delta)
    r = sqrt(max(s, 1e-12));
    if r <= delta
        w = 1.0;
    else
        w = delta / r;
    end
end

function a = wrap(a)
    a = mod(a + pi, 2*pi) - pi;
end

function z = between(xi, xj)
    dx = xj(1) - xi(1);
    dy = xj(2) - xi(2);
    c = cos(xi(3)); s = sin(xi(3));
    xr =  c*dx + s*dy;
    yr = -s*dx + c*dy;
    tr = wrap(xj(3) - xi(3));
    z = [xr; yr; tr];
end

function e = err(xi, xj, z)
    zhat = between(xi, xj);
    e = zhat - z;
    e(3) = wrap(e(3));
end

function [e0, Ji, Jj] = numericJacobianEdge(xi, xj, z, eps)
    e0 = err(xi, xj, z);
    Ji = zeros(3,3);
    Jj = zeros(3,3);
    for k=1:3
        d = zeros(3,1); d(k)=eps;
        ei = err(xi+d, xj, z);
        Ji(:,k) = (ei - e0)/eps;
        ej = err(xi, xj+d, z);
        Jj(:,k) = (ej - e0)/eps;
    end
end
