function active_perception_demo()
    nTheta = 360;
    thetaGrid = linspace(-pi, pi, nTheta);
    b = ones(1, nTheta) / nTheta;

    nViews = 12;
    candidateViews = linspace(-pi, pi, nViews);

    thetaTrue = 0.7;
    kappa = 4.0;

    for t = 1:6
        [aStar, ig] = choose_action(thetaGrid, b, candidateViews, kappa);
        % Simulate measurement
        p1 = measurement_likelihood(1, thetaTrue, aStar, kappa);
        z = rand() < p1;
        b = update_belief(thetaGrid, b, aStar, z, kappa);
        H = entropy_belief(b);
        fprintf('Step %d: a=%.2f, z=%d, IG=%.3f, H=%.3f\n', t, aStar, z, ig, H);
    end
end

function p = measurement_likelihood(z, theta, a, kappa)
    p1 = 1.0 ./ (1.0 + exp(-kappa .* cos(theta - a)));
    if z == 1
        p = p1;
    else
        p = 1.0 - p1;
    end
end

function H = entropy_belief(b)
    idx = b > 0;
    H = -sum(b(idx) .* log(b(idx)));
end

function [aStar, bestIG] = choose_action(thetaGrid, b, candidateViews, kappa)
    Hprior = entropy_belief(b);
    aStar = candidateViews(1);
    bestIG = -inf;
    for a = candidateViews
        Hpost = expected_entropy_after_action(thetaGrid, b, a, kappa);
        ig = Hprior - Hpost;
        if ig > bestIG
            bestIG = ig;
            aStar = a;
        end
    end
end

function Hexp = expected_entropy_after_action(thetaGrid, b, a, kappa)
    Hexp = 0.0;
    for z = 0:1
        like = measurement_likelihood(z, thetaGrid, a, kappa);
        bUnnorm = like .* b;
        pz = sum(bUnnorm);
        if pz > 0
            bPost = bUnnorm / pz;
            Hz = entropy_belief(bPost);
            Hexp = Hexp + pz * Hz;
        end
    end
end

function bNext = update_belief(thetaGrid, b, a, z, kappa)
    like = measurement_likelihood(z, thetaGrid, a, kappa);
    bUnnorm = like .* b;
    s = sum(bUnnorm);
    if s <= 0
        bNext = ones(size(b)) / numel(b);
    else
        bNext = bUnnorm / s;
    end
end
      
