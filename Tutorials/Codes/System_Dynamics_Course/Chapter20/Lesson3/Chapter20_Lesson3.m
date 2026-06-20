% Chapter20_Lesson3.m
% System Dynamics (Control Engineering) - Chapter 20, Lesson 3
% Sensitivity to Initial Conditions and Lyapunov Exponents (Intro)

clear; clc;

%% Part A) Logistic map LLE
% x_{n+1} = r x_n (1 - x_n)
% lambda ≈ (1/N) sum log | r (1 - 2 x_n) |

rList = [3.2, 3.5, 3.9, 4.0];
for r = rList
    lam = lyapunov_logistic(r, 0.234, 100000, 5000);
    fprintf('Logistic: r=%.1f, lambda ~= %.6f\n', r, lam);
end

%% Part B) Lorenz LLE (two-trajectory renormalization with RK4)
sigma = 10; rho = 28; beta = 8/3;
dt = 0.01;
T = 120;
transient = 20;
renormEvery = 10;
d0 = 1e-8;

lamL = lyapunov_lorenz_lle([1;1;1], dt, T, transient, renormEvery, d0, sigma, rho, beta, 0);
fprintf('Lorenz: lambda_max ~= %.4f 1/time-unit\n', lamL);

%% --- Functions ---
function xnext = logistic_step(x, r)
    xnext = r*x*(1-x);
end

function lam = lyapunov_logistic(r, x0, N, discard)
    x = x0;
    for i = 1:discard
        x = logistic_step(x, r);
    end

    s = 0.0;
    for i = 1:N
        fp = r*(1 - 2*x);
        s = s + log(abs(fp) + 1e-300);
        x = logistic_step(x, r);
    end
    lam = s / N;
end

function dX = lorenz_rhs(X, sigma, rho, beta)
    x = X(1); y = X(2); z = X(3);
    dX = [ sigma*(y-x);
           x*(rho - z) - y;
           x*y - beta*z ];
end

function Xn = rk4_step(rhs, X, dt, sigma, rho, beta)
    k1 = rhs(X, sigma, rho, beta);
    k2 = rhs(X + 0.5*dt*k1, sigma, rho, beta);
    k3 = rhs(X + 0.5*dt*k2, sigma, rho, beta);
    k4 = rhs(X + dt*k3, sigma, rho, beta);
    Xn = X + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end

function lam = lyapunov_lorenz_lle(X0, dt, T, transient, renormEvery, d0, sigma, rho, beta, seed)
    rng(seed);

    X = X0(:);
    u = randn(3,1); u = u / norm(u);
    Xp = X + d0*u;

    nTrans = round(transient/dt);
    for i = 1:nTrans
        X  = rk4_step(@lorenz_rhs, X,  dt, sigma, rho, beta);
        Xp = rk4_step(@lorenz_rhs, Xp, dt, sigma, rho, beta);
        dvec = Xp - X;
        d = norm(dvec);
        if d == 0
            Xp = X + d0*u;
        else
            Xp = X + d0*(dvec/d);
        end
    end

    steps = round(T/dt);
    s = 0.0;
    count = 0;
    for k = 1:steps
        X  = rk4_step(@lorenz_rhs, X,  dt, sigma, rho, beta);
        Xp = rk4_step(@lorenz_rhs, Xp, dt, sigma, rho, beta);

        if mod(k, renormEvery) == 0
            dvec = Xp - X;
            d = norm(dvec);
            if d == 0
                continue;
            end
            s = s + log(d/d0);
            count = count + 1;
            Xp = X + d0*(dvec/d);
        end
    end

    totalTime = count * renormEvery * dt;
    lam = s / totalTime;
end
