clc; clear; close all;

% === GA bounds ===
lb = [0 0  0   0 0  0];     % lower bounds
ub = [2 2  8   3 3 40];     % upper bounds

% === GA options ===
opts = optimoptions('ga', ...
    'PopulationSize', 20, ...
    'MaxGenerations', 50, ...
    'EliteCount', 2, ...
    'CrossoverFraction', 0.6, ...
    'MutationFcn', {@mutationgaussian, 0.4, 0.5}, ...
    'Display', 'iter', ...
    'PlotFcn', {@gaplotbestf}, ...
    'UseParallel', true);  % set true if you have Parallel Toolbox

% === Run GA ===
[bestParams, bestJ] = ga(@mga_fitness, 6, [], [], [], [], lb, ub, [], opts);

% === Show results ===
disp('--- Best parameters found ---');
disp('parameteres :]');
disp('[Ke_pos Kde_pos Umax_Pos Ke_theta Kde_theta Umax_theta]');
% disp([num2str(bestParams(1)) num2str(bestParams(1))])
disp(['Ke_pos    = ' num2str(bestParams(1))]);
disp(['Kde_pos   = ' num2str(bestParams(2))]);
disp(['Umax_pos  = ' num2str(bestParams(3))]);
disp(['Ke_theta  = ' num2str(bestParams(4))]);
disp(['Kde_theta = ' num2str(bestParams(5))]);
disp(['Umax_theta= ' num2str(bestParams(6))]);
disp(['Best Fitness J = ' num2str(bestJ)]);
