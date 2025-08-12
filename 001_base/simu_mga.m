clc; clear; close all;

% === GA bounds ===
lb = [0 0  0   0 0  0];     % lower bounds
ub = [2 2  40   3 3 40];     % upper bounds
% seed = [1.2, 1.0, 6,   0.6, 2.0, 25];  % nonzero Umax_pos
% === GA options ===
opts = optimoptions('ga', ...
    'PopulationSize', 30, ...
    'MaxGenerations', 100, ...
     'CrossoverFraction',0.65, ...
    'EliteCount', 2, ...
    'MutationFcn','mutationadaptfeasible', ...
    'CreationFcn','gacreationlinearfeasible', ...
    'Display', 'iter', ...
    'PlotFcn', {@gaplotbestf}, ...
    'UseParallel', true);  % set true if you have Parallel Toolbox

% Re-seed (EDIT #4)
% initPop = lb + (ub-lb).*rand(29,6);
% initPop = [seed; initPop];
seed = [1.2, 1.0, 8,   2.0, 2.0, 35];
gaopts = optimoptions('ga', ...,
    'InitialPopulationMatrix', [seed; seed.*(0.8+0.4*rand(7,6))]);

% === Run GA ===
[bestParams, bestJ] = ga(@mga_fitness, 6, [], [], [], [], lb, ub, [], opts);

% === Show results ===
paramNames = {'Ke_pos','Kde_pos','Umax_pos','Ke_theta','Kde_theta','Umax_theta'};
assert(numel(bestParams) == numel(paramNames), 'Param count mismatch.');

namesStr  = strjoin(paramNames, ' ');
valuesStr = sprintf('%.3g ', bestParams(:).');  % full precision, force row

% === Show results ===
disp('--- Best parameters found ---');
disp('parameters :]');
disp('[Ke_pos Kde_pos Umax_pos Ke_theta Kde_theta Umax_theta]');
disp(['[' namesStr '] = [' strtrim(valuesStr) '];']);  % copy-pasteable parallel assignment
fprintf('Best Fitness J = %.3g\n', bestJ);
