clc; clear; close all;

% === GA bounds ===
lb = [0    0    0.5   0    0    10];   % Opción A segura
ub = [3    5    20    pi   8    40];

% === GA options ===
opts = optimoptions('ga', ...
  'PopulationSize', 24, ...
  'MaxGenerations', 40, ...
  'EliteCount', 2, ...
  'CrossoverFraction', 0.65, ...
  'CreationFcn', 'gacreationlinearfeasible', ...
  'MutationFcn', 'mutationadaptfeasible', ...  % <- fixes your warning
  'CrossoverFcn', 'crossoverscattered', ...
  'ConstraintTolerance', 1e-6, ...
  'Display','iter', 'PlotFcn',{@gaplotbestf,@gaplotstopping}, ...
  'UseParallel', true);
[bestParams,bestJ] = ga(@mga_fitness, 6, [], [], [], [], lb, ub, [], opts);

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

